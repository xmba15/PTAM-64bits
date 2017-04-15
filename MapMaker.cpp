#include "MapMaker.h"
#include "MapPoint.h"
#include "Bundle.h"
#include "PatchFinder.h"
#include "HomographyInit.h"

#include "GCVD/image_interpolate.h"
#include "Persistence/instances.h"

#include <fstream>
#include <algorithm>

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

// Constructor sets up internal reference variable to Map.
// Most of the intialisation is done by Reset()..
MapMaker::MapMaker(Map& m, const ATANCamera &cam)
	: mMap(m), mCamera(cam)
{
	pthread = NULL;
	mbResetRequested = false;
	Reset();
	start(); 
	Persistence::GUI.RegisterCommand("SaveMap", GUICommandCallBack, this);
	Persistence::PV3::Register(mgvdWiggleScale, "MapMaker.WiggleScale", 0.1, Persistence::SILENT); // Default to 10cm between keyframes
}

void MapMaker::start() {
	std::cout << "attempting to start the map maker ..." << std::endl;
	if (!flag_IsStopped) return;

	pthread.reset(new boost::thread(&MapMaker::run, this));
	flag_StopRequest = false;
}

void MapMaker::Reset()
{
  // This is only called from within the mapmaker thread...
  mMap.Reset();
  mvFailureQueue.clear();
  while(!mqNewQueue.empty()) mqNewQueue.pop();
  mMap.vpKeyFrames.clear(); // TODO: actually erase old keyframes
  mvpKeyFrameQueue.clear(); // TODO: actually erase old keyframes
  mbBundleRunning = false;
  mbBundleConverged_Full = true;
  mbBundleConverged_Recent = true;
  mbResetDone = true;
  mbResetRequested = false;
  mbBundleAbortRequested = false;
}

void MapMaker::stop() {
	flag_StopRequest = true;
}

// CHECK_RESET is a handy macro which makes the mapmaker thread stop
// what it's doing and reset, if required.
#define CHECK_RESET if(mbResetRequested) {Reset(); continue;};

void MapMaker::run()
{
	flag_IsStopped = false;

#ifdef WIN32
	// For some reason, I get tracker thread starvation on Win32 when
	// adding key-frames. Perhaps this will help:
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_LOWEST);
#endif
	while (!flag_StopRequest)  // ShouldStop is a CVD::Thread func which return true if the thread is told to exit.
	{
		CHECK_RESET;

		// Handle any GUI commands encountered..
		while (!mvQueuedCommands.empty())
		{
			GUICommandHandler(mvQueuedCommands.begin()->sCommand, mvQueuedCommands.begin()->sParams);
			mvQueuedCommands.erase(mvQueuedCommands.begin());
		}

		if (!mMap.IsGood())  // Nothing to do if there is no map yet!
			continue;

		// From here on, mapmaker does various map-maintenance jobs in a certain priority
		// Hierarchy. For example, if there's a new key-frame to be added (QueueSize() is >0)
		// then that takes high priority.

		CHECK_RESET;
		// Should we run local bundle adjustment?
		if (!mbBundleConverged_Recent && QueueSize() == 0)
			BundleAdjustRecent();

		CHECK_RESET;
		// Are there any newly-made map points which need more measurements from older key-frames?
		if (mbBundleConverged_Recent && QueueSize() == 0)
			ReFindNewlyMade();

		CHECK_RESET;
		// Run global bundle adjustment?
		if (mbBundleConverged_Recent && !mbBundleConverged_Full && QueueSize() == 0)
			BundleAdjustAll();

		CHECK_RESET;
		// Very low priorty: re-find measurements marked as outliers
		if (mbBundleConverged_Recent && mbBundleConverged_Full && rand() % 20 == 0 && QueueSize() == 0)
			ReFindFromFailureQueue();

		CHECK_RESET;
		HandleBadPoints();

		CHECK_RESET;
		// Any new key-frames to be added?
		if (QueueSize() > 0)
			AddKeyFrameFromTopOfQueue(); // Integrate into map data struct, and process
	}

	flag_IsStopped = true;
}

// Tracker calls this to demand a reset
void MapMaker::RequestReset()
{
  mbResetDone = false;
  mbResetRequested = true;
}

bool MapMaker::ResetDone()
{
  return mbResetDone;
}

// HandleBadPoints() Does some heuristic checks on all points in the map to see if 
// they should be flagged as bad, based on tracker feedback.
void MapMaker::HandleBadPoints()
{
	if (mMap.vpPoints.size() == 0) return;
	std::vector<MapPoint::Ptr> vGoodPoints;

	// Did the tracker see this point as an outlier more often than as an inlier?
	for (unsigned int i = 0; i < mMap.vpPoints.size(); i++)
	{
		MapPoint::Ptr p = mMap.vpPoints[i];
		if (p->nMEstimatorOutlierCount > 20 && p->nMEstimatorOutlierCount > p->nMEstimatorInlierCount)
			p->bBad = true;

		if (p->bBad)
			mMap.vpPointsTrash.push_back(p);
		else
			vGoodPoints.push_back(p);
	}

	// All points marked as bad will be erased - erase all records of them
	// from keyframes in which they might have been measured.
	for (unsigned int i = 0; i < mMap.vpPointsTrash.size(); i++) {

		if (mMap.vpPointsTrash[i].use_count() == 0) continue;

		MapPoint::Ptr p = mMap.vpPointsTrash[i];

		for (unsigned int j = 0; j < mMap.vpKeyFrames.size(); j++)
		{
			KeyFrame::Ptr k = mMap.vpKeyFrames[j];
			if (k.use_count() > 0)
				if (k->mMeasurements.count(p))
					k->mMeasurements.erase(p);
		}
	}
	// Move bad points to the trash list.
	//mMap.MoveBadPointsToTrash();
	mMap.vpPoints = vGoodPoints;
	if (mMap.vpPoints.size() < 6) RequestReset();
}

MapMaker::~MapMaker()
{
	mbBundleAbortRequested = true;
	if (!flag_IsStopped) {
		stop();
		std::cout << "Waiting for map maker to exit the main loop ..." << std::endl;
		pthread->join();
	}
	std::cout << " .. mapmaker has died." << std::endl;
}

// Finds 3d coords of point in reference frame B from two z=1 plane projections
cv::Vec3d MapMaker::ReprojectPoint(RigidTransforms::SE3<> se3AfromB, const cv::Vec2d &v2A, const cv::Vec2d &v2B)
{
	cv::Matx<double, 3, 3> R = se3AfromB.get_rotation().get_matrix();
	cv::Vec3d t = se3AfromB.get_translation();

	double x1 = v2A[0], y1 = v2A[1], x2 = v2B[0], y2 = v2B[1];

	double mag2sq = x2*x2 + y2*y2, mag1sq = x1*x1 + y1*y1;

	double r00 = R(0, 0), r01 = R(0, 1), r02 = R(0, 2);
	double r10 = R(1, 0), r11 = R(1, 1), r12 = R(1, 2);
	double r20 = R(2, 0), r21 = R(2, 1), r22 = R(2, 2);

	double RtW2[3][3];
	// Column #1                  Column#2					 
	RtW2[0][0] = r00 - x2 * r20;  RtW2[0][1] = r10 - y2 * r20;
	RtW2[1][0] = r01 - x2 * r21;  RtW2[1][1] = r11 - y2 * r21;
	RtW2[2][0] = r02 - x2 * r22;  RtW2[2][1] = r12 - y2 * r22;

	// Column #3
	RtW2[0][2] = mag2sq * r20 - x2 * r00 - y2 * r10;
	RtW2[1][2] = mag2sq * r21 - x2 * r01 - y2 * r11;
	RtW2[2][2] = mag2sq * r22 - x2 * r02 - y2 * r12;


	// 2. Q = R'*W2*R + W1 (this is a PSD matrix, only the upper triangle...)
	double Q[6] = { RtW2[0][0] * r00 + RtW2[0][1] * r10 + RtW2[0][2] * r20 + 1,
		RtW2[0][0] * r01 + RtW2[0][1] * r11 + RtW2[0][2] * r21 + 0,
		RtW2[0][0] * r02 + RtW2[0][1] * r12 + RtW2[0][2] * r22 - x1,

		RtW2[1][0] * r01 + RtW2[1][1] * r11 + RtW2[1][2] * r21 + 1,
		RtW2[1][0] * r02 + RtW2[1][1] * r12 + RtW2[1][2] * r22 - y1,

		RtW2[2][0] * r02 + RtW2[2][1] * r12 + RtW2[2][2] * r22 + mag1sq };

	// 3. ksi = -R'*W2*t
	double ksi[3] = { -(RtW2[0][0] * t[0] + RtW2[0][1] * t[1] + RtW2[0][2] * t[2]),
		-(RtW2[1][0] * t[0] + RtW2[1][1] * t[1] + RtW2[1][2] * t[2]),
		-(RtW2[2][0] * t[0] + RtW2[2][1] * t[1] + RtW2[2][2] * t[2])
	};
	// ***************** inverting Q (just inlining the inversion...) ********************
	double det = +Q[0] * (Q[3] * Q[5] - Q[4] * Q[4])
		- Q[1] * (Q[1] * Q[5] - Q[4] * Q[2])
		+ Q[2] * (Q[1] * Q[4] - Q[3] * Q[2]);
	if (fabs(det) < 10E-6) {

		std::cout << "Found degenerate/ambiguous correspondence!" << std::endl;

		return cv::Vec3d(0, 0, -1); // return negative depth so that the mapmaker drops the point
	}

	// it follows that the inverse of Q will be PSD - do the upper triangle only 
	double Qinv[6];
	Qinv[0] = (Q[3] * Q[5] - Q[4] * Q[4]) / det;
	Qinv[1] = -(Q[1] * Q[5] - Q[2] * Q[4]) / det;
	Qinv[2] = (Q[1] * Q[4] - Q[2] * Q[3]) / det;

	Qinv[3] = (Q[0] * Q[5] - Q[2] * Q[2]) / det;
	Qinv[4] = -(Q[0] * Q[4] - Q[2] * Q[1]) / det;

	Qinv[5] = (Q[0] * Q[3] - Q[1] * Q[1]) / det;


	// ***************************** INVERSION DONE HERE !!! ****************************

	// FINALLY, obtaining world point M!

	cv::Vec3d  v3M(Qinv[0] * ksi[0] + Qinv[1] * ksi[1] + Qinv[2] * ksi[2],
		Qinv[1] * ksi[0] + Qinv[3] * ksi[1] + Qinv[4] * ksi[2],
		Qinv[2] * ksi[0] + Qinv[4] * ksi[1] + Qinv[5] * ksi[2]
	);

	return v3M;
}



// InitFromStereo() generates the initial match from two keyframes
// and a vector of image correspondences. Uses the 
bool MapMaker::InitFromStereo(KeyFrame::Ptr kF, KeyFrame::Ptr kS,
	std::vector<std::pair<cv::Point, cv::Point> > &vTrailMatches,
	RigidTransforms::SE3<> &se3TrackerPose)
{
	mdWiggleScale = *mgvdWiggleScale; // Cache this for the new map.

	mCamera.SetImageSize(kF->aLevels[0].im.size());

	std::vector<HomographyMatch> vMatches;
	for (unsigned int i = 0; i < vTrailMatches.size(); i++)
	{
		HomographyMatch m;
		m.v2CamPlaneFirst = mCamera.UnProject(vTrailMatches[i].first.x, vTrailMatches[i].first.y);
		m.v2CamPlaneSecond = mCamera.UnProject(vTrailMatches[i].second.x, vTrailMatches[i].second.y);
		m.m2PixelProjectionJac = mCamera.GetProjectionDerivs();
		vMatches.push_back(m);
	}

	RigidTransforms::SE3<> se3;
	bool bGood;
	HomographyInit HomographyInit;
	bGood = HomographyInit.Compute(vMatches, 5.0, se3);

	if (!bGood)
	{
		std::cout << "  Could not init from stereo pair, try again." << std::endl;
		return false;
	}

	// Check that the initialiser estimated a non-zero baseline
	double dTransMagn = cv::norm(se3.get_translation());

	if (dTransMagn == 0)
	{
		std::cout << "  Estimated zero baseline from stereo pair, try again." << std::endl;
		return false;
	}

	// change the scale of the map so the second camera is wiggleScale away from the first
	se3.get_translation() *= mdWiggleScale / dTransMagn;


	KeyFrame::Ptr pkFirst = kF;
	KeyFrame::Ptr pkSecond = kS;

	pkFirst->bFixed = true;
	pkFirst->se3CfromW = RigidTransforms::SE3<>();

	pkSecond->bFixed = false;
	pkSecond->se3CfromW = se3;

	// Construct map from the stereo matches.
	PatchFinder finder;

	for (unsigned int i = 0; i < vMatches.size(); i++)
	{
		MapPoint::Ptr p(new MapPoint());

		// Patch source stuff:
		p->pPatchSourceKF = pkFirst;
		p->nSourceLevel = 0;
		p->v3Normal_NC = cv::Vec3d(0, 0, -1);
		p->irCenter = vTrailMatches[i].first;
		p->v3Center_NC = CvUtils::backproject(mCamera.UnProject(p->irCenter.x, p->irCenter.y));
		p->v3OneDownFromCenter_NC = CvUtils::backproject(mCamera.UnProject(p->irCenter.x, p->irCenter.y + 1));
		p->v3OneRightFromCenter_NC = CvUtils::backproject(mCamera.UnProject(p->irCenter.x + 1, p->irCenter.y));
		p->v3Center_NC = CvUtils::normalize(p->v3Center_NC);
		p->v3OneDownFromCenter_NC = CvUtils::normalize(p->v3OneDownFromCenter_NC);
		p->v3OneRightFromCenter_NC = CvUtils::normalize(p->v3OneRightFromCenter_NC);
		p->RefreshPixelVectors();

		// Do sub-pixel alignment on the second image
		finder.MakeTemplateCoarseNoWarp(p);
		finder.MakeSubPixTemplate();
		finder.SetSubPixPos(cv::Vec2d(vTrailMatches[i].second.x, vTrailMatches[i].second.y));
		bool bGood = finder.IterateSubPixToConvergence(pkSecond, 10);
		if (!bGood)
		{
			p->bBad = true;
			continue;
		}

		// Triangulate point:
		cv::Vec2d v2SecondPos = finder.GetSubPixPos();
		p->v3WorldPos = ReprojectPoint(se3, vMatches[i].v2CamPlaneFirst, mCamera.UnProject(v2SecondPos));
		if (p->v3WorldPos[2] < 0.0)
		{
			p->bBad = true;
			continue;
		}

		// Not behind map? Good, then add to map.
		p->pMMData.reset(new MapMakerData());
		mMap.vpPoints.push_back(p);

		// Construct first two measurements and insert into relevant DBs:
		Measurement mFirst;
		mFirst.nLevel = 0;
		mFirst.Source = Measurement::SRC_ROOT;
		mFirst.v2RootPos = cv::Vec2d(vTrailMatches[i].first.x, vTrailMatches[i].first.y);
		mFirst.bSubPix = true;
		pkFirst->mMeasurements[p] = mFirst;
		p->pMMData->sMeasurementKFs.insert(pkFirst);

		Measurement mSecond;
		mSecond.nLevel = 0;
		mSecond.Source = Measurement::SRC_TRAIL;
		mSecond.v2RootPos = finder.GetSubPixPos();
		mSecond.bSubPix = true;
		pkSecond->mMeasurements[p] = mSecond;
		p->pMMData->sMeasurementKFs.insert(pkSecond);
	}

	if (mMap.vpPoints.size() < 4) {
		std::cout << "Too few map points to init." << std::endl;
		RequestReset();
		return false;
	}

	mMap.vpKeyFrames.push_back(pkFirst);
	mMap.vpKeyFrames.push_back(pkSecond);
	pkFirst->MakeKeyFrame_Rest();
	pkSecond->MakeKeyFrame_Rest();

	for (int i = 0; i < 5; i++) {
		BundleAdjustAll();
		if (mbResetRequested) return false;
	}

	// Estimate the feature depth distribution in the first two key-frames
	// (Needed for epipolar search)
	RefreshSceneDepth(pkFirst);
	RefreshSceneDepth(pkSecond);

	mdWiggleScaleDepthNormalized = mdWiggleScale / pkFirst->dSceneDepthMean;


	AddSomeMapPoints(0);
	AddSomeMapPoints(3);
	AddSomeMapPoints(1);
	AddSomeMapPoints(2);

	mbBundleConverged_Full = false;
	mbBundleConverged_Recent = false;

	while (!mbBundleConverged_Full)
	{
		BundleAdjustAll();
		if (mbResetRequested)
			return false;
	}

	// Rotate and translate the map so the dominant plane is at z=0:
	ApplyGlobalTransformationToMap(CalcPlaneAligner());
	mMap.bGood = true;
	se3TrackerPose = pkSecond->se3CfromW;

	std::cout << "  MapMaker: made initial map with " << mMap.vpPoints.size() << " points." << std::endl;
	return true;
}

// ThinCandidates() Thins out a key-frame's candidate list.
// Candidates are those salient corners where the mapmaker will attempt 
// to make a new map point by epipolar search. We don't want to make new points
// where there are already existing map points, this routine erases such candidates.
// Operates on a single level of a keyframe.
void MapMaker::ThinCandidates(KeyFrame::Ptr k, int nLevel)
{
	std::vector<Candidate> &vCSrc = k->aLevels[nLevel].vCandidates;
	std::vector<Candidate> vCGood;
	std::vector<cv::Point> irBusyLevelPos;
	// Make a list of `busy' image locations, which already have features at the same level
	// or at one level higher.
	for (meas_it it = k->mMeasurements.begin(); it != k->mMeasurements.end(); it++)
	{
		if (!(it->second.nLevel == nLevel || it->second.nLevel == nLevel + 1))
			continue;
		irBusyLevelPos.push_back(CvUtils::roundIL(it->second.v2RootPos / LevelScale(nLevel)));
	}

	// Only keep those candidates further than 10 pixels away from busy positions.
	unsigned int nMinMagSquared = 10 * 10;
	for (unsigned int i = 0; i < vCSrc.size(); i++)
	{
		cv::Point irC = vCSrc[i].irLevelPos;
		bool bGood = true;
		for (unsigned int j = 0; j < irBusyLevelPos.size(); j++)
		{
			cv::Point irB = irBusyLevelPos[j];

			if ((irB.x - irC.x) * (irB.x - irC.x) + (irB.y - irC.y) * (irB.y - irC.y)  < nMinMagSquared)
			{
				bGood = false;
				break;
			}
		}
		if (bGood)
			vCGood.push_back(vCSrc[i]);
	}
	vCSrc = vCGood;
}	

// Adds map points by epipolar search to the last-added key-frame, at a single
// specified pyramid level. Does epipolar search in the target keyframe as closest by
// the ClosestKeyFrame function.
void MapMaker::AddSomeMapPoints(int nLevel)
{
	KeyFrame::Ptr kSrc = mMap.vpKeyFrames[mMap.vpKeyFrames.size() - 1]; // The new keyframe
	KeyFrame::Ptr kTarget = ClosestKeyFrame(kSrc);
	Level &l = kSrc->aLevels[nLevel];

	ThinCandidates(kSrc, nLevel);

	for (unsigned int i = 0; i < l.vCandidates.size(); i++)
		AddPointEpipolar(kSrc, kTarget, nLevel, i);
}

// Rotates/translates the whole map and all keyframes
void MapMaker::ApplyGlobalTransformationToMap(RigidTransforms::SE3<> se3NewFromOld)
{
	for (unsigned int i = 0; i < mMap.vpKeyFrames.size(); i++)
		mMap.vpKeyFrames[i]->se3CfromW = mMap.vpKeyFrames[i]->se3CfromW * se3NewFromOld.inverse();

	//SO3<> so3Rot = se3NewFromOld.get_rotation();
	for (unsigned int i = 0; i < mMap.vpPoints.size(); i++)
	{
		mMap.vpPoints[i]->v3WorldPos =
			se3NewFromOld * mMap.vpPoints[i]->v3WorldPos;
		mMap.vpPoints[i]->RefreshPixelVectors();
	}
}

// Applies a global scale factor to the map
void MapMaker::ApplyGlobalScaleToMap(double dScale)
{
	for (unsigned int i = 0; i < mMap.vpKeyFrames.size(); i++)
		mMap.vpKeyFrames[i]->se3CfromW.get_translation() *= dScale;

	for (unsigned int i = 0; i < mMap.vpPoints.size(); i++)
	{
		(*mMap.vpPoints[i]).v3WorldPos *= dScale;
		(*mMap.vpPoints[i]).v3PixelRight_W *= dScale;
		(*mMap.vpPoints[i]).v3PixelDown_W *= dScale;
		(*mMap.vpPoints[i]).RefreshPixelVectors();
	}
}

// The tracker entry point for adding a new keyframe;
// the tracker thread doesn't want to hang about, so 
// just dumps it on the top of the mapmaker's queue to 
// be dealt with later, and return.
void MapMaker::AddKeyFrame(KeyFrame::Ptr k)
{
  k->pSBI = NULL; // Mapmaker uses a different SBI than the tracker, so will re-gen its own
  mvpKeyFrameQueue.push_back(k);
  if(mbBundleRunning)   // Tell the mapmaker to stop doing low-priority stuff and concentrate on this KF first.
    mbBundleAbortRequested = true;
}

// Mapmaker's code to handle incoming key-frames.
void MapMaker::AddKeyFrameFromTopOfQueue()
{
	if (mvpKeyFrameQueue.size() == 0)
		return;

	KeyFrame::Ptr pK = mvpKeyFrameQueue[0];
	mvpKeyFrameQueue.erase(mvpKeyFrameQueue.begin());
	pK->MakeKeyFrame_Rest();
	mMap.vpKeyFrames.push_back(pK);
	// Any measurements? Update the relevant point's measurement counter status map
	for (meas_it it = pK->mMeasurements.begin();
		it != pK->mMeasurements.end();
		it++)
	{
		it->first->pMMData->sMeasurementKFs.insert(pK);
		it->second.Source = Measurement::SRC_TRACKER;
	}

	// And maybe we missed some - this now adds to the map itself, too.
	ReFindInSingleKeyFrame(pK);

	AddSomeMapPoints(3);       // .. and add more map points by epipolar search.
	AddSomeMapPoints(0);
	AddSomeMapPoints(1);
	AddSomeMapPoints(2);

	mbBundleConverged_Full = false;
	mbBundleConverged_Recent = false;
}

// Tries to make a new map point out of a single candidate point
// by searching for that point in another keyframe, and triangulating
// if a match is found.
bool MapMaker::AddPointEpipolar(KeyFrame::Ptr kSrc,
	KeyFrame::Ptr kTarget,
	int nLevel,
	int nCandidate)
{
	static cv::Mat_<cv::Vec2d> imUnProj;
	static bool bMadeCache = false;
	if (!bMadeCache)
	{
		imUnProj.create(kSrc->aLevels[0].im.size());
		for (int i = 0; i < imUnProj.rows; i++)
			for (int j = 0; j < imUnProj.cols; j++) {
				imUnProj.ptr<cv::Vec2d>(i)[j] = mCamera.UnProject(j, i);
			}
		bMadeCache = true;
	}

	int nLevelScale = LevelScale(nLevel);
	Candidate &candidate = kSrc->aLevels[nLevel].vCandidates[nCandidate];
	cv::Point irLevelPos = candidate.irLevelPos;
	cv::Vec2d v2RootPos = LevelZeroPos(irLevelPos, nLevel);

	cv::Vec3d v3Ray_SC = CvUtils::backproject(mCamera.UnProject(v2RootPos));
	v3Ray_SC = CvUtils::normalize(v3Ray_SC);
	cv::Vec3d v3LineDirn_TC = kTarget->se3CfromW.get_rotation() * (kSrc->se3CfromW.get_rotation().inverse() * v3Ray_SC);

	// Restrict epipolar search to a relatively narrow depth range
	// to increase reliability
	double dMean = kSrc->dSceneDepthMean;
	double dSigma = kSrc->dSceneDepthSigma;
	double dStartDepth = max(mdWiggleScale, dMean - dSigma);
	double dEndDepth = min(40 * mdWiggleScale, dMean + dSigma);

	cv::Vec3d v3CamCenter_TC = kTarget->se3CfromW * kSrc->se3CfromW.inverse().get_translation(); // The camera end
	cv::Vec3d v3RayStart_TC = v3CamCenter_TC + dStartDepth * v3LineDirn_TC;                               // the far-away end
	cv::Vec3d v3RayEnd_TC = v3CamCenter_TC + dEndDepth * v3LineDirn_TC;                               // the far-away end


	if (v3RayEnd_TC[2] <= v3RayStart_TC[2])  // it's highly unlikely that we'll manage to get anything out if we're facing backwards wrt the other camera's view-ray
		return false;
	if (v3RayEnd_TC[2] <= 0.0)  return false;
	if (v3RayStart_TC[2] <= 0.0)
		v3RayStart_TC += v3LineDirn_TC * (0.001 - v3RayStart_TC[2] / v3LineDirn_TC[2]);

	cv::Vec2d v2A = CvUtils::pproject(v3RayStart_TC);
	cv::Vec2d v2B = CvUtils::pproject(v3RayEnd_TC);
	cv::Vec2d v2AlongProjectedLine = v2A - v2B;

	if (v2AlongProjectedLine.dot(v2AlongProjectedLine) < 0.00000001)
	{
		std::cout << "v2AlongProjectedLine too small." << std::endl;
		return false;
	}

	v2AlongProjectedLine = CvUtils::normalize(v2AlongProjectedLine);
	cv::Vec2d v2Normal(v2AlongProjectedLine[1], -v2AlongProjectedLine[0]);

	double dNormDist = v2A.dot(v2Normal);
	if (fabs(dNormDist) > mCamera.LargestRadiusInImage())
		return false;

	double dMinLen = min(v2AlongProjectedLine.dot(v2A), v2AlongProjectedLine.dot(v2B)) - 0.05;
	double dMaxLen = max(v2AlongProjectedLine.dot(v2A), v2AlongProjectedLine.dot(v2B)) + 0.05;
	if (dMinLen < -2.0)  dMinLen = -2.0;
	if (dMaxLen < -2.0)  dMaxLen = -2.0;
	if (dMinLen > 2.0)   dMinLen = 2.0;
	if (dMaxLen > 2.0)   dMaxLen = 2.0;

	// Find current-frame corners which might match this
	PatchFinder Finder;
	Finder.MakeTemplateCoarseNoWarp(kSrc, nLevel, irLevelPos);
	if (Finder.TemplateBad())  return false;

	std::vector<cv::Vec2d > &vv2Corners = kTarget->aLevels[nLevel].vImplaneCorners;
	std::vector<cv::Point> &vIR = kTarget->aLevels[nLevel].vCorners;
	if (!kTarget->aLevels[nLevel].bImplaneCornersCached)
	{
		for (unsigned int i = 0; i < vIR.size(); i++) {  // over all corners in target img..
			cv::Point p2LevelPos = CvUtils::IL(LevelZeroPos(vIR[i], nLevel));
			vv2Corners.push_back(imUnProj(p2LevelPos.y, p2LevelPos.x));
		}
		kTarget->aLevels[nLevel].bImplaneCornersCached = true;

	}

	int nBest = -1;
	int nBestZMSSD = Finder.mnMaxSSD + 1;
	double dMaxDistDiff = mCamera.OnePixelDist() * (4.0 + 1.0 * nLevelScale);
	double dMaxDistSq = dMaxDistDiff * dMaxDistDiff;

	for (unsigned int i = 0; i < vv2Corners.size(); i++)   // over all corners in target img..
	{
		cv::Vec2d v2Im = vv2Corners[i];
		double dDistDiff = dNormDist - v2Im.dot(v2Normal);
		if (dDistDiff * dDistDiff > dMaxDistSq)	continue; // skip if not along epi line
		if (v2Im.dot(v2AlongProjectedLine) < dMinLen)	continue; // skip if not far enough along line
		if (v2Im.dot(v2AlongProjectedLine) > dMaxLen)	continue; // or too far
		int nZMSSD = Finder.ZMSSDAtPoint(kTarget->aLevels[nLevel].im, vIR[i]);
		if (nZMSSD < nBestZMSSD)
		{
			nBest = i;
			nBestZMSSD = nZMSSD;
		}
	}

	if (nBest == -1)   return false;   // Nothing found.

	//  Found a likely candidate along epipolar ray
	Finder.MakeSubPixTemplate();
	Finder.SetSubPixPos(LevelZeroPos(vIR[nBest], nLevel));
	bool bSubPixConverges = Finder.IterateSubPixToConvergence(kTarget, 10);
	if (!bSubPixConverges)
		return false;

	// Now triangulate the 3d point...
	cv::Vec3d v3New;
	v3New = kTarget->se3CfromW.inverse() *
		ReprojectPoint(kSrc->se3CfromW * kTarget->se3CfromW.inverse(),
			mCamera.UnProject(v2RootPos),
			mCamera.UnProject(Finder.GetSubPixPos()));

	MapPoint::Ptr pNew(new MapPoint);
	pNew->v3WorldPos = v3New;
	pNew->pMMData.reset(new MapMakerData());

	// Patch source stuff:
	pNew->pPatchSourceKF = kSrc;
	pNew->nSourceLevel = nLevel;
	pNew->v3Normal_NC = cv::Vec3d(0, 0, -1);
	pNew->irCenter = irLevelPos;
	pNew->v3Center_NC = CvUtils::backproject(mCamera.UnProject(v2RootPos));
	pNew->v3OneRightFromCenter_NC = CvUtils::backproject(mCamera.UnProject(v2RootPos[0] + nLevelScale, v2RootPos[1]));
	pNew->v3OneDownFromCenter_NC = CvUtils::backproject(mCamera.UnProject(v2RootPos[0], v2RootPos[1] + nLevelScale));

	pNew->v3Center_NC = CvUtils::normalize(pNew->v3Center_NC);
	pNew->v3OneDownFromCenter_NC = CvUtils::normalize(pNew->v3OneDownFromCenter_NC);
	pNew->v3OneRightFromCenter_NC = CvUtils::normalize(pNew->v3OneRightFromCenter_NC);

	pNew->RefreshPixelVectors();

	mMap.vpPoints.push_back(pNew);
	mqNewQueue.push(pNew);
	Measurement m;
	m.Source = Measurement::SRC_ROOT;
	m.v2RootPos = v2RootPos;
	m.nLevel = nLevel;
	m.bSubPix = true;
	kSrc->mMeasurements[pNew] = m;

	m.Source = Measurement::SRC_EPIPOLAR;
	m.v2RootPos = Finder.GetSubPixPos();
	kTarget->mMeasurements[pNew] = m;
	pNew->pMMData->sMeasurementKFs.insert(kSrc);
	pNew->pMMData->sMeasurementKFs.insert(kTarget);
	return true;
}

double MapMaker::KeyFrameLinearDist(KeyFrame::Ptr k1, KeyFrame::Ptr k2)
{
	cv::Vec3d v3KF1_CamPos = k1->se3CfromW.inverse().get_translation();
	cv::Vec3d v3KF2_CamPos = k2->se3CfromW.inverse().get_translation();
	cv::Vec3d v3Diff = v3KF2_CamPos - v3KF1_CamPos;
	return cv::norm(v3Diff);
}

std::vector<KeyFrame::Ptr> MapMaker::NClosestKeyFrames(KeyFrame::Ptr k, unsigned int N)
{
	std::vector<std::pair<double, KeyFrame::Ptr > > vKFandScores;
	for (unsigned int i = 0; i < mMap.vpKeyFrames.size(); i++)
	{
		if (mMap.vpKeyFrames[i] == k)
			continue;
		double dDist = KeyFrameLinearDist(k, mMap.vpKeyFrames[i]);
		vKFandScores.push_back(make_pair(dDist, mMap.vpKeyFrames[i]));
	}
	if (N > vKFandScores.size())
		N = vKFandScores.size();
	partial_sort(vKFandScores.begin(), vKFandScores.begin() + N, vKFandScores.end());

	std::vector<KeyFrame::Ptr> vResult;
	for (unsigned int i = 0; i < N; i++)
		vResult.push_back(vKFandScores[i].second);
	return vResult;
}

KeyFrame::Ptr MapMaker::ClosestKeyFrame(KeyFrame::Ptr k)
{
	double dClosestDist = 9999999999.9;
	int nClosest = -1;
	for (unsigned int i = 0; i < mMap.vpKeyFrames.size(); i++)
	{
		if (mMap.vpKeyFrames[i] == k)
			continue;
		double dDist = KeyFrameLinearDist(k, mMap.vpKeyFrames[i]);
		if (dDist < dClosestDist)
		{
			dClosestDist = dDist;
			nClosest = i;
		}
	}
	assert(nClosest != -1);
	return mMap.vpKeyFrames[nClosest];
}

KeyFrame::Ptr MapMaker::ClosestKeyFrame(KeyFrame::Ptr pk, double &dClosestDist) {

	dClosestDist = 9999999999.9;
	int nClosest = -1;
	for (unsigned int i = 0; i<mMap.vpKeyFrames.size(); i++) {

		if (mMap.vpKeyFrames[i] == pk) continue;
		double dDist = KeyFrameLinearDist(pk, mMap.vpKeyFrames[i]);
		if (dDist < dClosestDist) {
			dClosestDist = dDist;
			nClosest = i;
		}
	}

	assert(nClosest != -1 && "For some reason I ended up with negative closest distance! CHECK YOUR CODE!!!!");
	return mMap.vpKeyFrames[nClosest];
}


double MapMaker::DistToNearestKeyFrame(KeyFrame::Ptr kCurrent)
{
	KeyFrame::Ptr pClosest = ClosestKeyFrame(kCurrent);
	double dDist = KeyFrameLinearDist(kCurrent, pClosest);
	return dDist;
}

bool MapMaker::NeedNewKeyFrame(KeyFrame::Ptr kCurrent)
{
	double dist;
	KeyFrame::Ptr pClosest = ClosestKeyFrame(kCurrent, dist);
	//double dDist = KeyFrameLinearDist(kCurrent, *pClosest);
	dist *= (1.0 / kCurrent->dSceneDepthMean);

	if (dist > Persistence::PV3::get<double>("MapMaker.MaxKFDistWiggleMult", 1.0, Persistence::SILENT) * mdWiggleScaleDepthNormalized)
		return true;
	return false;
}

// Perform bundle adjustment on all keyframes, all map points
void MapMaker::BundleAdjustAll()
{
	// construct the sets of kfs/points to be adjusted:
	// in this case, all of them
	std::set<KeyFrame::Ptr> sAdj;
	std::set<KeyFrame::Ptr> sFixed;
	for (unsigned int i = 0; i < mMap.vpKeyFrames.size(); i++)
		if (mMap.vpKeyFrames[i]->bFixed)
			sFixed.insert(mMap.vpKeyFrames[i]);
		else
			sAdj.insert(mMap.vpKeyFrames[i]);

	std::set<MapPoint::Ptr> sMapPoints;
	for (unsigned int i = 0; i < mMap.vpPoints.size(); i++)
		if (!mMap.vpPoints[i]->bBad)
			sMapPoints.insert(mMap.vpPoints[i]);

	BundleAdjust(sAdj, sFixed, sMapPoints, false);
}

// Peform a local bundle adjustment which only adjusts
// recently added key-frames
void MapMaker::BundleAdjustRecent()
{
	if (mMap.vpKeyFrames.size() < 8)
	{ // Ignore this unless map is big enough
		mbBundleConverged_Recent = true;
		return;
	}

	// First, make a list of the keyframes we want adjusted in the adjuster.
	// This will be the last keyframe inserted, and its four nearest neighbors
	std::set<KeyFrame::Ptr> sAdjustSet;
	KeyFrame::Ptr pkfNewest = mMap.vpKeyFrames.back();
	sAdjustSet.insert(pkfNewest);
	std::vector<KeyFrame::Ptr> vClosest = NClosestKeyFrames(pkfNewest, 4);
	for (int i = 0; i < 4; i++)
		if (vClosest[i]->bFixed == false)
			sAdjustSet.insert(vClosest[i]);

	// Now we find the set of features which they contain.
	std::set<MapPoint::Ptr> sMapPoints;
	for (std::set<KeyFrame::Ptr>::iterator iter = sAdjustSet.begin();
		iter != sAdjustSet.end();
		iter++)
	{
		std::map<MapPoint::Ptr, Measurement> &mKFMeas = (*iter)->mMeasurements;
		for (meas_it jiter = mKFMeas.begin(); jiter != mKFMeas.end(); jiter++)
			if (!jiter->first->bBad)
				sMapPoints.insert(jiter->first);
	}

	// Finally, add all keyframes which measure above points as fixed keyframes
	std::set<KeyFrame::Ptr> sFixedSet;
	for (std::vector<KeyFrame::Ptr>::iterator it = mMap.vpKeyFrames.begin(); it != mMap.vpKeyFrames.end(); it++)
	{
		if (sAdjustSet.count(*it))
			continue;
		for (meas_it jiter = (*it)->mMeasurements.begin(); jiter != (*it)->mMeasurements.end(); jiter++)
			if (sMapPoints.count(jiter->first))
			{
				sFixedSet.insert(*it);
				break;
			}
	}

	BundleAdjust(sAdjustSet, sFixedSet, sMapPoints, true);
}

// Common bundle adjustment code. This creates a bundle-adjust instance, populates it, and runs it.
void MapMaker::BundleAdjust(std::set<KeyFrame::Ptr> sAdjustSet, std::set<KeyFrame::Ptr> sFixedSet, std::set<MapPoint::Ptr> sMapPoints, bool bRecent)
{
	Bundle b(mCamera);   // Our bundle adjuster
	mbBundleRunning = true;
	mbBundleRunningIsRecent = bRecent;

	// The bundle adjuster does different accounting of keyframes and map points;
	// Translation maps are stored:
	std::map<MapPoint::Ptr, int> mPoint_BundleID;
	std::map<int, MapPoint::Ptr> mBundleID_Point;
	std::map<KeyFrame::Ptr, int> mView_BundleID;
	std::map<int, KeyFrame::Ptr> mBundleID_View;

	// Add the keyframes' poses to the bundle adjuster. Two parts: first nonfixed, then fixed.
	for (std::set<KeyFrame::Ptr>::iterator it = sAdjustSet.begin(); it != sAdjustSet.end(); it++)
	{
		int nBundleID = b.AddCamera((*it)->se3CfromW, (*it)->bFixed);
		mView_BundleID[*it] = nBundleID;
		mBundleID_View[nBundleID] = *it;
	}
	for (std::set<KeyFrame::Ptr>::iterator it = sFixedSet.begin(); it != sFixedSet.end(); it++)
	{
		int nBundleID = b.AddCamera((*it)->se3CfromW, true);
		mView_BundleID[*it] = nBundleID;
		mBundleID_View[nBundleID] = *it;
	}

	// Add the points' 3D position
	for (std::set<MapPoint::Ptr>::iterator it = sMapPoints.begin(); it != sMapPoints.end(); it++)
	{
		int nBundleID = b.AddPoint((*it)->v3WorldPos);
		mPoint_BundleID[*it] = nBundleID;
		mBundleID_Point[nBundleID] = *it;
	}

	// Add the relevant point-in-keyframe measurements
	for (unsigned int i = 0; i < mMap.vpKeyFrames.size(); i++)
	{
		if (mView_BundleID.count(mMap.vpKeyFrames[i]) == 0)
			continue;

		int nKF_BundleID = mView_BundleID[mMap.vpKeyFrames[i]];
		for (meas_it it = mMap.vpKeyFrames[i]->mMeasurements.begin();
			it != mMap.vpKeyFrames[i]->mMeasurements.end();
			it++)
		{
			if (mPoint_BundleID.count(it->first) == 0)
				continue;
			int nPoint_BundleID = mPoint_BundleID[it->first];
			b.AddMeas(nKF_BundleID, nPoint_BundleID, it->second.v2RootPos, LevelScale(it->second.nLevel) * LevelScale(it->second.nLevel));
		}
	}

	// Run the bundle adjuster. This returns the number of successful iterations
	int nAccepted = b.Compute(&mbBundleAbortRequested);

	if (nAccepted < 0)
	{
		// Crap: - LM Ran into a serious problem!
		// This is probably because the initial stereo was messed up.
		// Get rid of this map and start again! 
		std::cout << "!! MapMaker: Cholesky failure in bundle adjust. " << std::endl
			<< "   The map is probably corrupt: Ditching the map. " << std::endl;
		mbResetRequested = true;
		return;
	}

	// Bundle adjustment did some updates, apply these to the map
	if (nAccepted > 0)
	{

		for (std::map<MapPoint::Ptr, int>::iterator itr = mPoint_BundleID.begin();
			itr != mPoint_BundleID.end();
			itr++)
			itr->first->v3WorldPos = b.GetPoint(itr->second);

		for (std::map<KeyFrame::Ptr, int>::iterator itr = mView_BundleID.begin();
			itr != mView_BundleID.end();
			itr++)
			itr->first->se3CfromW = b.GetCamera(itr->second);
		if (bRecent)
			mbBundleConverged_Recent = false;
		mbBundleConverged_Full = false;
	}

	if (b.Converged())
	{
		mbBundleConverged_Recent = true;
		if (!bRecent)
			mbBundleConverged_Full = true;
	}

	mbBundleRunning = false;
	mbBundleAbortRequested = false;

	// Handle outlier measurements:
	std::vector<std::pair<int, int> > vOutliers_PC_pair = b.GetOutlierMeasurements();
	for (unsigned int i = 0; i < vOutliers_PC_pair.size(); i++)
	{
		MapPoint::Ptr pp = mBundleID_Point[vOutliers_PC_pair[i].first];
		KeyFrame::Ptr pk = mBundleID_View[vOutliers_PC_pair[i].second];
		Measurement &m = pk->mMeasurements[pp];
		if (pp->pMMData->GoodMeasCount() <= 2 || m.Source == Measurement::SRC_ROOT)   // Is the original source kf considered an outlier? That's bad.
			pp->bBad = true;
		else
		{
			// Do we retry it? Depends where it came from!!
			if (m.Source == Measurement::SRC_TRACKER || m.Source == Measurement::SRC_EPIPOLAR)
				mvFailureQueue.push_back(std::pair<KeyFrame::Ptr, MapPoint::Ptr>(pk, pp));
			else
				pp->pMMData->sNeverRetryKFs.insert(pk);
			pk->mMeasurements.erase(pp);
			pp->pMMData->sMeasurementKFs.erase(pk);
		}
	}
}

// Mapmaker's try-to-find-a-point-in-a-keyframe code. This is used to update
// data association if a bad measurement was detected, or if a point
// was never searched for in a keyframe in the first place. This operates
// much like the tracker! So most of the code looks just like in 
// TrackerData.h.
bool MapMaker::ReFind_Common(KeyFrame::Ptr k, MapPoint::Ptr p)
{
	// abort if either a measurement is already in the map, or we've
	// decided that this point-kf combo is beyond redemption
	if (p->pMMData->sMeasurementKFs.count(k)
		|| p->pMMData->sNeverRetryKFs.count(k))
		return false;

	static PatchFinder Finder;
	cv::Vec3d v3Cam = k->se3CfromW * p->v3WorldPos;
	if (v3Cam[2] < 0.001)
	{
		p->pMMData->sNeverRetryKFs.insert(k);
		return false;
	}
	cv::Vec2d v2ImPlane = CvUtils::pproject(v3Cam);
	if (cv::norm(v2ImPlane) > mCamera.LargestRadiusInImage())
	{
		p->pMMData->sNeverRetryKFs.insert(k);
		return false;
	}

	cv::Vec2d v2Image = mCamera.Project(v2ImPlane);
	if (mCamera.Invalid())
	{
		p->pMMData->sNeverRetryKFs.insert(k);
		return false;
	}

	cv::Size irImageSize = k->aLevels[0].im.size();
	if (v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > k->aLevels[0].im.cols - 1 || v2Image[1] > k->aLevels[0].im.rows - 1)
	{
		p->pMMData->sNeverRetryKFs.insert(k);
		return false;
	}

	cv::Matx<double, 2, 2> m2CamDerivs = mCamera.GetProjectionDerivs();
	Finder.MakeTemplateCoarse(p, k->se3CfromW, m2CamDerivs);

	if (Finder.TemplateBad())
	{
		p->pMMData->sNeverRetryKFs.insert(k);
		return false;
	}

	bool bFound = Finder.FindPatchCoarse(v2Image, k, 4);  // Very tight search radius!
	if (!bFound)
	{
		p->pMMData->sNeverRetryKFs.insert(k);
		return false;
	}

	// If we found something, generate a measurement struct and put it in the map
	Measurement m;
	m.nLevel = Finder.GetLevel();
	m.Source = Measurement::SRC_REFIND;

	if (Finder.GetLevel() > 0)
	{
		Finder.MakeSubPixTemplate();
		Finder.IterateSubPixToConvergence(k, 8);
		m.v2RootPos = Finder.GetSubPixPos();
		m.bSubPix = true;
	}
	else
	{
		m.v2RootPos = Finder.GetCoarsePosAsVector();
		m.bSubPix = false;
	}

	if (k->mMeasurements.count(p))
	{
		assert(0); // This should never happen, we checked for this at the start.
	}
	k->mMeasurements[p] = m;
	p->pMMData->sMeasurementKFs.insert(k);
	return true;
}

// A general data-association update for a single keyframe
// Do this on a new key-frame when it's passed in by the tracker
int MapMaker::ReFindInSingleKeyFrame(KeyFrame::Ptr k)
{
	std::vector<MapPoint::Ptr> vToFind;
	for (unsigned int i = 0; i < mMap.vpPoints.size(); i++)
		vToFind.push_back(mMap.vpPoints[i]);

	int nFoundNow = 0;
	for (unsigned int i = 0; i < vToFind.size(); i++)
		if (ReFind_Common(k, vToFind[i]))
			nFoundNow++;

	return nFoundNow;
}

// When new map points are generated, they're only created from a stereo pair
// this tries to make additional measurements in other KFs which they might
// be in.
void MapMaker::ReFindNewlyMade()
{
	if (mqNewQueue.empty())
		return;
	int nFound = 0;
	int nBad = 0;
	while (!mqNewQueue.empty() && mvpKeyFrameQueue.size() == 0)
	{
		MapPoint::Ptr pNew = mqNewQueue.front();
		mqNewQueue.pop();
		if (pNew->bBad)
		{
			nBad++;
			continue;
		}
		for (unsigned int i = 0; i < mMap.vpKeyFrames.size(); i++)
			if (ReFind_Common(mMap.vpKeyFrames[i], pNew))
				nFound++;
	}
}

// Dud measurements get a second chance.
void MapMaker::ReFindFromFailureQueue()
{
	if (mvFailureQueue.size() == 0)
		return;
	std::sort(mvFailureQueue.begin(), mvFailureQueue.end());
	int nFound = 0;

	for (unsigned int i = 0; i < mvFailureQueue.size(); i++) {
		std::pair<KeyFrame::Ptr, MapPoint::Ptr> KF_MP = mvFailureQueue[i];
		if (KF_MP.first.use_count() > 0) {
			if (KF_MP.second.use_count() > 0)
				if (ReFind_Common(KF_MP.first, KF_MP.second)) nFound++;
		}
	}
	mvFailureQueue.clear();
}

// Is the tracker's camera pose in cloud-cuckoo land?
bool MapMaker::IsDistanceToNearestKeyFrameExcessive(KeyFrame::Ptr kCurrent)
{
	return DistToNearestKeyFrame(kCurrent) > mdWiggleScale * 10.0;
}

// Find a dominant plane in the map, find an SE3<> to put it as the z=0 plane
RigidTransforms::SE3<> MapMaker::CalcPlaneAligner()
{
	unsigned int nPoints = mMap.vpPoints.size();
	if (nPoints < 10)
	{
		std::cout << "  MapMaker: CalcPlane: too few points to calc plane." << std::endl;
		return RigidTransforms::SE3<>();
	}

	int nRansacs = Persistence::PV3::get<int>("MapMaker.PlaneAlignerRansacs", 100, Persistence::HIDDEN | Persistence::SILENT);
	cv::Vec3d v3BestMean;
	cv::Vec3d v3BestNormal;
	double dBestDistSquared = 9999999999999999.9;

	for (int i = 0; i < nRansacs; i++)
	{
		int nA = rand() % nPoints;
		int nB = nA;
		int nC = nA;
		while (nB == nA)
			nB = rand() % nPoints;
		while (nC == nA || nC == nB)
			nC = rand() % nPoints;

		cv::Vec3d v3Mean = 0.33333333 * (mMap.vpPoints[nA]->v3WorldPos +
			mMap.vpPoints[nB]->v3WorldPos +
			mMap.vpPoints[nC]->v3WorldPos);

		cv::Vec3d v3CA = mMap.vpPoints[nC]->v3WorldPos - mMap.vpPoints[nA]->v3WorldPos;
		cv::Vec3d v3BA = mMap.vpPoints[nB]->v3WorldPos - mMap.vpPoints[nA]->v3WorldPos;
		cv::Vec3d v3Normal = v3CA ^ v3BA;

		double dNormalmag = cv::norm(v3Normal);
		if (dNormalmag == 0)
			continue;
		v3Normal *= (1.0 / dNormalmag);

		double dSumError = 0.0;
		for (unsigned int i = 0; i < nPoints; i++)
		{
			cv::Vec3d v3Diff = mMap.vpPoints[i]->v3WorldPos - v3Mean;
			double dDistSq = v3Diff.dot(v3Diff);
			if (dDistSq == 0.0)
				continue;
			double dNormDist = fabs(v3Diff.dot(v3Normal));

			if (dNormDist > 0.05)
				dNormDist = 0.05;
			dSumError += dNormDist;
		}
		if (dSumError < dBestDistSquared)
		{
			dBestDistSquared = dSumError;
			v3BestMean = v3Mean;
			v3BestNormal = v3Normal;
		}
	}

	// Done the ransacs, now collect the supposed inlier set
	std::vector<cv::Vec3d> vv3Inliers;
	for (unsigned int i = 0; i < nPoints; i++)
	{
		cv::Vec3d v3Diff = mMap.vpPoints[i]->v3WorldPos - v3BestMean;
		double dDistSq = v3Diff.dot(v3Diff);
		if (dDistSq == 0.0)
			continue;
		double dNormDist = fabs(v3Diff.dot(v3BestNormal));
		if (dNormDist < 0.05)
			vv3Inliers.push_back(mMap.vpPoints[i]->v3WorldPos);
	}

	// With these inliers, calculate mean and cov
	cv::Vec3d v3MeanOfInliers(0, 0, 0);
	for (unsigned int i = 0; i < vv3Inliers.size(); i++)
		v3MeanOfInliers += vv3Inliers[i];
	v3MeanOfInliers *= (1.0 / vv3Inliers.size());

	cv::Matx<double, 3, 3> m3Cov = cv::Matx<double, 3, 3>::zeros();
	cv::Vec3d v3Diff;
	for (unsigned int i = 0; i < vv3Inliers.size(); i++)
	{
		v3Diff = vv3Inliers[i] - v3MeanOfInliers;
		m3Cov += v3Diff * v3Diff.t();
	}

	cv::Matx<double, 3, 3> U, Vt;
	cv::Matx<double, 3, 1> w;
	cv::SVD::compute(m3Cov, w, U, Vt);
	cv::Vec3d v3Normal(Vt(2, 0), Vt(2, 1), Vt(2, 2));
	if (v3Normal[2] > 0) v3Normal = -v3Normal;

	cv::Matx<double, 3, 3> m3Rot = cv::Matx<double, 3, 3>::eye();

	m3Rot(2, 0) = v3Normal[0]; m3Rot(2, 1) = v3Normal[1]; m3Rot(2, 2) = v3Normal[2];
	double dot13 = v3Normal[0];
	m3Rot(0, 0) = m3Rot(0, 0) - dot13 * v3Normal[0]; m3Rot(0, 1) = m3Rot(0, 1) - dot13 * v3Normal[1]; m3Rot(0, 2) = m3Rot(0, 2) - dot13 * v3Normal[2];
	double norm1 = cv::norm(m3Rot.row(0));
	m3Rot(0, 0) /= norm1; m3Rot(0, 1) /= norm1; m3Rot(0, 2) /= norm1;

	m3Rot(1, 0) = -v3Normal[2] * m3Rot(0, 1) + v3Normal[1] * m3Rot(0, 2);
	m3Rot(1, 1) = v3Normal[2] * m3Rot(0, 0) - v3Normal[0] * m3Rot(0, 2);
	m3Rot(1, 2) = -v3Normal[1] * m3Rot(0, 0) + v3Normal[0] * m3Rot(0, 1);

	RigidTransforms::SE3<> se3Aligner;
	se3Aligner.get_rotation().get_matrix() = m3Rot;

	cv::Vec3d v3RMean = se3Aligner * v3MeanOfInliers;
	se3Aligner.get_translation() = -v3RMean;

	return se3Aligner;
}

// Calculates the depth(z-) distribution of map points visible in a keyframe
// This function is only used for the first two keyframes - all others
// get this filled in by the tracker
void MapMaker::RefreshSceneDepth(KeyFrame::Ptr pKF)
{
  double dSumDepth = 0.0;
  double dSumDepthSquared = 0.0;
  int nMeas = 0;
  for(meas_it it = pKF->mMeasurements.begin(); it!=pKF->mMeasurements.end(); it++)
    {
      MapPoint &point = *it->first;
      cv::Vec3d v3PosK = pKF->se3CfromW * point.v3WorldPos;
      dSumDepth += v3PosK[2];
      dSumDepthSquared += v3PosK[2] * v3PosK[2];
      nMeas++;
    }
 
  assert(nMeas > 2); // If not then something is seriously wrong with this KF!!
  pKF->dSceneDepthMean = dSumDepth / nMeas;
  pKF->dSceneDepthSigma = sqrt((dSumDepthSquared / nMeas) - (pKF->dSceneDepthMean) * (pKF->dSceneDepthMean));
}

void MapMaker::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
	Command c;
	c.sCommand = sCommand;
	c.sParams = sParams;
	((MapMaker*)ptr)->mvQueuedCommands.push_back(c);
}

void MapMaker::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
	if (sCommand == "SaveMap")
	{
		std::cout << "  MapMaker: Saving the map.... " << std::endl;
		std::ofstream ofs("map.dump");
		for (unsigned int i = 0; i < mMap.vpPoints.size(); i++)
		{
			ofs << mMap.vpPoints[i]->v3WorldPos << "  ";
			ofs << mMap.vpPoints[i]->nSourceLevel << endl;
		}
		ofs.close();

		for (unsigned int i = 0; i < mMap.vpKeyFrames.size(); i++)
		{
			std::ostringstream ost1;
			ost1 << "keyframes/" << i << ".jpg";
			//	  img_save(mMap.vpKeyFrames[i]->aLevels[0].im, ost1.str());

			std::ostringstream ost2;
			ost2 << "keyframes/" << i << ".info";
			std::ofstream ofs2;
			ofs2.open(ost2.str().c_str());
			ofs2 << mMap.vpKeyFrames[i]->se3CfromW << endl;
			ofs2.close();
		}
		std::cout << "  ... done saving map." << std::endl;
		return;
	}

	std::cout << "! MapMaker::GUICommandHandler: unhandled command " << sCommand << std::endl;
	exit(1);
}
