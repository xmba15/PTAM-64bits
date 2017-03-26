#include "Bundle.h"
#include "MEstimator.h"
#include "GCVD/Addedutils.h"

#include <fstream>
#include <iomanip>

#include "Persistence/instances.h"

#ifdef WIN32
inline bool isnan(double d) {return !(d==d);}
#endif

#define cout if(*mgvnBundleCout) std::cout

// Some inlines which replace standard matrix multiplications 
// with LL-triangle-only versions.
inline void BundleTriangle_UpdateM6U_LL(cv::Matx<double, 6, 6> &m6U,
	cv::Matx<double, 2, 6> &m26A)
{
	for (int r = 0; r < 6; r++)
		for (int c = 0; c <= r; c++)
			m6U(r, c) += m26A(0, r) * m26A(0, c) + m26A(1, r) * m26A(1, c);
}

inline void BundleTriangle_UpdateM3V_LL(cv::Matx<double, 3, 3> &m3V, cv::Matx<double, 2, 3> &m23B)
{
	for (int r = 0; r < 3; r++)
		for (int c = 0; c <= r; c++)
			m3V(r, c) += m23B(0, r) * m23B(0, c) + m23B(1, r) * m23B(1, c);
}

// Constructor copies MapMaker's camera parameters
Bundle::Bundle(const ATANCamera &TCam)
	: mCamera(TCam)
{
	mnCamsToUpdate = 0;
	mnStartRow = 0;
	Persistence::PV3::Register(mgvnMaxIterations, "Bundle.MaxIterations", 20, Persistence::SILENT);
	Persistence::PV3::Register(mgvdUpdateConvergenceLimit, "Bundle.UpdateSquaredConvergenceLimit", 1e-06, Persistence::SILENT);
	Persistence::PV3::Register(mgvnBundleCout, "Bundle.Cout", 0, Persistence::SILENT);
}

// Add a camera to the system, return value is the bundle adjuster's ID for the camera
int Bundle::AddCamera(RigidTransforms::SE3<> se3CamFromWorld, bool bFixed)
{
	int n = mvCameras.size();
	Camera c;
	c.bFixed = bFixed;
	c.se3CfW = se3CamFromWorld;
	if (!bFixed)
	{
		c.nStartRow = mnStartRow;
		mnStartRow += 6;
		mnCamsToUpdate++;
	}
	else
		c.nStartRow = -999999999;
	mvCameras.push_back(c);

	return n;
}

// Add a map point to the system, return value is the bundle adjuster's ID for the point
int Bundle::AddPoint(cv::Vec3d v3Pos)
{
	int n = mvPoints.size();
	BAPoint p;
	if (isnan(v3Pos.dot(v3Pos)))
	{
		std::cerr << " You sucker, tried to give me a nan " << v3Pos << std::endl;
		v3Pos = cv::Vec3d(0, 0, 0);
	}
	p.v3Pos = v3Pos;
	mvPoints.push_back(p);
	return n;
}

// Add a measurement of one point with one camera
void Bundle::AddMeas(int nCam, int nPoint, cv::Vec2d v2Pos, double dSigmaSquared)
{
	assert(nCam < (int)mvCameras.size());
	assert(nPoint < (int)mvPoints.size());
	mvPoints[nPoint].nMeasurements++;
	mvPoints[nPoint].sCameras.insert(nCam);
	Meas m;
	m.p = nPoint;
	m.c = nCam;
	m.v2Found = v2Pos;
	m.dSqrtInvNoise = sqrt(1.0 / dSigmaSquared);
	mMeasList.push_back(m);
}

// Zero temporary quantities stored in cameras and points
void Bundle::ClearAccumulators()
{
	for (size_t i = 0; i < mvPoints.size(); ++i)
	{
		mvPoints[i].m3V = cv::Matx<double, 3, 3>::zeros();
		mvPoints[i].v3EpsilonB = cv::Vec3d(0, 0, 0);
	}
	for (size_t i = 0; i < mvCameras.size(); ++i)
	{
		mvCameras[i].m6U = cv::Matx<double, 6, 6>::zeros();
		mvCameras[i].v6EpsilonA = cv::Vec<double, 6>::all(0);
	}
}

// Perform bundle adjustment. The parameter points to a signal bool 
// which mapmaker will set to high if another keyframe is incoming
// and bundle adjustment needs to be aborted.
// Returns number of accepted iterations if all good, negative 
// value for big error.
int Bundle::Compute(bool *pbAbortSignal)
{
	mpbAbortSignal = pbAbortSignal;

	// Some speedup data structures
	GenerateMeasLUTs();
	GenerateOffDiagScripts();

	// Initially behave like gauss-newton
	mdLambda = 0.0001;
	mdLambdaFactor = 2.0;
	mbConverged = false;
	mbHitMaxIterations = false;
	mnCounter = 0;
	mnAccepted = 0;

	// What MEstimator are we using today?
	static Persistence::pvar3<string> gvsMEstimator("BundleMEstimator", "Tukey", Persistence::SILENT);

	while (!mbConverged && !mbHitMaxIterations && !*pbAbortSignal)
	{
		bool bNoError;
		if (*gvsMEstimator == "Cauchy")
			bNoError = Do_LM_Step<Cauchy>(pbAbortSignal);
		else if (*gvsMEstimator == "Tukey")
			bNoError = Do_LM_Step<Tukey>(pbAbortSignal);
		else if (*gvsMEstimator == "Huber")
			bNoError = Do_LM_Step<Huber>(pbAbortSignal);
		else
		{
			cout << "Invalid BundleMEstimator selected !! " << std::endl;
			cout << "Defaulting to Tukey." << std::endl;
			*gvsMEstimator = "Tukey";
			bNoError = Do_LM_Step<Tukey>(pbAbortSignal);
		};

		if (!bNoError)
			return -1;
	}

	if (mbHitMaxIterations)
		cout << "  Hit max iterations." << std::endl;
	cout << "Final Sigma Squared: " << mdSigmaSquared << " (= " << sqrt(mdSigmaSquared) / 4.685 << " pixels.)" << std::endl;
	return mnAccepted;
}

// Reproject a single measurement, find error
inline void Bundle::ProjectAndFindSquaredError(Meas &meas)
{
  Camera &cam = mvCameras[meas.c];
  BAPoint &point = mvPoints[meas.p];
  
  // Project the point.
  meas.v3Cam = cam.se3CfW * point.v3Pos;
  if(meas.v3Cam[2] <= 0)
    {
      meas.bBad = true;
      return;
    }
  meas.bBad = false;
  cv::Vec2d v2ImPlane = CvUtils::pproject(meas.v3Cam);
  cv::Vec2d v2Image   = mCamera.Project(v2ImPlane);
  meas.m2CamDerivs = mCamera.GetProjectionDerivs();
  meas.v2Epsilon = meas.dSqrtInvNoise * (meas.v2Found - v2Image);
  meas.dErrorSquared = meas.v2Epsilon.dot(meas.v2Epsilon);
}

template<class MEstimator>
bool Bundle::Do_LM_Step(bool *pbAbortSignal)
{
	// Reset all accumulators to zero
	ClearAccumulators();

	//  Do a LM step according to Hartley and Zisserman Algo A6.4 in MVG 2nd Edition
	//  Actual work starts a bit further down - first we have to work out the 
	//  projections and errors for each point, so we can do tukey reweighting
	std::vector<double> vdErrorSquared;
	for (std::list<Meas>::iterator itr = mMeasList.begin(); itr != mMeasList.end(); itr++)
	{
		Meas &meas = *itr;
		ProjectAndFindSquaredError(meas);
		if (!meas.bBad)
			vdErrorSquared.push_back(meas.dErrorSquared);
	}

	// Projected all points and got vector of errors; find the median, 
	// And work out robust estimate of sigma, then scale this for the tukey
	// estimator
	mdSigmaSquared = MEstimator::FindSigmaSquared(vdErrorSquared);

	// Initially the median error might be very small - set a minimum
	// value so that good measurements don't get erased!
	static Persistence::pvar3<double> gvdMinSigma("Bundle.MinTukeySigma", 0.4, Persistence::SILENT);
	const double dMinSigmaSquared = (*gvdMinSigma) * (*gvdMinSigma);
	if (mdSigmaSquared < dMinSigmaSquared)
		mdSigmaSquared = dMinSigmaSquared;


	//  OK - good to go! weights can now be calced on second run through the loop.
	//  Back to Hartley and Zisserman....
	//  `` (i) Compute the derivative matrices Aij = [dxij/daj]
	//      and Bij = [dxij/dbi] and the error vectors eij''
	//
	//  Here we do this by looping over all measurements
	// 
	//  While we're here, might as well update the accumulators U, ea, B, eb
	//  from part (ii) as well, and let's calculate Wij while we're here as well.

	double dCurrentError = 0.0;
	for (std::list<Meas>::iterator itr = mMeasList.begin(); itr != mMeasList.end(); itr++)
	{
		Meas &meas = *itr;
		Camera &cam = mvCameras[meas.c];
		Point &point = mvPoints[meas.p];

		// Project the point.
		// We've done this before - results are still cached in meas.
		if (meas.bBad)
		{
			dCurrentError += 1.0;
			continue;
		};

		// What to do with the weights? The easiest option is to independently weight
		// The two jacobians, A and B, with sqrt of the tukey weight w;
		// And also weight the error vector v2Epsilon.
		// That makes everything else automatic.
		// Calc the square root of the tukey weight:
		double dWeight = MEstimator::SquareRootWeight(meas.dErrorSquared, mdSigmaSquared);
		// Re-weight error:
		meas.v2Epsilon = dWeight * meas.v2Epsilon;

		if (dWeight == 0)
		{
			meas.bBad = true;
			dCurrentError += 1.0;
			continue;
		}

		dCurrentError += MEstimator::ObjectiveScore(meas.dErrorSquared, mdSigmaSquared);

		// To re-weight the jacobians, I'll just re-weight the camera param matrix
		// This is only used for the jacs and will save a few fmuls
		cv::Matx<double, 2, 2> m2CamDerivs = dWeight * meas.m2CamDerivs;

		const double dOneOverCameraZ = 1.0 / meas.v3Cam[2];
		const Vec4d v4Cam = CvUtils::backproject(meas.v3Cam);

		// Calculate A: (the proj derivs WRT the camera)
		if (cam.bFixed)
			meas.m26A = cv::Matx<double, 2, 6>::zeros();
		else
		{
			for (int m = 0; m < 6; m++)
			{
				const cv::Vec4d v4Motion = RigidTransforms::SE3<>::generator_field(m, v4Cam);
				cv::Vec2d v2CamFrameMotion((v4Motion[0] - v4Cam[0] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ,
					(v4Motion[1] - v4Cam[1] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ);
				meas.m26A(0, m) = meas.dSqrtInvNoise * (m2CamDerivs(0, 0) * v2CamFrameMotion[0]
					+ m2CamDervis(0, 1) * v2CamFrameMotion[1]);
				meas.m26A(1, m) = meas.dSqrtInvNoise * (m2CamDervis(1, 0) * v2CamFrameMotion[0]
					+ m2CamDervis(1, 1) * v2CamFrameMotion[1]);
			}
		}

		// Calculate B: (the proj derivs WRT the point)
		for (int m = 0; m < 3; m++)
		{
			const cv::Matx<double, 3, 1> v3Motion = cam.se3CfW.get_rotation().get_matrix().col(m);
			cv::Vec2d v2CamFrameMotion((v3Motion[0] - v4Cam[0] * v3Motion[2] * dOneOverCameraZ) * dOneOverCameraZ,
				(v3Motion[1] - v4Cam[1] * v3Motion[2] * dOneOverCameraZ) * dOneOverCameraZ);
			meas.m23B(0, m) = meas.dSqrtInvNoise * (m2CamDerivs(0, 0) * v2CamFrameMotion[0] +
				m2CamDervis(0, 1) * v2CamFrameMotion[1]);
			meas.m23B(1, m) = meas.dSqrtInvNoise * (m2CamDervis(1, 0) * v2CamFrameMotion[0] +
				m2CamDervis(1, 1) * v2CamFrameMotion[1);
		}
		// Update the accumulators
		if (!cam.bFixed)
		{
			//	  cam.m6U += meas.m26A.T() * meas.m26A; 	  // SLOW SLOW this matrix is symmetric
			BundleTriangle_UpdateM6U_LL(cam.m6U, meas.m26A);
			cam.v6EpsilonA += meas.m26A.t() * meas.v2Epsilon;
			// NOISE COVAR OMITTED because it's the 2-Identity
		}

		//            point.m3V += meas.m23B.T() * meas.m23B;             // SLOW-ish this is symmetric too
		BundleTriangle_UpdateM3V_LL(point.m3V, meas.m23B);
		point.v3EpsilonB += meas.m23B.t() * meas.v2Epsilon;

		if (cam.bFixed)
			meas.m63W = cv::Matx<double, 6, 3>::zeros();
		else
			meas.m63W = meas.m26A.t() * meas.m23B;
	}

	// OK, done (i) and most of (ii) except calcing Yij; this depends on Vi, which should 
	// be finished now. So we can find V*i (by adding lambda) and then invert.
	// The next bits depend on mdLambda! So loop this next bit until error goes down.
	double dNewError = dCurrentError + 9999;
	while (dNewError > dCurrentError && !mbConverged && !mbHitMaxIterations && !*pbAbortSignal)
	{
		// Rest of part (ii) : find V*i inverse
		for (std::vector<BAPoint>::iterator itr = mvPoints.begin(); itr != mvPoints.end(); itr++)
		{
			BAPoint &point = *itr;
			cv::Matx<double, 3, 3> m3VStar = point.m3V;
			if (m3VStar[0][0] * m3VStar[1][1] * m3VStar[2][2] == 0)
				point.m3VStarInv = cv::Matx<double, 3, 3>::zeros();
			else
			{
				m3VStar(0, 0) *= (1.0 + mdLambda); m3VStar(1, 1) *= (1.0 + mdLambda); m3VStar(2, 2) *= (1.0 + mdLambda);

				if (fabs(CvUtils::M3Symm_LT_Det(m3VStar)) < 10e-6) {
					m3VStar(0, 1) = m3VStar(1, 0);
					m3VStar(0, 2) = m3VStar(2, 0);
					m3VStar(1, 2) = m3VStar(2, 1);

					cv::Matx<double, 3, 3> U_, Vt_;
					cv::Matx<double, 3, 1> w_;
					cv::SVD::compute(m3VStar, w_, U_, Vt_);
					cv::Matx<double, 3, 3> S_;
					for (int d_ = 0; d_ < 3; d_++) {
						S_(d_, 0) = S_(d_, 1) = S_(d_, 2);
						if (w_(d_, 0) != 0) S_(d_, d_) = 1.0 / w_(d_, 0);
					}
					point.m3VStarInv = Vt_.t() * S_ * U_.t();
				}
				else
					point.m3VStarInv = CvUtils::M3Symm_LT_Inverse(m3VStar);
			}
		}

		// Done part (ii), except calculating Yij;
		// But we can do this inline when we calculate S in part (iii).

		// Part (iii): Construct the the big block-matrix S which will be inverted.
		cv::Mat_<double> mS = cv::Mat_<double>::zeros(mnCamsToUpdate * 6, mnCamsToUpdate * 6);
		cv::Mat_<double> vE = cv::Mat_<double>::zeros(mnCamsToUpdate * 6, 1);

		cv::Matx<double, 6, 6> m6(6, 6); // Temp working space
		cv::Vec<double, 6> v6; // Temp working space

		// Calculate on-diagonal blocks of S (i.e. only one camera at a time:)
		for (unsigned int j = 0; j < mvCameras.size(); j++)
		{
			Camera &cam_j = mvCameras[j];
			if (cam_j.bFixed) continue;
			int nCamJStartRow = cam_j.nStartRow;

			// First, do the diagonal elements.
			//m6= cam_j.m6U;     // can't do this anymore because cam_j.m6U is LL!!
			for (int r = 0; r < 6; r++)
			{
				for (int c = 0; c < r; c++)
					m6(r, c) = m6(c, r) = cam_j.m6U(r, c);
				m6(r, r) = cam_j.m6U(r, r);
			}

			//for (int nn = 0; nn < 6; nn++)
			//	m6(nn, nn) *= (1.0 + mdLambda);

			//loop unrolling

			m6(0, 0) *= (1.0 + mdLambda);
			m6(1, 1) *= (1.0 + mdLambda);
			m6(2, 2) *= (1.0 + mdLambda);
			m6(3, 3) *= (1.0 + mdLambda);
			m6(4, 4) *= (1.0 + mdLambda);
			m6(5, 5) *= (1.0 + mdLambda);

			v6 = cam_j.v6EpsilonA;

			std::vector<Meas*> &vMeasLUTj = mvMeasLUTs[j];
			// Sum over measurements (points):
			for (unsigned int i = 0; i < mvPoints.size(); i++)
			{
				Meas* pMeas = vMeasLUTj[i];
				if (pMeas == NULL || pMeas->bBad)
					continue;
				m6 -= pMeas->m63W * mvPoints[i].m3VStarInv * pMeas->m63W.t();  // SLOW SLOW should by 6x6sy
				v6 -= pMeas->m63W * (mvPoints[i].m3VStarInv * mvPoints[i].v3EpsilonB);
			}

			// doing a PSD 6x6 block assignment (and respective error vector)
			int r_, c_;
			for (r_ = 0; r_ < 6; r_++) {

				vE(nCamJStartRow + r_, 0) = v6[r_];                         // the error vector
				mS(nCamJStartRow + r_, nCamJStartRow + r_) = m6(r_, r_);    // diagonal of mS

				for (c_ = 0; c_ < r_; c_++)
					mS(nCamJStartRow + r_, nCamJStartRow + c_) = mS(nCamJStartRow + c_, nCamJStartRow + r_) = m6(r_, c_);
			}
		}

		// Now find off-diag elements of S. These are camera-point-camera combinations, of which there are lots.
		// New code which doesn't waste as much time finding i-jk pairs; these are pre-stored in a per-i list.
		for (unsigned int i = 0; i < mvPoints.size(); i++)
		{
			BAPoint &p = mvPoints[i];
			int nCurrentJ = -1;
			int nJRow = -1;
			Meas* pMeas_ij;
			Meas* pMeas_ik;

			cv::Matx<double, 6, 3> m63_MIJW_times_m3VStarInv;

			for (std::vector<OffDiagScriptEntry>::iterator it = p.vOffDiagonalScript.begin();
				it != p.vOffDiagonalScript.end();
				it++)
			{
				OffDiagScriptEntry &e = *it;
				pMeas_ik = mvMeasLUTs[e.k][i];
				if (pMeas_ik == NULL || pMeas_ik->bBad)
					continue;

				if (e.j != nCurrentJ)
				{
					pMeas_ij = mvMeasLUTs[e.j][i];
					if (pMeas_ij == NULL || pMeas_ij->bBad)
						continue;
					nCurrentJ = e.j;
					nJRow = mvCameras[e.j].nStartRow;
					m63_MIJW_times_m3VStarInv = pMeas_ij->m63W * p.m3VStarInv;
				}

				int nKRow = mvCameras[pMeas_ik->c].nStartRow;

				int _r, _c;
				for (_r = 0; _r < 6; _r++)
					for (_c = 0; _c < 6; _c++)
						mS(nJRow + _r, nKRow + _c) -= m63_MIJW_x_m3VStarInv(_r, 0) * pMeasurement_Cam2->m63W(_c, 0) +
						m63_MIJW_x_m3VStarInv(_r, 1) * pMeasurement_Cam2->m63W(_c, 1) +
						m63_MIJW_x_m3VStarInv(_r, 2) * pMeasurement_Cam2->m63W(_c, 2);

				assert(nKRow < nJRow);
			}
		}

		// Did this purely LL triangle - now update the TR bit as well!
		// (This is actually unneccessary since the lapack cholesky solver
		// uses only one triangle, but I'm leaving it in here anyway.)
		for (int i = 0; i < mS.rows; i++)
			for (int j = 0; j < i; j++)
				mS(j, i) = mS(i, j);

		// Got fat matrix S and vector E from part(iii). Now Cholesky-decompose
		// the matrix, and find the camera update vector.
		cv::Mat_<double> vCamerasUpdate(mS.rows, 1);
		cv::solve(mS, vE, vCamerasUpdate, cv::DECOMP_CHOLESKY);

		// Part (iv): Compute the map updates
		cv::Mat_<double> vMapUpdates(mvPoints.size() * 3, 1);
		for (unsigned int i = 0; i < mvPoints.size(); i++)
		{
			cv::Vec3d v3Sum(0, 0, 0);
			for (unsigned int j = 0; j < mvCameras.size(); j++)
			{
				Camera &cam = mvCameras[j];
				if (cam.bFixed)
					continue;
				Meas *pMeas = mvMeasLUTs[j][i];
				if (pMeas == NULL || pMeas->bBad)
					continue;

				for (_c = 0; _c < 3; _c++)
					v3Sum[_c] += pMea->m63W(0, _c) * vCamerasUpdate(cam.nStartRow + 0, 0) +
					pMeas->m63W(1, _c) * mvCamerasUpdate(cam.nStartRow + 1, 0) +
					pMeas->m63W(2, _c) * mvCamerasUpdate(cam.nStartRow + 2, 0) +
					pMeas->m63W(3, _c) * mvCamerasUpdate(cam.nStartRow + 3, 0) +
					pMeas->m63W(4, _c) * mvCamerasUpdate(cam.nStartRow + 4, 0) +
					pMeas->m63W(5, _c) * mvCamerasUpdate(cam.nStartRow + 5, 0);
			}

			cv::Vec3d v3 = mvPoints[i].v3EpsilonB - v3Sum;

			vMapUpdates(i * 3 + 0, 0) = mvPoints[i].m3VStarInv(0, 0) * v3[0] + mvPoints[i].m3VStarInv(0, 1) * v3[1]
				+ mvPoints[i].m3VStarInv(0, 2) * v3[2];
			vMapUpdates(i * 3 + 1, 0) = mvPoints[i].m3VStarInv(1, 0) * v3[0] + mvPoints[i].m3VStarInv(1, 1) * v3[1]
				+ mvPoints[i].m3VStarInv(1, 2) * v3[2];
			vMapUpdates(i * 3 + 2, 0) = mvPoints[i].m3VStarInv(2, 0) * v3[0] + mvPoints[i].m3VStarInv(2, 1) * v3[1]
				+ mvPoints[i].m3VStarInv(2, 2) * v3[2];

			if (isnan(vMapUpdates(i * 3 + 0, 0) * vMapUpdates(i * 3 + 0, 0) +
				vMapUpdates(i * 3 + 1, 0) * vMapUpdates(i * 3 + 1, 0) +
				vMapUpdates(i * 3 + 2, 0) * vMapUpdates(i * 3 + 2, 0)))
			{
				std::cerr << "NANNERY! " << std::endl;
				std::cerr << mvPoints[i].m3VStarInv << std::endl;
			}
		}

		// OK, got the two update vectors.
		// First check for convergence..
		// (this is a very poor convergence test)
		double dSumSquaredUpdate = cv::norm(vCamerasUpdate, cv::NORM_L2SQR) + cv::norm(vMapUpdates, cv::NORM_L2SQR);
		if (dSumSquaredUpdate < *mgvdUpdateConvergenceLimit)
			mbConverged = true;

		// Now re-project everything and measure the error;
		// NB we don't keep these projections, SLOW, bit of a waste.

		// Temp versions of updated pose and pos:
		for (unsigned int j = 0; j < mvCameras.size(); j++)
		{
			if (mvCameras[j].bFixed)
				mvCameras[j].se3CfWNew = mvCameras[j].se3CfW;
			else
				mvCameras[j].se3CfWNew = RigidTransforms::SE3<>::exp(cv::Vec<double, 6>(
					vCamerasUpdate(mvCameras[j].nStartRow + 0, 0),
					vCamerasUpdate(mvCameras[j].nStartRow + 1, 0),
					vCamerasUpdate(mvCameras[j].nStartRow + 2, 0),
					vCamerasUpdate(mvCameras[j].nStartRow + 3, 0),
					vCamerasUpdate(mvCameras[j].nStartRow + 4, 0),
					vCamerasUpdate(mvCameras[j].nStartRow + 5, 0)))
				* mvCameras[j].se3CfW;
		}

		for (unsigned int i = 0; i < mvPoints.size(); i++)
			mvPoints[i].v3PosNew = cv::Vec3d(mvPoints[i].v3Pos[0] + vMapUpdates(i * 3 + 0, 0),
				mvPoints[i].v3Pos[1] + vMapUpdates(i * 3 + 1, 0),
				mvPoints[i].v3Pos[2] + vMapUpdates(i * 3 + 2, 0));

		// Calculate new error by re-projecting, doing tukey, etc etc:
		dNewError = FindNewError<MEstimator>();

		cout << std::setprecision(1) << "L" << mdLambda << std::setprecision(3) << "\tOld " << dCurrentError << "  New " << dNewError << "  Diff " << dCurrentError - dNewError << "\t";

		// Was the step good? If not, modify lambda and try again!!
		// (if it was good, will break from this loop.)
		if (dNewError > dCurrentError)
		{
			cout << " TRY AGAIN " << std::endl;
			ModifyLambda_BadStep();
		}

		mnCounter++;
		if (mnCounter >= *mgvnMaxIterations)
			mbHitMaxIterations = true;
	}   // End of while error too big loop

	if (dNewError < dCurrentError) // Was the last step a good one?
	{
		cout << " WINNER            ------------ " << std::endl;
		// Woo! got somewhere. Update lambda and make changes permanent.
		ModifyLambda_GoodStep();
		for (unsigned int j = 0; j < mvCameras.size(); j++)
			mvCameras[j].se3CfW = mvCameras[j].se3CfWNew;
		for (unsigned int i = 0; i < mvPoints.size(); i++)
			mvPoints[i].v3Pos = mvPoints[i].v3PosNew;
		mnAccepted++;
	}

	// Finally, ditch all the outliers.
	std::vector<std::list<Meas>::iterator> vit;
	for (std::list<Meas>::iterator itr = mMeasList.begin(); itr != mMeasList.end(); itr++)
		if (itr->bBad)
		{
			vit.push_back(itr);
			mvOutlierMeasurementIdx.push_back(std::make_pair(itr->p, itr->c));
			mvPoints[itr->p].nOutliers++;
			mvMeasLUTs[itr->c][itr->p] = NULL;
		}

	for (unsigned int i = 0; i < vit.size(); i++)
		mMeasList.erase(vit[i]);

	cout << "Nuked " << vit.size() << " measurements." << std::endl;
	return true;
}

// Find the new total error if cameras and points used their 
// new coordinates
template<class MEstimator>
double Bundle::FindNewError()
{
	std::ofstream ofs;
	double dNewError = 0;
	std::vector<double> vdErrorSquared;
	for (std::list<Meas>::iterator itr = mMeasList.begin(); itr != mMeasList.end(); itr++)
	{
		Meas &meas = *itr;
		// Project the point.
		cv::Vec3d v3Cam = mvCameras[meas.c].se3CfWNew * mvPoints[meas.p].v3PosNew;

		if (v3Cam[2] <= 0)
		{
			dNewError += 1.0;
			cout << ".";
			continue;
		}

		cv::Vec2d v2ImPlane = CvUtils::pproject(v3Cam);
		cv::Vec2d v2Image = mCamera.Project(v2ImPlane);
		cv::Vec2d v2Error = meas.dSqrtInvNoise * (meas.v2Found - v2Image);
		double dErrorSquared = v2Error[0] * v2Error[0] + v2Error[1] * v2Error[1];
		dNewError += MEstimator::ObjectiveScore(dErrorSquared, mdSigmaSquared);
	}

	return dNewError;
}

// Optimisation: make a bunch of tables, one per camera
// which point to measurements (if any) for each point
// This is faster than a std::map lookup.
void Bundle::GenerateMeasLUTs()
{
  mvMeasLUTs.clear();
  for(unsigned int nCam = 0; nCam < mvCameras.size(); nCam++)
    {
      mvMeasLUTs.push_back(vector<Meas*>());
      mvMeasLUTs.back().resize(mvPoints.size(), NULL);
    };
  for(list<Meas>::iterator it = mMeasList.begin(); it!=mMeasList.end(); it++)
    mvMeasLUTs[it->c][it->p] =  &(*it);
}

// Optimisation: make a per-point list of all
// observation camera-camera pairs; this is then
// scanned to make the off-diagonal elements of matrix S
void Bundle::GenerateOffDiagScripts()
{
  for(unsigned int i=0; i<mvPoints.size(); i++)
    {
      BAPoint &p = mvPoints[i];
      p.vOffDiagonalScript.clear();
      for(set<int>::iterator it_j = p.sCameras.begin(); it_j!=p.sCameras.end(); it_j++)
	{
	  int j = *it_j;
	  if(mvCameras[j].bFixed)
	    continue;
	  Meas *pMeas_j = mvMeasLUTs[j][i];
	  assert(pMeas_j != NULL);
	  
	  for(set<int>::iterator it_k = p.sCameras.begin(); it_k!=it_j; it_k++)
	    {
	      int k = *it_k;
	      if(mvCameras[k].bFixed)
		continue;
	      
	      Meas *pMeas_k = mvMeasLUTs[k][i];
	      assert(pMeas_k != NULL);
	      
	      OffDiagScriptEntry e;
	      e.j = j;
	      e.k = k;
	      p.vOffDiagonalScript.push_back(e);
	    }
	}
    }
}

void Bundle::ModifyLambda_GoodStep()
{
  mdLambdaFactor = 2.0;
  mdLambda *= 0.3;
};

void Bundle::ModifyLambda_BadStep()
{
  mdLambda = mdLambda * mdLambdaFactor;
  mdLambdaFactor = mdLambdaFactor * 2;
};


Vector<3> Bundle::GetPoint(int n)
{
  return mvPoints.at(n).v3Pos;
}

SE3<> Bundle::GetCamera(int n)
{
  return mvCameras.at(n).se3CfW;
}

set<int> Bundle::GetOutliers()
{
  set<int> sOutliers;
  set<int>::iterator hint = sOutliers.begin();
  for(unsigned int i=0; i<mvPoints.size(); i++)
    {
      Point &p = mvPoints[i];
      if(p.nMeasurements > 0 && p.nMeasurements == p.nOutliers)
	hint = sOutliers.insert(hint, i);
    }
  return sOutliers;
};


vector<pair<int, int> > Bundle::GetOutlierMeasurements()
{
  return mvOutlierMeasurementIdx;
}
