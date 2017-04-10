#pragma once

#include "GCVD/SE3.h"
#include "PatchFinder.h"
#include "ATANCamera.h"
#include "boost/shared_ptr.hpp"
#include "additionalUtility.h"

// This class contains all the intermediate results associated with
// a map-point that the tracker keeps up-to-date. TrackerData
// basically handles all the tracker's point-projection jobs,
// and also contains the PatchFinder which does the image search.
// It's very code-heavy for an h-file (it's a bunch of methods really)
// but it's only included from Tracker.cc!

struct TrackerData
{
	typedef boost::shared_ptr<TrackerData> Ptr;
	TrackerData(MapPoint::Ptr pMapPoint) : Point(pMapPoint) {};

	MapPoint::Ptr Point;
	PatchFinder Finder;

	// Projection itermediates:
	cv::Vec3d v3Cam;        // Coords in current cam frame
	cv::Vec2d v2ImPlane;    // Coords in current cam z=1 plane
	cv::Vec2d v2Image;      // Pixel coords in LEVEL0
	cv::Matx<double, 2, 2> m2CamDerivs;  // Camera projection derivs
	bool bInImage;
	bool bPotentiallyVisible;

	int nSearchLevel;
	bool bSearched;
	bool bFound;
	bool bDidSubPix;
	cv::Vec2d v2Found;      // Pixel coords of found patch (L0)
	double dSqrtInvNoise;   // Only depends on search level..


	// Stuff for pose update:
	cv::Vec2d v2Error_CovScaled;
	cv::Matx<double, 2, 6> m26Jacobian;   // Jacobian wrt camera position

	// Project point into image given certain pose and camera.
	// This can bail out at several stages if the point
	// will not be properly in the image.
	inline void Project(const RigidTransforms::SE3<> &se3CFromW, ATANCamera &Cam)
	{
		bInImage = bPotentiallyVisible = false;
		v3Cam = se3CFromW * Point->v3WorldPos;
		if (v3Cam[2] < 0.001)
			return;
		v2ImPlane = CvUtils::pproject(v3Cam);
		if (cv::norm(v2ImPlane) > Cam.LargestRadiusInImage())
			return;
		v2Image = Cam.Project(v2ImPlane);
		if (Cam.Invalid())
			return;

		if (v2Image[0] < 0 || v2Image[1] < 0 || v2Image[0] > irImageSize.width-1 || v2Image[1] > irImageSize.height-1)
			return;
		bInImage = true;
	}

	// Get the projection derivatives (depend only on the camera.)
	// This is called Unsafe because it depends on the camera caching 
	// results from the previous projection:
	// Only do this right after the same point has been projected!
	inline void GetDerivsUnsafe(ATANCamera &Cam)
	{
		m2CamDerivs = Cam.GetProjectionDerivs();
	}

	// Does projection and gets camera derivs all in one.
	inline void ProjectAndDerivs(RigidTransforms::SE3<> &se3, ATANCamera &Cam)
	{
		Project(se3, Cam);
		if (bFound)
			GetDerivsUnsafe(Cam);
	}

	// Jacobian of projection W.R.T. the camera position
	// I.e. if  p_cam = SE3Old * p_world, 
	//         SE3New = SE3Motion * SE3Old
	inline void CalcJacobian()
	{
		double dOneOverCameraZ = 1.0 / v3Cam[2];
		for (int m = 0; m < 6; m++)
		{
			const cv::Vec<double, 4> v4Motion = RigidTransforms::SE3<>::generator_field(m, CvUtils::backproject(v3Cam));
			cv::Vec2d v2CamFrameMotion((v4Motion[0] - v3Cam[0] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ,
				(v4Motion[1] - v3Cam[1] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ);
			m26Jacobian(0, m) = m2CamDerivs(0, 0) * v2CamFrameMotion[0] + m2CamDerivs(0, 1) * v2CamFrameMotion[1];
			m26Jacobian(1, m) = m2CamDerivs(1, 0) * v2CamFrameMotion[0] + m2CamDerivs(1, 1) * v2CamFrameMotion[1];
		};
	}

	// Sometimes in tracker instead of reprojecting, just update the error linearly!
	inline void LinearUpdate(const cv::Vec<double, 6> &v6)
	{
		v2Image += m26Jacobian * v6;
	}

	// This static member is filled in by the tracker and allows in-image checks in this class above.
	static cv::Size irImageSize;
};
