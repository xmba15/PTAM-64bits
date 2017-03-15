// SmallBlurryImage - A small and blurry representation of an image.
// used by the relocaliser.

#pragma once

#include "GCVD/SE2.h"
#include "GCVD/SE3.h"
#include "KeyFrame.h"
#include "ATANCamera.h"
#include "additionalUtility.h"

class SmallBlurryImage
{
public:
	SmallBlurryImage();
	SmallBlurryImage(KeyFrame &kf, double dBlur = 2.5);
	void MakeFromKF(KeyFrame &kf, double dBlur = 2.5);
	void MakeJacs();
	double ZMSSD(SmallBlurryImage &other);
	std::pair<RigidTransforms::SE2<>, double> IteratePosRelToTarget(SmallBlurryImage &other, int nIterations = 10);
	static RigidTransforms::SE3<> SE3fromSE2(RigidTransforms::SE2<> se2, ATANCamera camera);

protected:
	cv::Mat_<uchar> mimSmall;
	cv::Mat_<double> mimTemplate;
	cv::Mat_<cv::Vec2d > mimImageJacs;
	bool mbMadeJacs;
	static cv::Size mirSize;
};
