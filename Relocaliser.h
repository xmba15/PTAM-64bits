// SmallBlurryImage-based relocaliser
// 
// Each KF stores a small, blurred version of itself;
// Just compare a small, blurred version of the input frame to all the KFs,
// choose the closest match, and then estimate a camera rotation by direct image
// minimisation.

#pragma once
#include "additionalUtility.h"
using namespace additionalUtility;

#include "GCVD/SE2.h"
#include "GCVD/SE3.h"
#include "ATANCamera.h"
#include "SmallBlurryImage.h"

#include "Map.h"

class Relocaliser
{
public:
	Relocaliser(Map &map, ATANCamera &camera);
	bool AttemptRecovery(KeyFrame::Ptr pKF);
	SE3<> BestPose();

protected:
	void ScoreKFs(KeyFrame::Ptr pKFCurrentF);
	Map &mMap;
	ATANCamera mCamera;
	int mnBest;
	double mdBestScore;
	SE2<> mse2;
	SE3<> mse3Best;

};