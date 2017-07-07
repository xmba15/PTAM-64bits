// Declares the EyeGame class
// EyeGame is a trivial AR app which draws some 3D graphics
// Draws a bunch of 3d eyeballs remniscient of the 
// AVL logo
//

#pragma once
#include "additionalUtility.h"
using namespace additionalUtility;

#include "GCVD/SE3.h"

using namespace RigidTransforms;

class EyeGame
{
public:
	EyeGame();
	void DrawStuff(cv::Vec3f v3CameraPos);
	void Reset();
	void Init();


protected:
	bool mbInitialised;
	void DrawEye();
	void LookAt(int nEye, cv::Vec3f v3, double dRotLimit);
	void MakeShadowTex();

	GLuint mnEyeDisplayList;
	GLuint mnShadowTex;
	double mdEyeRadius;
	double mdShadowHalfSize;
	SE3<> ase3WorldFromEye[4];
	int mnFrameCounter;

};
