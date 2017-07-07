// This file declares the ARDriver class
//
// ARDriver provides basic graphics services for drawing augmented
// graphics. It manages the OpenGL setup and the camera's radial
// distortion so that real and distorted virtual graphics can be
// properly blended.
//

#pragma once
#include "additionalUtility.h"
using namespace additionalUtility;
using namespace std;

#include "GCVD/SE3.h"
#include "ATANCamera.h"
#include "GLWindow2.h"
#include "EyeGame.h"

using namespace std;


class ARDriver
{
public:
	ARDriver(const ATANCamera &cam, GLWindow2 &glw);
	void Render(cv::Mat &imFrame, SE3<> se3CamFromWorld);
	void Reset();
	void Init();
protected:
	ATANCamera mCamera;
	GLWindow2 &mGLWindow;
	void DrawFadingGrid();
	void MakeFrameBuffer();
	void DrawFBBackGround();
	void DrawDistortedFB();
	void SetFrustum();

	// Texture stuff:
	GLuint mnFrameBuffer;
	GLuint mnFrameBufferTex;
	GLuint mnFrameTex;

	int mnCounter;
	cv::Size2i mirFBSize;
	cv::Size2i mirFrameSize;
	SE3<> mse3;
	bool mbInitialised;

	// Eyeballs:
	EyeGame mGame;
};