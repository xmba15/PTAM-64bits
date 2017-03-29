// Copyright 2008 Isis Innovation Limited
#include "System.h"
#include "OpenGL.h"
#include <stdlib.h>
#include "ATANCamera.h"
#include "MapMaker.h"
#include "Tracker.h"
#include "ARDriver.h"
#include "MapViewer.h"

#include "Persistence/instances.h"


System::System()
	: mGLWindow(mVideoSource.imgSize(), "PTAM")
{
	Persistence::GUI.RegisterCommand("exit", GUICommandCallBack, this);
	Persistence::GUI.RegisterCommand("quit", GUICommandCallBack, this);

	// First, check if the camera is calibrated.
	// If not, we need to run the calibration widget.
	cv::Vec<double, NUMTRACKERCAMPARAMETERS> vTest;

	vTest = Persistence::PV3::get<cv::Vec<double, NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, Persistence::HIDDEN);
	mpCamera = new ATANCamera("Camera");
	cv::Vec2d v2;
	if (v2 == v2);
	if (vTest == ATANCamera::mvDefaultParams)
	{
		std::cout << std::endl;
		std::cout << "! Camera.Parameters is not set, need to run the CameraCalibrator tool" << std::endl;
		std::cout << "  and/or put the Camera.Parameters= line into the appropriate .cfg file." << std::endl;
		exit(1);
	}

	mpMap = new Map;
	mpMapMaker = new MapMaker(*mpMap, *mpCamera);
	mpTracker = new Tracker(mVideoSource.imgSize(), *mpCamera, *mpMap, *mpMapMaker);
	mpARDriver = new ARDriver(*mpCamera, mVideoSource.imgSize(), mGLWindow);
	mpMapViewer = new MapViewer(*mpMap, mGLWindow);

	Persistence::GUI.ParseLine("GLWindow.AddMenu Menu Menu");
	Persistence::GUI.ParseLine("Menu.ShowMenu Root");
	Persistence::GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
	Persistence::GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
	Persistence::GUI.ParseLine("DrawAR=0");
	Persistence::GUI.ParseLine("DrawMap=0");
	Persistence::GUI.ParseLine("Menu.AddMenuToggle Root \"View Map\" DrawMap Root");
	Persistence::GUI.ParseLine("Menu.AddMenuToggle Root \"Draw AR\" DrawAR Root");

	mbDone = false;
}

void System::Run()
{
	while (!mbDone)
	{

		// We use two versions of each video frame:
		// One black and white (for processing by the tracker etc)
		// and one RGB, for drawing.

		// Grab new video frame...
		mVideoSource.GetAndFillFrameBWandRGB(mimFrameBW, mimFrameRGB);
		static bool bFirstFrame = true;
		if (bFirstFrame)
		{
			mpARDriver->Init();
			bFirstFrame = false;
		}

		mGLWindow.SetupViewport();
		mGLWindow.SetupVideoOrtho();
		mGLWindow.SetupVideoRasterPosAndZoom();

		if (!mpMap->IsGood())
			mpARDriver->Reset();

		static Persistence::pvar3<int> gvnDrawMap("DrawMap", 0, Persistence::HIDDEN | Persistence::SILENT);
		static Persistence::pvar3<int> gvnDrawAR("DrawAR", 0, Persistence::HIDDEN | Persistence::SILENT);

		bool bDrawMap = mpMap->IsGood() && *gvnDrawMap;
		bool bDrawAR = mpMap->IsGood() && *gvnDrawAR;

		mpTracker->TrackFrame(mimFrameBW, !bDrawAR && !bDrawMap, mimFrameRGB);

		if (bDrawMap)
			mpMapViewer->DrawMap(mpTracker->GetCurrentPose());
		else if (bDrawAR)
			mpARDriver->Render(mimFrameRGB, mpTracker->GetCurrentPose());

		//      mGLWindow.GetMousePoseUpdate();
		string sCaption;
		if (bDrawMap)
			sCaption = mpMapViewer->GetMessageForUser();
		else
			sCaption = mpTracker->GetMessageForUser();
		mGLWindow.DrawCaption(sCaption);
		mGLWindow.DrawMenus();
		mGLWindow.swap_buffers();
		mGLWindow.HandlePendingEvents();
	}
}

void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
	if (sCommand == "quit" || sCommand == "exit")
		static_cast<System*>(ptr)->mbDone = true;
}








