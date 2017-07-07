// Defines the System class
//
// This stores the main functional classes of the system, like the
// mapmaker, map, tracker etc, and spawns the working threads.
//

#pragma once
#include "additionalUtility.h"
using namespace additionalUtility;

#include "VideoSource.h"
#include "GLWindow2.h"

class ATANCamera;
class Map;
class MapMaker;
class Tracker;
class ARDriver;
class MapViewer;

class System
{
public:
	System();
	void Run();

private:
	VideoSource mVideoSource;
	GLWindow2 mGLWindow;
	cv::Mat mimFrameRGB;
	cv::Mat_<uchar> mimFrameBW;

	Map *mpMap;
	MapMaker *mpMapMaker;
	Tracker *mpTracker;
	ATANCamera *mpCamera;
	ARDriver *mpARDriver;
	MapViewer *mpMapViewer;

	bool mbDone;

	static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
};
