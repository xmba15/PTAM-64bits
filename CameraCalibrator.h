// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

#pragma once

#define NOMINMAX

#include "CalibImage.h"
#include "VideoSourceOpenCV.h"
#include "Persistence/PVars.h"
#include <vector>
#include "GLWindow2.h"
#include "additionalUtility.h"

using namespace additionalUtility;

class CameraCalibrator
{
public:
  CameraCalibrator();
  void Run();
  
protected:
  void Reset();
  void HandleFrame(cv::Mat imFrame);
  static void MainLoopCallback(void* pvUserData);
  void MainLoopStep();
  VideoSource mVideoSource;
  
  GLWindow2 mGLWindow;
  ATANCamera mCamera;
  bool mbDone;

  std::vector<CalibImage> mvCalibImgs;
  void OptimizeOneStep();
  
  bool mbGrabNextFrame;
  Persistence::pvar3<int> mgvnOptimizing;
  Persistence::pvar3<int> mgvnShowImage;
  Persistence::pvar3<int> mgvnDisableDistortion;
  double mdMeanPixelError;

  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
};
