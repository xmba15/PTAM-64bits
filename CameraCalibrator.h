// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

#ifndef __CAMERACALIBRATOR_H
#define __CAMERACALIBRATOR_H
#include "CalibImage.h"
#include "VideoSourceOpenCV.h"
#include <gvars3/gvars3.h>
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
#if _WIN64
  void HandleFrame(cv::Mat imFrame);
#else
  void HandleFrame(CVD::Image<CVD::byte> imFrame);
#endif
  static void MainLoopCallback(void* pvUserData);
  void MainLoopStep();
  VideoSource mVideoSource;
  
  GLWindow2 mGLWindow;
  ATANCamera mCamera;
  bool mbDone;

  std::vector<CalibImage> mvCalibImgs;
  void OptimizeOneStep();
  
  bool mbGrabNextFrame;
  GVars3::gvar3<int> mgvnOptimizing;
  GVars3::gvar3<int> mgvnShowImage;
  GVars3::gvar3<int> mgvnDisableDistortion;
  double mdMeanPixelError;

  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
};

#endif
