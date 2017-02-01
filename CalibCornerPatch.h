// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

#ifndef __CALIB_CORNER_PATCH_H
#define __CALIB_CORNER_PATCH_H
#include <TooN/TooN.h>
using namespace TooN;
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "additionalUtility.h"

using namespace additionalUtility;

class CalibCornerPatch
{
public:
  struct Params
  {
    Params();
    TooN::Matrix<2> m2Warp();
    TooN::Vector<2> v2Pos;
    TooN::Vector<2> v2Angles;
    double dMean;
    double dGain;
  };
  
  CalibCornerPatch(int nSideSize = 8);
  bool IterateOnImage(Params &params, cv::Mat &im);
  bool IterateOnImageWithDrawing(Params &params, cv::Mat &im);

 protected:

  void MakeTemplateWithCurrentParams();
  double Iterate(cv::Mat &im);

  Params mParams;
  cv::Mat mimTemplate;
  std::vector<cv::Mat> mimGradients;
  std::vector<cv::Mat> mimAngleJacs;

  void MakeSharedTemplate();
  static cv::Mat mimSharedSourceTemplate;
  double mdLastError;
};
#endif

