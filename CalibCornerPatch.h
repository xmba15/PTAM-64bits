// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

#ifndef __CALIB_CORNER_PATCH_H
#define __CALIB_CORNER_PATCH_H
#include <TooN/TooN.h>
using namespace TooN;
#include <vector>

#if _WIN64
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "additionalUtility.h"
#else
#include <cvd/image.h>
#include <cvd/byte.h>
#endif

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
#if _WIN64
  bool IterateOnImage(Params &params, cv::Mat &im);
  bool IterateOnImageWithDrawing(Params &params, cv::Mat &im);
#else
  bool IterateOnImage(Params &params, CVD::Image<CVD::byte> &im);
  bool IterateOnImageWithDrawing(Params &params, CVD::Image<CVD::byte> &im);
#endif
 protected:
  void MakeTemplateWithCurrentParams();
#if _WIN64
  //void FillTemplate(cv::Mat &im, Params params);
  double Iterate(cv::Mat &im);
#else
  double Iterate(CVD::Image<CVD::byte> &im);
  CVD::Image<float> mimTemplate; //Same as mimSmall but with the mean image intensity subtracted, used in all image operations
  CVD::Image<Vector<2> > mimGradients;
  CVD::Image<Vector<2> > mimAngleJacs; 
#endif

  Params mParams;
  cv::Mat mimTemplate;
  std::vector<cv::Mat> mimGradients;
  std::vector<cv::Mat> mimAngleJacs;

  void MakeSharedTemplate();
  //static CVD::Image<float> mimSharedSourceTemplate; 
  static cv::Mat mimSharedSourceTemplate;
  double mdLastError;
};

#endif
