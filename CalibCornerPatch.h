// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

#ifndef __CALIB_CORNER_PATCH_H
#define __CALIB_CORNER_PATCH_H
#include <TooN/TooN.h>
using namespace TooN;
#include <cvd/image.h>
#include <cvd/byte.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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
  bool IterateOnImage(Params &params, CVD::Image<CVD::byte> &im);
  bool IterateOnImage(Params &params, cv::Mat &im);
  bool IterateOnImageWithDrawing(Params &params, CVD::Image<CVD::byte> &im);
  bool IterateOnIMageWithDrawing(Params &params, cv::Mat &im);
 protected:
  void MakeTemplateWithCurrentParams();
  //void FillTemplate(CVD::Image<float> &im, Params params);
  void FillTemplate(cv::Mat &im, Params params);
  double Iterate(CVD::Image<CVD::byte> &im);
  double Iterate(cv::Mat &im);
  Params mParams;
  CVD::Image<float> mimTemplate; //Same as mimSmall but with the mean image intensity subtracted, used in all image operations
  CVD::Image<Vector<2> > mimGradients;
  CVD::Image<Vector<2> > mimAngleJacs; 
  
  cv::Mat mimTemplate;
  cv::Mat mimGradients;
  cv::Mat mimAngleJacs;

  void MakeSharedTemplate();
  //static CVD::Image<float> mimSharedSourceTemplate; 
  static cv::Mat mimSharedSourceTemplate;
  double mdLastError;
};

#endif
