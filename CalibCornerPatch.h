// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

#ifndef __CALIB_CORNER_PATCH_H
#define __CALIB_CORNER_PATCH_H

#include <vector>

#include "additionalUtility.h"

using namespace additionalUtility;

class CalibCornerPatch
{
public:
  struct Params
  {
    Params();
    //TooN::Matrix<2> m2Warp();
	cv::Matx<double, 2, 2> m2Warp();
    cv::Vec2d v2Pos;
    cv::Vec2d v2Angles;
    double dMean;
    double dGain;
  };

  CalibCornerPatch(int nSideSize = 8);
  bool IterateOnImage(Params &params, cv::Mat_<uchar> &im);
  bool IterateOnImageWithDrawing(Params &params, cv::Mat_<uchar> &im);

 protected:

  void MakeTemplateWithCurrentParams();
  void FillTemplate(cv::Mat_<double> &im, Params params);
  double Iterate(cv::Mat_<uchar> &im);

  Params mParams;
  cv::Mat_<double> mimTemplate;
  cv::Mat_<cv::Vec2d> mimGradients;
  cv::Mat_<cv::Vec2d> mimAngleJacs;

  void MakeSharedTemplate();
  static cv::Mat_<double> mimSharedSourceTemplate;
  double mdLastError;
};
#endif

