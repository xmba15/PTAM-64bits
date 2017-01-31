// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

#ifndef __CALIB_IMAGE_H
#define __CALIB_IMAGE_H
#include "ATANCamera.h"
#include "CalibCornerPatch.h"
#include <vector>
#include <TooN/se3.h>

const int N_NOT_TRIED=-1;
const int N_FAILED=-2;

struct CalibGridCorner
{
  struct NeighborState
  {
    NeighborState() {val = N_NOT_TRIED;}
    int val;
  };
  
  CalibCornerPatch::Params Params;
#if _WIN64
  cv::Point irGridPos;
#else
  CVD::ImageRef irGridPos;
#endif

  NeighborState aNeighborStates[4];
  
  TooN::Matrix<2> GetSteps(std::vector<CalibGridCorner> &vgc); 
  TooN::Matrix<2> mInheritedSteps;
  
  void Draw();
  
  double ExpansionPotential();
};

class CalibImage
{
public:

#if _WIN64
  bool MakeFromImage(cv::Mat &im);
#else
  bool MakeFromImage(CVD::Image<CVD::byte> &im);
#endif
  TooN::SE3<> mse3CamFromWorld;
  void DrawImageGrid();
  void Draw3DGrid(ATANCamera &Camera, bool bDrawErrors);
  void GuessInitialPose(ATANCamera &Camera);

  struct ErrorAndJacobians
  {
    TooN::Vector<2> v2Error;
    TooN::Matrix<2,6> m26PoseJac;
	TooN::Matrix<2, NUMTRACKERCAMPARAMETERS> m2NCameraJac;
  };

  std::vector<ErrorAndJacobians> Project(ATANCamera &Camera);

#if _WIN64
  cv::Mat mim;
#else
  CVD::Image<CVD::byte> mim;
#endif

protected:
#if _WIN64
	std::vector<cv::Point> mvCorners;
#else
  std::vector<CVD::ImageRef> mvCorners;
#endif
  std::vector<CalibGridCorner> mvGridCorners;
  
  bool ExpandByAngle(int nSrc, int nDirn);
  int NextToExpand();
  void ExpandByStep(int n);

#if _WIN64
  cv::Point IR_from_dirn(int nDirn);
#else
  CVD::ImageRef IR_from_dirn(int nDirn);
#endif

};


#endif

