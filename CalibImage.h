// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

#ifndef __CALIB_IMAGE_H
#define __CALIB_IMAGE_H
#include "ATANCamera.h"
#include "CalibCornerPatch.h"
#include <vector>
#include "additionalUtility.h"

using namespace additionalUtility;

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
  cv::Point irGridPos;

  NeighborState aNeighborStates[4];
  
  cv::Matx<double, 2, 2> GetSteps(std::vector<CalibGridCorner> &vgc); 
  cv::Matx<double, 2, 2> mInheritedSteps;
  
  void Draw();
  
  double ExpansionPotential();
};

class CalibImage
{
public:
  bool MakeFromImage(cv::Mat &im);
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

  cv::Mat mim;

protected:

  std::vector<cv::Point> mvCorners;
  std::vector<CalibGridCorner> mvGridCorners;
  
  bool ExpandByAngle(int nSrc, int nDirn);
  int NextToExpand();
  void ExpandByStep(int n);

  cv::Point IR_from_dirn(int nDirn);
};

#endif