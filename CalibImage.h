#pragma once

#include "ATANCamera.h"
#include "CalibCornerPatch.h"
#include <vector>
#include "additionalUtility.h"
#include "GCVD/Addedutils.h"
#include "GCVD/SE3.h"

using namespace additionalUtility;
using namespace RigidTransforms;

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
  bool MakeFromImage(cv::Mat_<uchar> &im, cv::Mat &cim);
  RigidTransforms::SE3<> mse3CamFromWorld;
  void DrawImageGrid();
  void Draw3DGrid(ATANCamera &Camera, bool bDrawErrors);
  void GuessInitialPose(ATANCamera &Camera);

  struct ErrorAndJacobians
  {
    cv::Vec2d v2Error;
    cv::Matx<double, 2, 6> m26PoseJac;
	cv::Matx<double, 2, NUMTRACKERCAMPARAMETERS> m2NCameraJac;
	ErrorAndJacobians() : m26PoseJac(cv::Matx<double, 2, 6>()), m2NCameraJac(cv::Matx<double, 2, NUMTRACKERCAMPARAMETERS>()) {}
  };

  std::vector<ErrorAndJacobians> Project(ATANCamera &Camera);

  cv::Mat_<uchar> mim;
  cv::Mat rgbmim;
protected:

  std::vector<cv::Point> mvCorners;
  std::vector<CalibGridCorner> mvGridCorners;
  
  bool ExpandByAngle(int nSrc, int nDirn);
  int NextToExpand();
  void ExpandByStep(int n);

  cv::Point IR_from_dirn(int nDirn);
};
