// Declares the HomographyInit class and a few helper functions. 
//
// This class is used by MapMaker to bootstrap the map, and implements
// the homography decomposition of Faugeras and Lustman's 1988 tech
// report.
//
// Implementation according to Faugeras and Lustman

#pragma once

#include "GCVD/SE3.h"
#include <vector>
#include "additionalUtility.h"

// Homography matches are 2D-2D matches in a stereo pair, unprojected
// to the Z=1 plane.
struct HomographyMatch
{
  // To be filled in by MapMaker:
  cv::Vec2d v2CamPlaneFirst;
  cv::Vec2d v2CamPlaneSecond;
  cv::Matx<double, 2, 2> m2PixelProjectionJac;
};

// Storage for each homography decomposition
struct HomographyDecomposition
{
  cv::Vec3d v3Tp;
  cv::Matx<double, 3, 3> m3Rp;
  double d;
  cv::Vec3d v3n;
  
  // The resolved composition..
  RigidTransforms::SE3<> se3SecondFromFirst;
  int nScore;
};

class HomographyInit
{
public:
  bool Compute(std::vector<HomographyMatch> vMatches, double dMaxPixelError, RigidTransforms::SE3<> &se3SecondCameraPose);
protected:
  cv::Matx<double, 3, 3> HomographyFromMatches(std::vector<HomographyMatch> vMatches);
  void BestHomographyFromMatches_MLESAC();
  void DecomposeHomography();
  void ChooseBestDecomposition();
  void RefineHomographyWithInliers();
  
  bool IsHomographyInlier(cv::Matx<double, 3, 3> m3Homography, HomographyMatch match);
  double MLESACScore(cv::Matx<double, 3, 3> m3Homography, HomographyMatch match);
  
  double mdMaxPixelErrorSquared;
  cv::Matx<double, 3, 3> mm3BestHomography;
  std::vector<HomographyMatch> mvMatches;
  std::vector<HomographyMatch> mvHomographyInliers;
  std::vector<HomographyDecomposition> mvDecompositions;
};
