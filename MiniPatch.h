// Declares MiniPatch class
// 
// This is a simple pixel-patch class, used for tracking small patches
// it's used by the tracker for building the initial map

#pragma once

#include "GCVD/Addedutils.h"
#include <vector>
#include "additionalUtility.h"

struct MiniPatch
{
  void SampleFromImage(cv::Point irPos, cv::Mat_<uchar> &im);  // Copy pixels out of source image
  bool FindPatch(cv::Point &irPos,           // Find patch in a new image
		 cv::Mat_<uchar> &im, 
		 int nRange,
		 std::vector<cv::Point> &vCorners,
		 std::vector<int> *pvRowLUT = NULL);
  
  inline int SSDAtPoint(cv::Mat_<uchar> &im, const cv::Point &ir); // Score function
  static int mnHalfPatchSize;     // How big is the patch?
  static int mnRange;             // How far to search? 
  static int mnMaxSSD;            // Max SSD for matches?
  cv::Mat_<uchar> mimOrigPatch;  // Original pixels
};
