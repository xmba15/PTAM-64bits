// Declares MiniPatch class
// 
// This is a simple pixel-patch class, used for tracking small patches
// it's used by the tracker for building the initial map

#pragma once
#include "additionalUtility.h"
using namespace additionalUtility;

#include "GCVD/Addedutils.h"


#include <vector>

struct MiniPatch
{
	// lift a ptach verbatim from source image "im"
	void SampleFromImage(cv::Point2i irPos, cv::Mat_<uchar> &im);

	// Finds this patch in a new image
	bool FindPatch(cv::Point2i &irPos,
		cv::Mat_<uchar> &im,
		int nRange,
		std::vector<cv::Point2i> &vCorners,
		std::vector<int> *pvRowLUT = NULL);

	inline int SSDAtPoint(cv::Mat_<uchar> &im, const cv::Point2i &ir); // Score function

	static int mnHalfPatchSize;     // How big is the patch?
	static int mnRange;             // How far to search? 
	static int mnMaxSSD;            // Max SSD for matches?

	cv::Mat_<uchar> mimOrigPatch;  // Original pixels

};