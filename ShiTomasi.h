#pragma once

#include "additionalUtility.h"
using namespace additionalUtility;

double FindShiTomasiScoreAtPoint(cv::Mat_<uchar> &image,
	int nHalfBoxSize,
	cv::Point2i irCenter);