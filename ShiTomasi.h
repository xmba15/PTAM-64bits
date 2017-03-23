#pragma once

#include "additionalUtility.h"

double FindShiTomasiScoreAtPoint(cv::Mat_<uchar> &image,
	int nHalfBoxSize,
	cv::Point irCenter);
