#pragma once

#include <stdlib.h>
#include "additionalUtility.h"

using namespace additionalUtility;
using namespace cv;

struct VideoSourceData;

class VideoSource
{
public:
	VideoSource();
	~VideoSource() {
		std::cout << "terminate video source" << std::endl;
	}

	void GetAndFillFrameBWandRGB(cv::Mat_<uchar> &imBW, cv::Mat &imRGB);
	cv::Size2i getSize();

private:
	cv::VideoCapture *pcap;
	cv::Size2i mirSize;
};
