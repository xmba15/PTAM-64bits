#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include "additionalUtility.h"

using namespace additionalUtility;

struct VideoSourceData;

class VideoSource
{
public:
	VideoSource();
	//~VideoSource();
	void GetAndFillFrameBWandRGB(cv::Mat &imBW, cv::Mat &imRGB);
	cv::Size imgSize();

private:
	unsigned char *m_buffer;
	void *mptr;
	cv::Size size;
};
