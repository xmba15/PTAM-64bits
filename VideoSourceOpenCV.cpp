#define WIN32_LEAN_AND_MEAN
#include "VideoSourceOpenCV.h"

#ifdef _WIN32
#include <windows.h>
#endif

using namespace std;

#define CAPTURE_SIZE_X	640
#define CAPTURE_SIZE_Y	480
#define FPS				30

int tmpCount = 0;

VideoSource::VideoSource()
{
	std::cout << "Opening Video Source" << std::endl;
	mptr = new cv::VideoCapture(0);
	//cv::VideoCapture cap(0);
	cv::VideoCapture *cap = (cv::VideoCapture*) mptr;
	if (!cap->isOpened()) {
		std::cerr << "Unable to get the camera!" << std::endl;
		exit(-1);
	}
	VideoSource::size = cv::Size(CAPTURE_SIZE_X, CAPTURE_SIZE_Y);
};


void VideoSource::GetAndFillFrameBWandRGB(cv::Mat &imBW, cv::Mat &imRGB) 
{
	std::stringstream ss;
	ss << tmpCount;
	std::cout << "frame" + ss.str() << std::endl;
	tmpCount++;

	cv::Mat frame;
	cv::VideoCapture *cap = (cv::VideoCapture*) mptr;
	*cap >> frame;
	cv::resize(frame, imRGB, VideoSource::size);
	cv::cvtColor(imRGB, imBW, CV_BGR2GRAY);
}

cv::Size VideoSource::imgSize() {
	return VideoSource::size;
}
