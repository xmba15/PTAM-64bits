#define WIN32_LEAN_AND_MEAN
#include "VideoSourceOpenCV.h"
#include <Windows.h>


using namespace CVD;
using namespace std;

#define CAPTURE_SIZE_X	640
#define CAPTURE_SIZE_Y	480
#define FPS				30

VideoSource::VideoSource()
{
	//EWC_Open(MEDIASUBTYPE_RGB24, CAPTURE_SIZE_X, CAPTURE_SIZE_Y, FPS);
 //   m_buffer = new unsigned char[EWC_GetBufferSize(0)];

	//mirSize.x = CAPTURE_SIZE_X;
	//mirSize.y = CAPTURE_SIZE_Y;
	std::cout << "Opening Video Source" << std::endl;
	mptr = new cv::VideoCapture(0);
	//cv::VideoCapture cap(0);
	cv::VideoCapture *cap = (cv::VideoCapture*) mptr;
	if (!cap->isOpened()) {
		std::cerr << "Unable to get the camera!" << std::endl;
		exit(-1);
	}
	VideoSource::imagesize = cv::Size(CAPTURE_SIZE_X, CAPTURE_SIZE_Y);
};


//void VideoSource::GetAndFillFrameBWandRGB(Image<CVD::byte> &imBW, Image<CVD::Rgb<CVD::byte> > &imRGB)
//{
//	EWC_GetImage(0, m_buffer);
//
//	unsigned char* pImage = m_buffer;
//
//	BasicImage<CVD::byte> imCaptured(pImage, mirSize);
//	imRGB.resize(mirSize);
//	imBW.resize(mirSize);
//
//	for (int y=0; y<mirSize.y; y++) {
//		for (int x=0; x<mirSize.x; x++) {
//			imRGB[y][x].blue = *pImage;
//			pImage++;
//
//			imRGB[y][x].green = *pImage;
//			imBW[y][x]        = *pImage;
//			pImage++;
//
//			imRGB[y][x].red = *pImage;
//			pImage++;
//		}
//	}
//
//}

void VideoSource::GetAndFillFrameBWandRGB(cv::Mat &imBW, cv::Mat &imRGB) {
	cv::Mat frame;
	cv::VideoCapture *cap = (cv::VideoCapture*) mptr;
	*cap >> frame;
	cv::resize(frame, imRGB, imagesize);
	cv::cvtColor(imRGB, imBW, CV_BGR2GRAY);
}

//ImageRef VideoSource::Size()
//{
//	return mirSize;
//}

cv::Size VideoSource::imageSize() {
	return VideoSource::imagesize;
}