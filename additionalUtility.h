#pragma once

#define NOMINMAX

#include <TooN/TooN.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "OpenGL.h"
#include "glwindow.h"
#include "gl_helpers.h"

namespace additionalUtility
{
	inline TooN::Vector<2> size2Vec(const cv::Size &imagesize)
	{
		TooN::Vector<2> r;
		r[0] = imagesize.width;
		r[1] = imagesize.height;
		return r;
	}

	inline TooN::Vector<2> size2Vec(const cv::Point &pt)
	{
		TooN::Vector<2> r;
		r[0] = pt.x;
		r[1] = pt.y;
		return r;
	}

	inline void cv_sample(const cv::Mat &im, double x, double y, float& result)
	{
		int lx = (int)x;
		int ly = (int)y;
		int w = im.size().width;
		float a = im.ptr<float>(ly)[lx];
		float b = im.ptr<float>(ly)[lx + 1];
		float c = im.ptr<float>(ly + 1)[lx];
		float d = im.ptr<float>(ly + 1)[lx + 1];
		float e = a - b;
		x -= lx;
		y -= ly;
		result = (float)(x*(y*(e - c + d) - e) + y*(c - a) + a);
	}

	int cv_transform(cv::Mat& in, cv::Mat& out, const TooN::Matrix<2>& M, const TooN::Vector<2>& inOrig, const TooN::Vector<2>& outOrig, const float defaultValue = float());
	double getSubpix(const cv::Mat &img, cv::Point2d pt);
	double getSubpix(const cv::Mat &img, TooN::Vector<2> vec);
	inline unsigned int mag_squared(const cv::Point &pt)
	{
		return (pt.x * pt.x + pt.y * pt.y);
	}
};