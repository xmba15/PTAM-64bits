#pragma once

#define NOMINMAX

#ifdef _WIN32
#include <windows.h>
#endif

#include "OpenCV.h"
#include "OpenGL.h"
#include <algorithm>
#include <cassert>

namespace additionalUtility
{
	inline void cv_sample(const cv::Mat &im, double x, double y, double& result)
	{
		int lx = (int)x;
		int ly = (int)y;
		int w = im.size().width;
		double a = im.ptr<double>(ly)[lx];
		double b = im.ptr<double>(ly)[lx + 1];
		double c = im.ptr<double>(ly + 1)[lx];
		double d = im.ptr<double>(ly + 1)[lx + 1];
		double e = a - b;
		x -= lx;
		y -= ly;
		result = (float)(x*(y*(e - c + d) - e) + y*(c - a) + a);
	}

	inline cv::Matx<double, 2, 2> M2Inverse(const cv::Matx<double, 2, 2> &m)
	{
		cv::Matx<double, 2, 2> m2Res;
		double dDet = m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1);
		assert(dDet != 0.0);
		double dInverseDet = 1.0 / dDet;
		m2Res(0, 0) = m(1, 1) * dInverseDet;
		m2Res(1, 1) = m(0, 0) * dInverseDet;
		m2Res(1, 0) = -m(1, 0) * dInverseDet;
		m2Res(0, 1) = -m(0, 1) * dInverseDet;
		return m2Res;
	};

	int cv_transform(cv::Mat& in, cv::Mat& out, const cv::Matx<double, 2, 2>& M, const cv::Vec2d& inOrig, const cv::Vec2d& outOrig, const double defaultValue = double());
	int cv_transform(cv::Mat& in, cv::Mat& out, const cv::Matx<double, 2, 2>& M, const cv::Size& inOrig, const cv::Size& outOrig, const double defaultValue = double());
	inline unsigned int mag_squared(const cv::Point &pt)
	{
		return (pt.x * pt.x + pt.y * pt.y);
	}
};