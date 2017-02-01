#pragma once

#define NOMINMAX

#include <TooN/TooN.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "OpenGL.h"
#include "glwindow.h"
#include "gl_helpers.h"

inline TooN::Vector<2> size2Vec(const cv::Size &imagesize);
inline TooN::Vector<2> size2Vec(const cv::Point &pt);
inline void cv_sample(const cv::Mat &im, double x, double y, float& result);
int cv_transform(cv::Mat& in, cv::Mat& out, const TooN::Matrix<2>& M, const TooN::Vector<2>& inOrig, const TooN::Vector<2>& outOrig, const float defaultValue = float());
double getSubpix(const cv::Mat &img, cv::Point2d pt);
double getSubpix(const cv::Mat &img, TooN::Vector<2> vec);
inline unsigned int mag_squared(const cv::Point &pt);
