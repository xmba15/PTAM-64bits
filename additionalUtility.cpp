#include "additionalUtility.h"

inline TooN::Vector<2> size2Vec(const cv::Size &imagesize)
{
	TooN::Vector<2> r;
	r[0] = imagesize.width;
	r[1] = imagesize.height;
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

int cv_transform(cv::Mat& in, cv::Mat& out, const TooN::Matrix<2>& M, const TooN::Vector<2>& inOrig, const TooN::Vector<2>& outOrig, const float defaultValue = float())
{
	const int w = out.size().width, h = out.size().height, iw = in.size().width, ih = in.size().height;
	const TooN::Vector<2> across = M.T()[0];
	const TooN::Vector<2> down = M.T()[1];

	const TooN::Vector<2> p0 = inOrig - M*outOrig;
	const TooN::Vector<2> p1 = p0 + w*across;
	const TooN::Vector<2> p2 = p0 + h*down;
	const TooN::Vector<2> p3 = p0 + w*across + h*down;

	// ul --> p0
	// ur --> w*across + p0
	// ll --> h*down + p0
	// lr --> w*across + h*down + p0
	double min_x = p0[0], min_y = p0[1];
	double max_x = min_x, max_y = min_y;

	// Minimal comparisons needed to determine bounds
	if (across[0] < 0)
		min_x += w*across[0];
	else
		max_x += w*across[0];
	if (down[0] < 0)
		min_x += h*down[0];
	else
		max_x += h*down[0];
	if (across[1] < 0)
		min_y += w*across[1];
	else
		max_y += w*across[1];
	if (down[1] < 0)
		min_y += h*down[1];
	else
		max_y += h*down[1];

	// This gets from the end of one row to the beginning of the next
	const TooN::Vector<2> carriage_return = down - w*across;

	//If the patch being extracted is completely in the image then no 
	//check is needed with each point.
	if (min_x >= 0 && min_y >= 0 && max_x < iw - 1 && max_y < ih - 1)
	{
		TooN::Vector<2> p = p0;
		for (int i = 0; i < h; ++i, p += carriage_return)
			for (int j = 0; j < w; ++j, p += across)
				//cv_sample(in, p[0], p[1], out[i][j]);
				cv_sample(in, p[0], p[1], out.ptr<float>(i)[j]);

		return 0;
	}
	else // Check each source location
	{
		// Store as doubles to avoid conversion cost for comparison
		const double x_bound = iw - 1;
		const double y_bound = ih - 1;
		int count = 0;
		TooN::Vector<2> p = p0;
		for (int i = 0; i<h; ++i, p += carriage_return) {
			for (int j = 0; j<w; ++j, p += across) {
				//Make sure that we are extracting pixels in the image
				if (0 <= p[0] && 0 <= p[1] && p[0] < x_bound && p[1] < y_bound)
					//sample(in, p[0], p[1], out[i][j]);
					cv_sample(in, p[0], p[1], out.ptr<float>(i)[j]);
				else {
					//out[i][j] = defaultValue;
					out.ptr<float>(i)[j] = defaultValue;
					++count;
				}
			}
		}
		return count;
	}
}

double getSubpix(const cv::Mat &img, cv::Point2d pt)
{
	cv::Mat patch;
	cv::getRectSubPix(img, cv::Size(1, 1), pt, patch);
	return (double)patch.ptr<float>(0)[0];
}