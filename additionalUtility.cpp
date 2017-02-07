#include "additionalUtility.h"

int additionalUtility::cv_transform(cv::Mat &in, cv::Mat &out, const cv::Matx<double, 2, 2>& M, const cv::Vec2d& inOrig, const cv::Vec2d& outOrig, const double defaultValue)
{
	const int w = out.size().width, h = out.size().height, iw = in.size().width, ih = in.size().height;
	const cv::Vec2d across = cv::Vec2d(M(0, 0), M(0,1));
	const cv::Vec2d down = cv::Vec2d(M(1, 0), M(1, 1));
	const cv::Vec2d p0 = inOrig - M*outOrig;
	const cv::Vec2d p1 = p0 + w*across;
	const cv::Vec2d p2 = p0 + h*down;
	const cv::Vec2d p3 = p0 + w*across + h*down;

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
	cv::Vec2d carriage_return = down - w*across;

	//If the patch being extracted is completely in the image then no 
	//check is needed with each point.
	if (min_x >= 0 && min_y >= 0 && max_x < iw - 1 && max_y < ih - 1)
	{
		cv::Vec2d p = p0;
		for (int i = 0; i < h; ++i, p += carriage_return)
			for (int j = 0; j < w; ++j, p += across)
				additionalUtility::cv_sample(in, p[0], p[1], out.ptr<double>(i)[j]);

		return 0;
	}
	else // Check each source location
	{
		// Store as doubles to avoid conversion cost for comparison
		const double x_bound = iw - 1;
		const double y_bound = ih - 1;
		int count = 0;
		cv::Vec2d p = p0;
		for (int i = 0; i<h; ++i, p += carriage_return) {
			for (int j = 0; j<w; ++j, p += across) {
				//Make sure that we are extracting pixels in the image
				if (0 <= p[0] && 0 <= p[1] && p[0] < x_bound && p[1] < y_bound)
					additionalUtility::cv_sample(in, p[0], p[1], out.ptr<double>(i)[j]);
				else {
					out.ptr<double>(i)[j] = defaultValue;
					++count;
				}
			}
		}
		return count;
	}
}

int additionalUtility::cv_transform(cv::Mat& in, cv::Mat& out, const cv::Matx<double, 2, 2>& M, const cv::Size& inOrig, const cv::Size& outOrig, const double defaultValue)
{
	cv::Vec2d vec_inOrig = cv::Vec2d(inOrig.width, inOrig.height);
	cv::Vec2d vec_outOrig = cv::Vec2d(outOrig.width, outOrig.height);
	return additionalUtility::cv_transform(in, out, M, vec_inOrig, vec_outOrig, defaultValue);
}

double additionalUtility::getSubpix(const cv::Mat &img, cv::Point2d pt)
{
	cv::Mat patch;
	cv::getRectSubPix(img, cv::Size(1, 1), pt, patch);
	return patch.ptr<double>(0)[0];
}

double additionalUtility::getSubpix(const cv::Mat &img, cv::Vec2d vec)
{
	cv::Point2d pt(vec[0], vec[1]);
	return getSubpix(img, pt);
}
