#include "ShiTomasi.h"
#include <math.h>

double FindShiTomasiScoreAtPoint(cv::Mat_<uchar> &image,
	int nHalfBoxSize,
	cv::Point irCenter)
{
	double dXX = 0;
	double dYY = 0;
	double dXY = 0;

	cv::Point irStart = irCenter - cv::Point(nHalfBoxSize, nHalfBoxSize);
	cv::Point irEnd = irCenter + cv::Point(nHalfBoxSize, nHalfBoxSize);

	for (int i = irStart.y; i < irEnd.y; i++)
		for (int j = irStart.x; j < irEnd.x; j++)
		{
			double dx = image.ptr<uchar>(i)[j + 1] - image.ptr<uchar>(i)[j - 1];
			double dy = image.ptr<uchar>(i + 1)[j] - image.ptr<uchar>(i - 1)[j];
			dXX += dx*dx;
			dYY += dy*dy;
			dXY += dx*dy;
		}

	int nPixels = (irEnd.x - irStart.x + 1) * (irEnd.y - irStart.y + 1);
	dXX = dXX / (2.0 * nPixels);
	dYY = dYY / (2.0 * nPixels);
	dXY = dXY / (2.0 * nPixels);

	// Find and return smaller eigenvalue:
	return 0.5 * (dXX + dYY - sqrt((dXX + dYY) * (dXX + dYY) - 4 * (dXX * dYY - dXY * dXY)));
}
