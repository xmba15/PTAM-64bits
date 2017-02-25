#include "MiniPatch.h"

// Scoring function
inline int MiniPatch::SSDAtPoint(cv::Mat_<uchar> &im, const cv::Point &ir)
{
	if (!CvUtils::in_image_with_border(ir.y, ir.x, im, mnHalfPatchSize, mnHalfPatchSize))
		return mnMaxSSD + 1;
	cv::Point irImgBase(ir.x - mnHalfPatchSize, ir.y - mnHalfPatchSize);
	int nRows = mimOrigPatch.rows;
	int nCols = mimOrigPatch.cols;
	unsigned char *imagepointer;
	unsigned char *templatepointer;
	int nDiff;
	int nSumSqDiff = 0;

	for (int nRow = 0; nRow < nRows; nRow++)
	{
		imagepointer = im.ptr<uchar>(irImgBase.y + nRow, irImgBase.x);
		templatepointer = mimOrigPatch.ptr<uchar>(nRow);

		for (int nCol = 0; nCol < nCols; nCol++)
		{
			nDiff = imagepointer[nCol] - templatepointer[nCol];
			nSumSqDiff += nDiff * nDiff;
		}
	}
	return nSumSqDiff;
}

// Find a patch by searching at FAST corners in an input image
// If available, a row-corner LUT is used to speed up search through the
// FAST corners
bool MiniPatch::FindPatch(cv::Point &irPos,
	cv::Mat_<uchar> &im,
	int nRange,
	std::vector<cv::Point> &vCorners,
	std::vector<int> *pvRowLUT)
{
	cv::Point irCenter = irPos;
	cv::Point irBest;
	int nBestSSD = mnMaxSSD + 1;
	cv::Point irBBoxTL = irPos - cv::Point(nRange, nRange);
	cv::Point irBBoxBR = irPos + cv::Point(nRange, nRange);

	vector<cv::Point>::iterator i;
	if (!pvRowLUT)
	{	
		for (i = vCorners.begin(); i != vCorners.end(); i++)
			if (i->y >= irBBoxTL.y) break;
	}
	else
	{
		int nTopRow = irBBoxTL.y;
		if (nTopRow < 0)
			nTopRow = 0;
		if (nTopRow >= (int)pvRowLUT->size())
			nTopRow = (int)pvRowLUT->size() - 1;
		i = vCorners.begin() + (*pvRowLUT)[nTopRow];
	}

	for (; i != vCorners.end(); i++)
	{
		if (i->x < irBBoxTL.x || i->x > irBBoxBR.x)
			continue;
		if (i->y > irBBoxBR.y)
			break;
		int nSSD = SSDAtPoint(im, *i);

		if (nSSD < nBestSSD)
		{
			irBest = *i;
			nBestSSD = nSSD;
		}
	}
	if (nBestSSD < mnMaxSSD)
	{
		irPos = irBest;
		return true;
	}
	else
		return false;
}

// Define the patch from an input image
void MiniPatch::SampleFromImage(cv::Point irPos, cv::Mat_<uchar> &im)
{
	assert(CvUtils::in_image_with_border(irPos.y, irPos.x, im, mnHalfPatchSize, mnHalfPatchSize));
	cv::Size irPatchSize(2 * mnHalfPatchSize + 1, 2 * mnHalfPatchSize + 1);

	// Top-left corner of the patch to be copied into mimOrigpatch
	cv::Point TLCorner(irPos.x - mnHalfPatchSize, irPos.y - mnHalfPatchSize);
	// And the lower - right corner of ther region to be lifted in the image
	cv::Point LRCorner(TLCorner.x + irPatchSize.width, TLCorner.y + irPatchSize.height);

	cv::Mat_<uchar> tempPatch = im(cv::Range(TLCorner.y, LRCorner.y), cv::Range(TLCorner.x, LRCorner.x));
	// copyTo() should do the necessary allocation and sizing
	tempPatch.copyTo(mimOrigPatch);
}

// Static members
int MiniPatch::mnHalfPatchSize = 4;
int MiniPatch::mnRange = 10;
int MiniPatch::mnMaxSSD = 9999;
