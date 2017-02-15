#include "CalibCornerPatch.h"
#include "OpenGL.h"
#include <math.h>
#include "GCVD/image_interpolate.h"

using namespace additionalUtility;

cv::Mat_<double> CalibCornerPatch::mimSharedSourceTemplate;

CalibCornerPatch::CalibCornerPatch(int nSideSize)
{
	mimTemplate.create(cv::Size(nSideSize, nSideSize));
	mimGradients.create(cv::Size(nSideSize, nSideSize));
	mimAngleJacs.create(cv::Size(nSideSize, nSideSize));

	if (mimSharedSourceTemplate.size().width == 0)
	{
		MakeSharedTemplate();
	}
}

void CalibCornerPatch::MakeTemplateWithCurrentParams()
{
	int r, c;
	double dBlurSigma = 2.0;
	int nExtraPixels = (int)ceil(dBlurSigma * 3.0);

	cv::Mat_<double> imToBlur; 
	int im2BlurRows = mimTemplate.rows + nExtraPixels;
	int im2BlurCols = mimTemplate.cols + nExtraPixels;
	cv::Mat_<double> imTwiceToBlur(im2BlurRows * 2, im2BlurCols * 2);

	int nOffsetCol, nOffsetRow;
	{
		cv::Matx<double, 2, 2> m2Warp = mParams.m2Warp();
		CvUtils::transform<>(mimSharedSourceTemplate,
			imTwiceToBlur,
			CvUtils::M2Inverse(m2Warp),
			0.5 * cv::Vec2d(mimSharedSourceTemplate.cols - 1, mimSharedSourceTemplate.rows - 1),
			0.5 * cv::Vec2d(imTwiceToBlur.cols - 1, imTwiceToBlur.rows - 1)
			);

		CvUtils::halfSample(imTwiceToBlur, imToBlur);
		int gkerSize = (int)ceil(dBlurSigma*3.0); // where 3.0 is the default "sigmas" parameter in libCVD
		gkerSize += gkerSize % 2 == 0 ? 1 : 0;
		cv::GaussianBlur(imToBlur, imToBlur, cv::Size(gkerSize, gkerSize), dBlurSigma);

		nOffsetCol = (imToBlur.cols - mimTemplate.cols) / 2;
		nOffsetRow = (imToBlur.rows - mimTemplate.rows) / 2;
		imToBlur(cv::Range(nOffsetRow, nOffsetRow + mimTemplate.rows),
			     cv::Range(nOffsetCol, nOffsetCol + mimTemplate.cols)).copyTo(mimTemplate);
	};

	for (int dof = 0; dof<2; dof++) {
		cv::Mat_<double> m2Warp(2, 2);
		for (int i = 0; i<2; i++)
		{
			double dAngle = mParams.v2Angles[i];
			if (dof == i) dAngle += 0.01;

			m2Warp(0, i) = cos(dAngle);
			m2Warp(1, i) = sin(dAngle);
		};
		CvUtils::transform<>(mimSharedSourceTemplate,
			imTwiceToBlur,
			CvUtils::M2Inverse(m2Warp),
			0.5 * cv::Vec2d(mimSharedSourceTemplate.cols - 1, mimSharedSourceTemplate.rows - 1),
			0.5 * cv::Vec2d(imTwiceToBlur.cols - 1, imTwiceToBlur.rows - 1)
			);
		
		CvUtils::halfSample(imTwiceToBlur, imToBlur);
		cv::Mat temp;
		int gkerSize = (int)ceil(dBlurSigma*3.0);
		gkerSize += gkerSize % 2 == 0 ? 1 : 0;
		cv::GaussianBlur(imToBlur, temp, cv::Size(gkerSize, gkerSize), dBlurSigma);
		temp.copyTo(imToBlur);

		for (r = 0; r < mimTemplate.rows; r++) {
			cv::Vec<double, 2>* aRowPtr = mimAngleJacs.ptr<cv::Vec<double, 2>>(r);
			double* bRowPtr = imToBlur.ptr<double>(r + nOffsetRow);
			double* tRowPtr = mimTemplate.ptr<double>(r);
			for (c = 0; c < mimTemplate.cols; c++)
				aRowPtr[c][dof] = (bRowPtr[c + nOffsetCol] - tRowPtr[c]) / 0.01;
		}
	} 

	for (r = 0; r < mimGradients.rows; r++) {
		double* bRowPtr = imToBlur.ptr<double>(r + nOffsetRow);
		double* bRowPtr1 = imToBlur.ptr<double>(r + nOffsetRow + 1);
		double* bRowPtr_1 = imToBlur.ptr<double>(r + nOffsetRow - 1);
		cv::Vec<double, 2>* gRowPtr = mimGradients.ptr<cv::Vec<double, 2>>(r);
		for (c = 0; c < mimGradients.cols; c++)
		{
			gRowPtr[c][0] = 0.5 * (bRowPtr[c + nOffsetCol + 1] - bRowPtr[c + nOffsetCol - 1]);
			gRowPtr[c][1] = 0.5 * (bRowPtr1[c + nOffsetCol] - bRowPtr_1[c + nOffsetCol]);
		}
	}
} 

bool CalibCornerPatch::IterateOnImageWithDrawing(CalibCornerPatch::Params &params, cv::Mat_<uchar> &im)
{
	bool bReturn = IterateOnImage(params, im);
	if (!bReturn)
	{
		glPointSize(5);
		glColor3f(1, 0, 0);
		glBegin(GL_POINTS);
		glVertex2d(params.v2Pos[0], params.v2Pos[1]);
		glEnd();
	}
	return bReturn;
}

bool CalibCornerPatch::IterateOnImage(CalibCornerPatch::Params &params, cv::Mat_<uchar> &im)
{
	mParams = params;
	double dLastUpdate = 0.0;
	for (int i = 0; i < 20; i++)
	{
		MakeTemplateWithCurrentParams();
		dLastUpdate = Iterate(im);

		if (dLastUpdate < 0)
			return false;
		if (dLastUpdate < 0.00001)
			break;
	}
	if (dLastUpdate > 0.001)
		return false;
	if (fabs(sin(mParams.v2Angles[0] - mParams.v2Angles[1])) < sin(CV_PI / 6.0))
		return false;
	if (fabs(mParams.dGain) < 20.0)
		return false;
	if (mdLastError > 25.0)
		return false;

	params = mParams;
	return true;
}

double CalibCornerPatch::Iterate(cv::Mat_<uchar> &im)
{
	cv::Vec2d v2TL = cv::Vec2d(mParams.v2Pos[0] - (mimTemplate.cols - 1) / 2.0, mParams.v2Pos[1] - (mimTemplate.rows - 1) / 2.0);
	if (!(v2TL[0] >= 0.0 && v2TL[1] >= 0.0)) return -1.0;

	cv::Vec2d v2BR = v2TL + cv::Vec2d(mimTemplate.cols - 1, mimTemplate.rows - 1);
	if ((v2BR[0] >= (im.cols - 1.0)) || (v2BR[1] >= (im.rows - 1.0))) return -1.0;

	CvUtils::image_interpolate< CvUtils::Interpolate::Bilinear, uchar> imInterp(im);

	cv::Matx<double, 6, 6> m6JTJ = cv::Matx<double, 6, 6>::zeros(); // this is the J'*J (can be though of as information matrix)
	cv::Vec<double, 6> v6JTD;       // J' * error (think information vector)
	double dSum = 0.0;
	for (int r = 0; r < mimTemplate.rows; r++) {
		double* tRowPtr = mimTemplate.ptr<double>(r);
		cv::Vec<double, 2>* gRowPtr = mimGradients.ptr<cv::Vec<double, 2>>(r);
		cv::Vec<double, 2>* aRowPtr = mimAngleJacs.ptr<cv::Vec<double, 2>>(r);
		for (int c = 0; c < mimTemplate.cols; c++)
		{
			cv::Vec2d v2Pos_Template(c - (mimTemplate.cols - 1) / 2.0, r - (mimTemplate.rows - 1) / 2.0);
			cv::Vec2d v2Pos_Image = mParams.v2Pos + v2Pos_Template;
			double dDiff = imInterp[v2Pos_Image][0] - (mParams.dGain * tRowPtr[c] + mParams.dMean);

			dSum += fabs(dDiff);

			cv::Vec<double, 6> v6Jac;
			v6Jac[0] = -1.0 * mParams.dGain * gRowPtr[c][0];
			v6Jac[1] = -1.0 * mParams.dGain * gRowPtr[c][1];
			v6Jac[2] = aRowPtr[c][0] * mParams.dGain;
			v6Jac[3] = aRowPtr[c][1] * mParams.dGain,
			v6Jac[4] = 1.0;	
			v6Jac[5] = tRowPtr[c];
			m6JTJ += v6Jac * v6Jac.t();
			v6JTD += dDiff* v6Jac;
		}
	}
	cv::Vec<double, 6> v6Update;
	if (cv::determinant(m6JTJ) == 0) return 9999;
	cv::solve(m6JTJ, v6JTD, v6Update, cv::DECOMP_CHOLESKY);

	mParams.v2Pos += cv::Vec2d(v6Update[0], v6Update[1]);
	mParams.v2Angles += cv::Vec2d(v6Update[2], v6Update[3]);
	mParams.dMean += v6Update[4];
	mParams.dGain += v6Update[5];
	mdLastError = dSum / mimTemplate.size().area();
	return sqrt(v6Update[0] * v6Update[0] + v6Update[1] * v6Update[1]);
}

void CalibCornerPatch::MakeSharedTemplate()
{
  const int nSideSize = 100;
  const int nHalf = nSideSize >> 1;
  
  mimSharedSourceTemplate.create(cv::Size(nSideSize, nSideSize));

  for (int x = 0; x < mimSharedSourceTemplate.rows; x++) {
	  for (int y = 0; y < mimSharedSourceTemplate.cols; y++) {
		  double fX = (x < nHalf) ? 1.0 : -1.0;
		  double fY = (y < nHalf) ? 1.0 : -1.0;
		  mimSharedSourceTemplate.ptr<double>(x)[y] = fX * fY;
	  }
  }
}

CalibCornerPatch::Params::Params()
{
  v2Angles[0] = 0.0;
  v2Angles[1] = CV_PI / 2.0;
  dMean = 0.0;
  dGain = 1.0;
}

cv::Matx<double, 2, 2> CalibCornerPatch::Params::m2Warp()
{
	cv::Matx<double, 2, 2> m2Warp;
  for(int i=0; i<2; i++)
    {
      m2Warp(0, i) = cos(v2Angles[i]);
      m2Warp(1, i) = sin(v2Angles[i]);
    };
  return m2Warp;
}
