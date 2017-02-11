// Copyright 2008 Isis Innovation Limited
#include "CalibCornerPatch.h"
#include "OpenGL.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
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
  double dBlurSigma = 2.0;
  int nExtraPixels = (int) (dBlurSigma * 6.0) + 2;
  int ksize = (int)ceil(dBlurSigma * 3.0);

  cv::Mat_<double> imToBlur(mimTemplate.size() + cv::Size(nExtraPixels, nExtraPixels));
  cv::Mat_<double> imTwiceToBlur(imToBlur.size() * 2);

  // Make actual template:
  int nOffset;
  {
    cv::Matx<double, 2, 2> m2Warp = mParams.m2Warp();
	
	/*additionalUtility::cv_transform(mimSharedSourceTemplate, imTwiceToBlur, 
		additionalUtility::M2Inverse(m2Warp,
			0.5 * mimSharedSourceTemplate.size());*/

	additionalUtility::cv_transform(mimSharedSourceTemplate, imTwiceToBlur,
		additionalUtility::M2Inverse(m2Warp),
		0.5 * cv::Vec2d(mimSharedSourceTemplate.cols - 1, mimSharedSourceTemplate.rows - 1),
		0.5 * cv::Vec2d(imTwiceToBlur.cols - 1, imTwiceToBlur.rows - 1)
		);
	cv::pyrDown(imTwiceToBlur, imToBlur, imToBlur.size());
	cv::GaussianBlur(imToBlur, imToBlur, cv::Size(ksize, ksize), dBlurSigma, 3.0);
	nOffset = (imToBlur.size().width - mimTemplate.size().width) / 2;
	imToBlur(cv::Rect(nOffset, nOffset, mimTemplate.size().width, mimTemplate.size().height)).copyTo(mimTemplate);
  };
  
  // Make numerical angle jac images:
  for(int dof=0; dof<2; dof++)
    {
      cv::Matx<double, 2, 2> m2Warp;
      for(int i=0; i<2; i++)
	{
	  double dAngle = mParams.v2Angles[i];
	  if(dof == i)
	    dAngle += 0.01;
	  m2Warp(0,i) = cos(dAngle);
	  m2Warp(1,i) = sin(dAngle);
	};
      
	  additionalUtility::cv_transform(mimSharedSourceTemplate, imToBlur,
		  additionalUtility::M2Inverse(m2Warp),
		  0.5 * cv::Vec2d(mimSharedSourceTemplate.cols - 1, mimSharedSourceTemplate.rows - 1),
		  0.5 * cv::Vec2d(imTwiceToBlur.cols - 1, imTwiceToBlur.rows - 1)
	  );
		  cv::pyrDown(imTwiceToBlur, imToBlur, imToBlur.size());
		  cv::GaussianBlur(imToBlur, imToBlur, cv::Size(ksize, ksize), dBlurSigma, 3.0);

	  for (int i = 0; i < mimTemplate.rows; i++) {
		  cv::Vec2d* aRowPtr = mimAngleJacs.ptr<cv::Vec2d>(i);
		  double* bRowPtr = imToBlur.ptr<double>(i + nOffset);
		  double* tRowPtr = mimTemplate.ptr<double>(i);
		  for (int j = 0; j < mimTemplate.cols; j++) {
			  aRowPtr[j][dof] = (bRowPtr[j + nOffset] - tRowPtr[j]) / 0.01;
		  }
	  }

    };
  
  for (int i = 0; i < mimTemplate.rows; i++) {
	  cv::Vec2d* gRowPtr = mimGradients.ptr<cv::Vec2d>(i);
	  double* bRowPtr = imToBlur.ptr<double>(i + nOffset);
	  double* bRowPtr1 = imToBlur.ptr<double>(i + nOffset + 1);
	  double* bRowPtr_1 = imToBlur.ptr<double>(i + nOffset - 1);
	  for (int j = 0; j < mimTemplate.cols; j++) {
		  gRowPtr[j][0] = 0.5 * (bRowPtr1[j + nOffset] -
			  bRowPtr_1[j + nOffset]);
		  gRowPtr[j][1] = 0.5 * (bRowPtr[j + nOffset + 1] -
			  bRowPtr[j + nOffset - 1]);
	  }
  }
}

bool CalibCornerPatch::IterateOnImageWithDrawing(CalibCornerPatch::Params &params, cv::Mat_<uchar> &im)
{
	bool bReturn = IterateOnImage(params, im);
	if (!bReturn)
	{
		glPointSize(3);
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
	if (fabs(sin(mParams.v2Angles[0] - mParams.v2Angles[1])) < sin(M_PI / 6.0))
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
	if (!(v2TL[0] >= 0.0 && v2TL[1] >= 0.0))
		return -1.0;
	cv::Vec2d v2BR = cv::Vec2d(v2TL[0] + (mimTemplate.cols - 1), v2TL[1] + (mimTemplate.rows - 1));
	if (!(v2BR[0] < (im.cols - 1.0) && v2BR[1] < (im.rows - 1.0)))
		return -1.0;

	//image_interpolate<Interpolate::Bilinear, byte> imInterp(im);
	cv::Matx<double, 6, 6> m6JTJ = cv::Matx<double, 6, 6>::zeros();
	cv::Vec<double, 6> v6JTD;

	double dSum = 0.0;
	for (int i = 0; i < mimTemplate.rows; i++) {
		for (int j = 0; j < mimTemplate.cols; j++) {
			cv::Vec2d v2Pos_Template = cv::Vec2d(j - (mimTemplate.cols - 1) / 2.0, i - (mimTemplate.rows - 1) / 2.0);
			cv::Vec2d v2Pos_Image = mParams.v2Pos + v2Pos_Template;
			double dDiff = getSubpix(im, cv::Point2d(v2Pos_Image[0], v2Pos_Image[1])) -
				(mParams.dGain * mimTemplate.ptr<float>(i)[j] + mParams.dMean);
			dSum += fabs(dDiff);
			cv::Vec<double, 6> v6Jac;
			// Jac for center pos: Minus sign because +pos equates to sliding template -
			v6Jac[0] = -1.0 * mParams.dGain * mimGradients.ptr<cv::Vec2d>(i)[j][0];
			v6Jac[1] = -1.0 * mParams.dGain * mimGradients.ptr<cv::Vec2d>(i)[j][1];
			// Jac for angles: dPos/dAngle needs finishing by multiplying by pos..
			v6Jac[2] = mimAngleJacs.ptr<cv::Vec2d>(i)[j][0] * mParams.dGain;
			v6Jac[3] = mimAngleJacs.ptr<cv::Vec2d>(i)[j][1] * mParams.dGain;
			// Jac for mean:
			v6Jac[4] = 1.0;
			// Jac for gain:
			v6Jac[5] = mimTemplate.ptr<double>(i)[j];

			m6JTJ += v6Jac * v6Jac.t();
			v6JTD += dDiff * v6Jac;
		}
	}

	  cv::Vec<double, 6> v6Update; // the 6x1 LS solution
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
  
  cv::resize(mimSharedSourceTemplate, mimSharedSourceTemplate, cv::Size(nSideSize, nSideSize));

  for (int x = 0; x < mimSharedSourceTemplate.rows; x++) {
	  for (int y = 0; y < mimSharedSourceTemplate.cols; y++) {
		  float fX = (x < nHalf) ? 1.0 : -1.0;
		  float fY = (y < nHalf) ? 1.0 : -1.0;
		  mimSharedSourceTemplate.ptr<float>(x)[y] = fX * fY;
	  }
  }
}

CalibCornerPatch::Params::Params()
{
  v2Angles[0] = 0.0;
  v2Angles[1] = M_PI / 2.0;
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
