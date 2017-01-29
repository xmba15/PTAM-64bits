// Copyright 2008 Isis Innovation Limited
#include "CalibCornerPatch.h"
#include "OpenGL.h"
#include <TooN/helpers.h>
#include <TooN/Cholesky.h>
#include "SmallMatrixOpts.h"
#include <math.h>

#if !_WIN64
#include <cvd/vector_image_ref.h>
#include <cvd/vision.h>
#include <cvd/utility.h>
#include <cvd/convolution.h>
#include <cvd/image_interpolate.h>
#endif

using namespace std;
using namespace CVD;

//Image<float> CalibCornerPatch::mimSharedSourceTemplate;
cv::Mat CalibCornerPatch::mimSharedSourceTemplate;

CalibCornerPatch::CalibCornerPatch(int nSideSize)
{
  /*mimTemplate.resize(ImageRef(nSideSize, nSideSize));
  mimGradients.resize(ImageRef(nSideSize, nSideSize));
  mimAngleJacs.resize(ImageRef(nSideSize, nSideSize));
  */
	cv::resize(mimTemplate, mimTemplate, cv::Size(nSideSize, nSideSize));
	cv::resize(mimGradients[0], mimGradients, cv::Size(nSideSize, nSideSize));
	cv::resize(mimGradients[1], mimGradients, cv::Size(nSideSize, nSideSize));
	cv::resize(mimAngleJacs[0], mimAngleJacs, cv::Size(nSideSize, nSideSize));
	cv::resize(mimAngleJacs[1], mimAngleJacs, cv::Size(nSideSize, nSideSize));

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

 /* Image<float> imToBlur(mimTemplate.size() + ImageRef(nExtraPixels, nExtraPixels));
  Image<float> imTwiceToBlur(imToBlur.size() * 2);*/

  cv::Mat imToBlur(mimTemplate.size() + cv::Size(nExtraPixels, nExtraPixels), CV_32FC1);
  cv::Mat imTwiceToBlur(imToBlur.size() * 2, CV_32FC1);

  // Make actual template:
  int nOffset;
  {
    Matrix<2> m2Warp = mParams.m2Warp();
    //CVD::transform(mimSharedSourceTemplate, imTwiceToBlur,
		  // M2Inverse(m2Warp),
		  // vec(mimSharedSourceTemplate.size() - ImageRef(1,1)) * 0.5,
		  // vec(imTwiceToBlur.size() - ImageRef(1,1)) * 0.5);
    //halfSample(imTwiceToBlur, imToBlur);
    //convolveGaussian(imToBlur, dBlurSigma);
	//nOffset = (imToBlur.size().x - mimTemplate.size().x) / 2;
	//cvd::copy(imToBlur, mimTemplate, mimTemplate.size(), ImageRef(nOffset, nOffset));

	cv_transform(mimSharedSourceTemplate, imTwiceToBlur,
		M2Inverse(m2Warp),
		size2Vec(mimSharedSourceTemplate.size() - cv::Size(1, 1))* 0.5,
		size2Vec(imTwiceToBlur.size() - cv::Size(1, 1)) * 0.5);
	cv::pyrDown(imTwiceToBlur, imToBlur, imToBlur.size());
	cv::GaussianBlur(imToBlur, imToBlur, cv::Size(ksize, ksize), dBlurSigma, 3.0);
	nOffset = (imToBlur.size().width - mimTemplate.size().width) / 2;
	imToBlur(cv::Rect(nOffset, nOffset, mimTemplate.size().width, mimTemplate.size().height)).copyTo(mimTemplate);
  };
  
  // Make numerical angle jac images:
  for(int dof=0; dof<2; dof++)
    {
      Matrix<2> m2Warp;
      for(int i=0; i<2; i++)
	{
	  double dAngle = mParams.v2Angles[i];
	  if(dof == i)
	    dAngle += 0.01;
	  m2Warp[0][i] = cos(dAngle);
	  m2Warp[1][i] = sin(dAngle);
	};
      
      //CVD::transform(mimSharedSourceTemplate, imTwiceToBlur,
		    // M2Inverse(m2Warp),
		    // vec(mimSharedSourceTemplate.size() - ImageRef(1,1)) * 0.5,
		    // vec(imTwiceToBlur.size() - ImageRef(1,1)) * 0.5);
      //halfSample(imTwiceToBlur, imToBlur);
      //convolveGaussian(imToBlur, dBlurSigma);
	  cv_transform(mimSharedSourceTemplate, imToBlur,
		  M2Inverse(m2Warp),
		  size2Vec(mimSharedSourceTemplate.size() - cv::Size(1, 1))* 0.5,
		  size2Vec(imTwiceToBlur.size() - cv::Size(1, 1)) * 0.5);
	  cv::pyrDown(imTwiceToBlur, imToBlur, imToBlur.size());
	  cv::GaussianBlur(imToBlur, imToBlur, cv::Size(ksize, ksize), dBlurSigma, 3.0);

	  //     ImageRef ir;
	  //     do
	  //mimAngleJacs[ir][dof] = (imToBlur[ir + ImageRef(nOffset, nOffset)] - mimTemplate[ir]) / 0.01;
	  //     while(ir.next(mimTemplate.size()));

	  for (int i = 0; i < mimTemplate.rows; i++) {
		  for (int j = 0; j < mimTemplate.cols; j++) {
			  mimAngleJacs[dof].ptr<float>(i)[j] = (imToBlur.ptr<float>(i + nOffset)[j + nOffset] - mimTemplate.ptr<float>(i)[j]) / 0.01;
		  }
	  }

    };
  
  // Make the image of image gradients here too (while we have the bigger template to work from)
 // ImageRef ir;
 // do
 //   {
 //     mimGradients[ir][0] = 0.5 * 
	//(imToBlur[ir + ImageRef(nOffset + 1, nOffset)] - 
	// imToBlur[ir + ImageRef(nOffset - 1, nOffset)]);
 //     mimGradients[ir][1] = 0.5 * 
	//(imToBlur[ir + ImageRef(nOffset, nOffset + 1 )] - 
	// imToBlur[ir + ImageRef(nOffset, nOffset - 1 )]);
 //   }
 // while(ir.next(mimGradients.size()));

  for (int i = 0; i < mimTemplate.rows; i++) {
	  for (int j = 0; j < mimTemplate.cols; j++) {
		  mimGradients[0].ptr<float>(i)[j] = 0.5 * (imToBlur.ptr<float>(i + nOffset + 1)[j + nOffset] -
			  imToBlur.ptr<float>(i + nOffset - 1)[j + nOffset] );
		  mimGradients[1].ptr<float>(i)[j] = 0.5 * (imToBlur.ptr<float>(i + nOffset)[j + nOffset + 1] -
			  imToBlur.ptr<float>(i + nOffset)[j + nOffset - 1]);
	  }
  }
}

bool CalibCornerPatch::IterateOnImageWithDrawing(CalibCornerPatch::Params &params, cv::Mat &im)
{
	bool bReturn = IterateOnImage(params, im);
	if (!bReturn)
	{
		glPointSize(3);
		glColor3f(1, 0, 0);
		glBegin(GL_POINTS);
		glVertex(params.v2Pos);
		glEnd();
	}
	return bReturn;
}

bool CalibCornerPatch::IterateOnImage(CalibCornerPatch::Params &params, cv::Mat &im)
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

double CalibCornerPatch::Iterate(cv::Mat &im)
{
	TooN::Vector<2> v2TL = mParams.v2Pos - size2Vec(mimTemplate.size() - cv::Size(1, 1)) / 2.0;
	if (!(v2TL[0] >= 0.0 && v2TL[1] >= 0.0))
		return -1.0;
	TooN::Vector<2> v2BR = v2TL + size2Vec(mimTemplate.size() - cv::Size(1, 1));
	if (!(v2BR[0] < (im.size().width - 1.0) && v2BR[1] < (im.size().height - 1.0)))
		return -1.0;

	//image_interpolate<Interpolate::Bilinear, byte> imInterp(im);
	TooN::Matrix<6> m6JTJ = TooN::Zeros;
	TooN::Vector<6> v6JTD = TooN::Zeros;

	double dSum = 0.0;
	for (int i = 0; i < mimTemplate.rows; i++) {
		for (int j = 0; j < mimTemplate.cols; j++) {
			TooN::Vector<2> v2Pos_Template = size2Vec(cv::Size(j, i)) - size2Vec(mimTemplate.size() - cv::Size(1, 1)) / 2.0;
			TooN::Vector<2> v2Pos_Image = mParams.v2Pos + v2Pos_Template;
			double dDiff = getSubpix(im, cv::Point2d(v2Pos_Image[0], v2Pos_Image[1])) -
				(mParams.dGain * mimTemplate.ptr<float>(i)[j] + mParams.dMean);
			dSum += fabs(dDiff);
			TooN::Vector<6> v6Jac;
			// Jac for center pos: Minus sign because +pos equates to sliding template -
			v6Jac[0] = -1.0 * mParams.dGain * mimGradients[0].ptr<float>(i)[j];
			v6Jac[1] = -1.0 * mParams.dGain * mimGradients[1].ptr<float>(i)[j];
			// Jac for angles: dPos/dAngle needs finishing by multiplying by pos..
			v6Jac[2] = mimAngleJacs[0].ptr<float>(i)[j] * mParams.dGain;
			v6Jac[3] = mimAngleJacs[1].ptr<float>(i)[j] * mParams.dGain;
			// Jac for mean:
			v6Jac[4] = 1.0;
			// Jac for gain:
			v6Jac[5] = mimTemplate.ptr<float>(i)[j];

			m6JTJ += v6Jac.as_col() * v6Jac.as_row();
			v6JTD += dDiff * v6Jac;
		}
	}

	  TooN::Cholesky<6> chol(m6JTJ);
	  TooN::Vector<6> v6Update = 0.7 * chol.backsub(v6JTD);
	  mParams.v2Pos += v6Update.slice<0,2>();
	  mParams.v2Angles += v6Update.slice<2,2>();
	  mParams.dMean += v6Update[4];
	  mParams.dGain += v6Update[5];
	  mdLastError = dSum / mimTemplate.size().area();
	  return sqrt(v6Update.slice<0,2>() * v6Update.slice<0,2>());
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

Matrix<2> CalibCornerPatch::Params::m2Warp()
{
  Matrix<2> m2Warp;
  for(int i=0; i<2; i++)
    {
      m2Warp[0][i] = cos(v2Angles[i]);
      m2Warp[1][i] = sin(v2Angles[i]);
    };
  return m2Warp;
}
