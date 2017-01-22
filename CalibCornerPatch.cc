// Copyright 2008 Isis Innovation Limited
#include "CalibCornerPatch.h"
#include "OpenGL.h"
#include <TooN/helpers.h>
#include <cvd/vector_image_ref.h>
#include <cvd/vision.h>
#include <cvd/utility.h>
#include <cvd/convolution.h>
#include <cvd/image_interpolate.h>
#include <TooN/Cholesky.h>
#include "SmallMatrixOpts.h"


using namespace std;
using namespace CVD;

//Image<float> CalibCornerPatch::mimSharedSourceTemplate;
cv::Mat CalibCornerPatch::mimSharedSourceTemplate;

inline void cv_sample(const BasicImage<float>& im, double x, double y, float& result)
{
	int lx = (int)x;
	int ly = (int)y;
	int w = im.size().x;
	const float* base = im[ly] + lx;
	float a = base[0];
	float b = base[1];
	float c = base[w];
	float d = base[w + 1];
	float e = a - b;
	x -= lx;
	y -= ly;
	result = (float)(x*(y*(e - c + d) - e) + y*(c - a) + a);
}
int cv_transform(cv::Mat& in, cv::Mat& out, const TooN::Matrix<2>& M, const TooN::Vector<2>& inOrig, const TooN::Vector<2>& outOrig, const TooN::Vector<2>& defaultValue)
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
		for (int i = 0; i<h; ++i, p += carriage_return)
			for (int j = 0; j<w; ++j, p += across)
				sample(in, p[0], p[1], out[i][j]);
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
					sample(in, p[0], p[1], out[i][j]);
				else {
					out[i][j] = defaultValue;
					++count;
				}
			}
		}
		return count;
	}
}

CalibCornerPatch::CalibCornerPatch(int nSideSize)
{
  /*mimTemplate.resize(ImageRef(nSideSize, nSideSize));
  mimGradients.resize(ImageRef(nSideSize, nSideSize));
  mimAngleJacs.resize(ImageRef(nSideSize, nSideSize));
  */
	cv::resize(mimTemplate, mimTemplate, cv::Size(nSideSize, nSideSize));
	cv::resize(mimGradients, mimGradients, cv::Size(nSideSize, nSideSize));
	cv::resize(mimAngleJacs, mimAngleJacs, cv::Size(nSideSize, nSideSize));

  //if(mimSharedSourceTemplate.size().x == 0)
  //  {
  //    MakeSharedTemplate();
  //  }
	if (mimSharedSourceTemplate.size().width == 0)
	{
		MakeSharedTemplate();
	}
}

void CalibCornerPatch::MakeTemplateWithCurrentParams()
{
  double dBlurSigma = 2.0;
  int nExtraPixels = (int) (dBlurSigma * 6.0) + 2;
  
 /* Image<float> imToBlur(mimTemplate.size() + ImageRef(nExtraPixels, nExtraPixels));
  Image<float> imTwiceToBlur(imToBlur.size() * 2);*/

  cv::Mat imToBlur(mimTemplate.size() + cv::Size(nExtraPixels, nExtraPixels), CV_32FC1);
  cv::Mat imTwiceToBlur(imToBlur.size() * 2, CV_32FC1);

  // Make actual template:
  int nOffset;
  {
    Matrix<2> m2Warp = mParams.m2Warp();
    CVD::transform(mimSharedSourceTemplate, imTwiceToBlur,
		   M2Inverse(m2Warp),
		   vec(mimSharedSourceTemplate.size() - ImageRef(1,1)) * 0.5,
		   vec(imTwiceToBlur.size() - ImageRef(1,1)) * 0.5);
    halfSample(imTwiceToBlur, imToBlur);
    convolveGaussian(imToBlur, dBlurSigma);
    
    nOffset = (imToBlur.size().x - mimTemplate.size().x) / 2;
    copy(imToBlur, mimTemplate, mimTemplate.size(), ImageRef(nOffset, nOffset));
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
      
      CVD::transform(mimSharedSourceTemplate, imTwiceToBlur,
		     M2Inverse(m2Warp),
		     vec(mimSharedSourceTemplate.size() - ImageRef(1,1)) * 0.5,
		     vec(imTwiceToBlur.size() - ImageRef(1,1)) * 0.5);
      halfSample(imTwiceToBlur, imToBlur);
      convolveGaussian(imToBlur, dBlurSigma);
      ImageRef ir;
      do
	mimAngleJacs[ir][dof] = (imToBlur[ir + ImageRef(nOffset, nOffset)] - mimTemplate[ir]) / 0.01;
      while(ir.next(mimTemplate.size()));
    };
  
  // Make the image of image gradients here too (while we have the bigger template to work from)
  ImageRef ir;
  do
    {
      mimGradients[ir][0] = 0.5 * 
	(imToBlur[ir + ImageRef(nOffset + 1, nOffset)] - 
	 imToBlur[ir + ImageRef(nOffset - 1, nOffset)]);
      mimGradients[ir][1] = 0.5 * 
	(imToBlur[ir + ImageRef(nOffset, nOffset + 1 )] - 
	 imToBlur[ir + ImageRef(nOffset, nOffset - 1 )]);
    }
  while(ir.next(mimGradients.size()));
}

bool CalibCornerPatch::IterateOnImageWithDrawing(CalibCornerPatch::Params &params, Image<byte> &im)
{
  bool bReturn = IterateOnImage(params, im);
  if(!bReturn)
    {
      glPointSize(3);
      glColor3f(1,0,0);
      glBegin(GL_POINTS);
      glVertex(params.v2Pos);
      glEnd();
    }
  return bReturn;
}

bool CalibCornerPatch::IterateOnImage(CalibCornerPatch::Params &params, Image<byte> &im)
{
  mParams = params;
  double dLastUpdate = 0.0;
  for(int i=0; i<20; i++)
    {
      MakeTemplateWithCurrentParams();
      dLastUpdate = Iterate(im);
      
      if(dLastUpdate < 0)
	return false;
      if(dLastUpdate < 0.00001)
	break;
    }
  if(dLastUpdate > 0.001)
    return false;
  if(fabs(sin(mParams.v2Angles[0] - mParams.v2Angles[1])) < sin(M_PI / 6.0))
    return false;
  if(fabs(mParams.dGain) < 20.0)
    return false;
  if(mdLastError > 25.0)
    return false;
  
  params = mParams;
  return true;
}

double CalibCornerPatch::Iterate(Image<byte> &im)
{ 
  Vector<2> v2TL = mParams.v2Pos - vec(mimTemplate.size() - ImageRef(1,1)) / 2.0;
  if(!(v2TL[0] >= 0.0 && v2TL[1] >= 0.0))
    return -1.0;
  Vector<2> v2BR = v2TL + vec(mimTemplate.size() - ImageRef(1,1));
  if(!(v2BR[0] < (im.size().x - 1.0) && v2BR[1] < (im.size().y - 1.0)))
    return -1.0;
  
  image_interpolate<Interpolate::Bilinear, byte> imInterp(im);
  Matrix<6> m6JTJ = Zeros;
  Vector<6> v6JTD = Zeros;

  
  
  ImageRef ir;
  double dSum = 0.0;
  do
    {
      Vector<2> v2Pos_Template = vec(ir) - vec(mimTemplate.size() - ImageRef(1,1)) / 2.0;
      Vector<2> v2Pos_Image = mParams.v2Pos + v2Pos_Template;
      double dDiff = imInterp[v2Pos_Image] - (mParams.dGain * mimTemplate[ir] + mParams.dMean);
      dSum += fabs(dDiff);
      Vector<6> v6Jac;
      // Jac for center pos: Minus sign because +pos equates to sliding template -
      v6Jac.slice<0,2>() = -1.0 * mParams.dGain * mimGradients[ir]; 
      // Jac for angles: dPos/dAngle needs finishing by multiplying by pos..
      v6Jac[2] =  mimAngleJacs[ir][0] * mParams.dGain;
      v6Jac[3] =  mimAngleJacs[ir][1] * mParams.dGain;
      // Jac for mean:
      v6Jac[4] = 1.0; 
      // Jac for gain:
      v6Jac[5] = mimTemplate[ir];     
      
      m6JTJ += v6Jac.as_col() * v6Jac.as_row();
      v6JTD += dDiff * v6Jac;
    }
  while(ir.next(mimTemplate.size()));
  
  Cholesky<6> chol(m6JTJ);
  Vector<6> v6Update = 0.7 * chol.backsub(v6JTD);
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
  
  //mimSharedSourceTemplate.resize(ImageRef(nSideSize, nSideSize));
  cv::resize(mimSharedSourceTemplate, mimSharedSourceTemplate, cv::Size(nSideSize, nSideSize));
  //ImageRef ir;
  
  for (int x = 0; x < mimSharedSourceTemplate.rows; x++) {
	  for (int y = 0; y < mimSharedSourceTemplate.cols; y++) {
		  float fX = (x < nHalf) ? 1.0 : -1.0;
		  float fY = (y < nHalf) ? 1.0 : -1.0;
		  mimSharedSourceTemplate.at<float>(x, y) = fX * fY;
	  }
  }
  //do
  //  {
  //    float fX = (ir.x < nHalf) ? 1.0 : -1.0;
  //    float fY = (ir.y < nHalf) ? 1.0 : -1.0;
  //    mimSharedSourceTemplate[ir] = fX * fY;
  //  }
  //while(ir.next(mimSharedSourceTemplate.size()));
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