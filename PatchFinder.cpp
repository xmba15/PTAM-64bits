// Copyright 2008 Isis Innovation Limited
#include "PatchFinder.h"
#include "KeyFrame.h"

#include "GCVD/Addedutils.h"
#include "GCVD/image_interpolate.h"

#if HAVE_XMMINTRIN
#include <xmmintrin.h>
#endif

PatchFinder::PatchFinder(int nPatchSize)
  : mimTemplate(cv::Mat_<uchar>(nPatchSize,nPatchSize))
{
  mnPatchSize = nPatchSize;
  mirCenter = cv::Point(nPatchSize/2, nPatchSize/2);
  int nMaxSSDPerPixel = 500; // Pretty arbitrary... could make a GVar out of this.
  mnMaxSSD = mnPatchSize * mnPatchSize * nMaxSSDPerPixel;
  // Populate the speed-up caches with bogus values:
  mm2LastWarpMatrix = 9999.9 * cv::Matx<double, 2, 2>::eye();
  mpLastTemplateMapPoint = NULL;
}

// Find the warping matrix and search level
int PatchFinder::CalcSearchLevelAndWarpMatrix(MapPoint &p,
RigidTransforms::SE3<> se3CFromW, cv::Matx<double, 2, 2> &m2CamDerivs)
{
	// Calc point pos in new view camera frame
	// Slightly dumb that we re-calculate this here when the tracker's already done this!
	cv::Vec3d v3Cam = se3CFromW * p.v3WorldPos;
	double dOneOverCameraZ = 1.0 / v3Cam[2];

	// Project the source keyframe's one-pixel-right and one-pixel-down vectors into the current view
	cv::Vec3d v3MotionRight = se3CFromW.get_rotation() * p.v3PixelRight_W;
	cv::Vec3d v3MotionDown = se3CFromW.get_rotation() * p.v3PixelDown_W;
	// Calculate in-image derivatives of source image pixel motions:
	double invDepthRight = 1.0 / (v3Cam[2] + v3MotionRight[2]);
	double invDepthDown  = 1.0 / (v3Cam[2] + v3MotionDown[2]);

	mm2WarpInverse(0, 0) = (m2CamDerivs(0, 0) * (v3MotionRight[0] - v3MotionRight[2] * v3Cam[0] * dOneOverCameraZ) +
		                    m2CamDerivs(0, 1) * (v3MotionRight[1] - v3MotionRight[2] * v3Cam[1] * dOneOverCameraZ)) * invDepthRight;
	mm2WarpInverse(1, 0) = (m2CamDerivs(1, 0) * (v3MotionRight[0] - v3MotionRight[2] * v3Cam[0] * dOneOverCameraZ) +
		                    m2CamDerivs(1, 1) * (v3MotionRight[1] - v3MotionRight[2] * v3Cam[1] * dOneOverCameraZ)) * invDepthRight;

	mm2WarpInverse(0, 1) = (m2CamDerivs(0, 0) * (v3MotionDown[0] - v3MotionDown[2] * v3Cam[0] * dOneOverCameraZ) +
		                    m2CamDerivs(0, 1) * (v3MotionDown[1] - v3MotionDown[2] * v3Cam[1] * dOneOverCameraZ)) * invDepthDown; 
	mm2WarpInverse(1, 1) = (m2CamDerivs(1, 0) * (v3MotionDown[0] - v3MotionDown[2] * v3Cam[0] * dOneOverCameraZ) +
		                    m2CamDerivs(1, 1) * (v3MotionDown[1] - v3MotionDown[2] * v3Cam[1] * dOneOverCameraZ)) * invDepthDown;

	double dDet = mm2WarpInverse(0, 0) * mm2WarpInverse(1, 1) - mm2WarpInverse(0, 1) * mm2WarpInverse(1, 0);
	mnSearchLevel = 0;

	// This warp matrix is likely not appropriate for finding at level zero, which is 
	// the level at which it has been calculated. Vary the search level until the 
	// at that level would be appropriate (does not actually modify the matrix.)
	while (dDet > 3 && mnSearchLevel < LEVELS - 1)
	{
		mnSearchLevel++;
		dDet *= 0.25;
	}

	// Some warps are inappropriate, e.g. too near the camera, too far, or reflected, 
	// or zero area.. reject these!
	if (dDet > 3 || dDet < 0.25)
	{
		mbTemplateBad = true;
		return -1;
	}
	else
		return mnSearchLevel;
}

// This is just a convenience function wich caluclates the warp matrix and generates
// the template all in one call.
void PatchFinder::MakeTemplateCoarse(MapPoint &p,
	RigidTransforms::SE3<> se3CFromW,
	cv::Matx<double, 2, 2> &m2CamDerivs)
{
	CalcSearchLevelAndWarpMatrix(p, se3CFromW, m2CamDerivs);
	MakeTemplateCoarseCont(p);
}

// This function generates the warped search template.
void PatchFinder::MakeTemplateCoarseCont(MapPoint &p)
{
	// Get the warping matrix appropriate for use with CVD::transform...
	cv::Matx<double, 2, 2> m2 = CvUtils::M2Inverse(mm2WarpInverse) * LevelScale(mnSearchLevel);
	// m2 now represents the number of pixels in the source image for one 
	// pixel of template image

	// Optimisation: Don't re-gen the coarse template if it's going to be substantially the 
	// same as was made last time. This saves time when the camera is not moving. For this, 
	// check that (a) this patchfinder is still working on the same map point and (b) the 
	// warping matrix has not changed much.

	bool bNeedToRefreshTemplate = false;

	if (&p != mpLastTemplateMapPoint) bNeedToRefreshTemplate = true;
	// Still the same map point? Then compare warping matrix..
	for (int i = 0; !bNeedToRefreshTemplate && i < 2; i++)
	{
		cv::Vec2d v2Diff(m2(0, i) - mm2LastWarpMatrix(0, i),
			             m2(1, i) - mm2LastWarpMatrix(1, i));
		const double dRefreshLimit = 0.07;  // Sort of works out as half a pixel displacement in src img
		if (v2Diff[0] * v2Diff[0] + v2Diff[1] * v2Diff[1] > dRefreshLimit * dRefreshLimit)
			bNeedToRefreshTemplate = true;
	}

	// Need to regenerate template? Then go ahead.
	if (bNeedToRefreshTemplate)
	{
		int nOutside;  // Use CVD::transform to warp the patch according the the warping matrix m2
					   // This returns the number of pixels outside the source image hit, which should be zero.
		nOutside = CvUtils::transform(p.pPatchSourceKF->aLevels[p.nSourceLevel].im,
			                          mimTemplate,
			                          m2,
			                          cv::Vec2d(p.irCenter.x, p.irCenter.y),
			                          cv::Vec2d(mirCenter.x, mirCenter.y));

		if (nOutside)
			mbTemplateBad = true;
		else
			mbTemplateBad = false;

		MakeTemplateSums();

		// Store the parameters which allow us to determine if we need to re-calculate
		// the patch next time round.
		mpLastTemplateMapPoint = &p;
		mm2LastWarpMatrix = m2;
	}
}

// This makes a template without warping. Used for epipolar search, where we don't really know 
// what the warping matrix should be. (Although to be fair, I should do rotation for epipolar,
// which we could approximate without knowing patch depth!)
void PatchFinder::MakeTemplateCoarseNoWarp(KeyFrame &k, int nLevel, cv::Point irLevelPos)
{
	mnSearchLevel = nLevel;
	cv::Mat_<uchar> im = k.aLevels[nLevel].im;
	if (!CvUtils::in_image_with_border(irLevelPos.x, irLevelPos.y, im, mnPatchSize / 2 + 1, mnPatchSize / 2 + 1))
	{
		mbTemplateBad = true;
		return;
	}
	mbTemplateBad = false;

	int nOffsetRow = irLevelPos.y - mirCenter.y,
		nOffsetCol = irLevelPos.x - mirCenter.x;

	im(cv::Range(nOffsetRow, nOffsetRow + mimTemplate.rows),
		cv::Range(nOffsetCol, nOffsetCol + mimTemplate.cols)).copyTo(mimTemplate);

	MakeTemplateSums();
}

// Convenient wrapper for the above
void PatchFinder::MakeTemplateCoarseNoWarp(MapPoint &p)
{
	MakeTemplateCoarseNoWarp(*p.pPatchSourceKF, p.nSourceLevel, p.irCenter);
}

// Finds the sum, and sum-squared, of template pixels. These sums are used
// to calculate the ZMSSD.
inline void PatchFinder::MakeTemplateSums()
{
  int nSum = 0;
  int nSumSq = 0;
  for (int i = 0; i < mimTemplate.rows; i++) {
	  for (int j = 0; j < mimTemplate.cols; j++) {
		  int b = mimTemplate.ptr<uchar>(i)[j];
		  nSum += b;
		  nSumSq += b*b;
	  }
  }
  
  mnTemplateSum = nSum;
  mnTemplateSumSq = nSumSq;
}

// One of the main functions of the class! Looks at the appropriate level of 
// the target keyframe to try and find the template. Looks only at FAST corner points
// which are within radius nRange of the center. (Params are supplied in Level0
// coords.) Returns true on patch found.
bool PatchFinder::FindPatchCoarse(cv::Point irPos, KeyFrame &kf, unsigned int nRange)
{
	mbFound = false;

	// Convert from L0 coords to search level quantities
	int nLevelScale = LevelScale(mnSearchLevel);
	mirPredictedPos = irPos;
	irPos = irPos / nLevelScale;
	nRange = (nRange + nLevelScale - 1) / nLevelScale;

	// Bounding box of search circle
	int nTop = irPos.y - nRange;
	int nBottomPlusOne = irPos.y + nRange + 1;
	int nLeft = irPos.x - nRange;
	int nRight = irPos.x + nRange;

	// Ref variable for the search level
	Level &L = kf.aLevels[mnSearchLevel];

	// Some bounds checks on the bounding box..
	if (nTop < 0)
		nTop = 0;
	if (nTop >= L.im.size().y)
		return false;
	if (nBottomPlusOne <= 0)
		return false;

	// The next section finds all the FAST corners in the target level which 
	// are near enough the search center. It's a bit optimised to use 
	// a corner row look-up-table, since otherwise the routine
	// would spend a long time trawling throught the whole list of FAST corners!
	std::vector<cv::Point>::iterator i;
	std::vector<cv::Point>::iterator i_end;

	i = L.vCorners.begin() + L.vCornerRowLUT[nTop];

	if (nBottomPlusOne >= L.im.rows)
		i_end = L.vCorners.end();
	else
		i_end = L.vCorners.begin() + L.vCornerRowLUT[nBottomPlusOne];

	cv::Point irBest;             // Best match so far
	int nBestSSD = mnMaxSSD + 1; // Best score so far is beyond the max allowed

	for (; i < i_end; i++)          // For each corner ...
	{
		if (i->x < nLeft || i->x > nRight)
			continue;
		if ((irPos.x - i->x) * (irPos.x - i->x) +
			(irPos.y - i->y) * (irPos.y - i->y) > nRange * nRange)
			continue;              // ... reject all those not close enough..

		int nSSD;                // .. and find the ZMSSD at those near enough.
		nSSD = ZMSSDAtPoint(L.im, *i);
		if (nSSD < nBestSSD)      // Best yet?
		{
			irBest = *i;
			nBestSSD = nSSD;
		}
	} // done looping over corners

	if (nBestSSD < mnMaxSSD)      // Found a valid match?
	{
		mv2CoarsePos = LevelZeroPos(irBest, mnSearchLevel);
		mbFound = true;
	}
	else
		mbFound = false;
	return mbFound;
}

// Makes an inverse composition template out of the coarse template.
// Includes calculating image of derivatives (gradients.) The inverse composition
// used here operates on three variables: x offet, y offset, and difference in patch
// means; hence things like mm3HInv are dim 3, but the trivial mean jacobian 
// (always unity, for each pixel) is not stored.
void PatchFinder::MakeSubPixTemplate()
{
  mimJacs.create(mimTemplate.rows - 2, mimTemplate.cols - 2);
  
  cv::Matx<double, 3, 3> m3H = cv::Matx<double, 3, 3>::zeros(); // This stores jTj.

  for (int i = 1; i < mnPatchSize - 1; i++) {
	  uchar* pTempImRow0 = mimTemplate.ptr<uchar>(i);
	  uchar* pTempImRow_1 = mimTemplate.ptr<uchar>(i - 1);
	  uchar* pTempImRow1 = mimTemplate.ptr<uchar>(i + 1);
	  cv::Vec2d* gRowPtr = mimJacs.ptr<cv::Vec2d>(i - 1);
	  for (int j = 1; j < mnPatchSize - 1; j++) {
		  cv::Vec2d v2Grad(0.5 * (pTempImRow0[j + 1] - pTempImRow0[j - 1]),
			  0.5 * (pTempImRow1[j] - pTempImRow_1[j]));
		  gRowPtr[j - 1] = v2Grad;
		  // populate upper triagle of m3H with the gradient gram-matrix
		  m3H(0, 0) += v2Grad[0] * v2Grad[0]; m3H(0, 1) += v2Grad[0] * v2Grad[1]; m3H(0, 2) += v2Grad[0];
		  m3H(1, 1) += v2Grad[1] * v2Grad[1]; m3H(1, 2) += v2Grad[1];
		  m3H(2, 2) += 1;
	  }
  }


  
  // filling-in the lower triangle of m3H
  m3H(1, 0) = m3H(0, 1);
  m3H(2, 0) = m3H(0, 2); m3H(2, 1) = m3H(1, 2);

  double det = CvUtils::M3Det(m3H);
  if (std::fabs(det) < 10e-8) {
	  // Make a note that low rank system was obtained...
	  std::cout << "Low rank materials in makeSubPixTemplate!!!!" << endl; // difficult to occur. It contradicts feature selection...

	  std::cout << "JtJ : " << m3H << std::endl;
	  std::cout << " The computed determinant : " << det << std::endl;
	  // invert the Jacobian gram matrix using the SVD
	  cv::Vec3d w;
	  cv::Matx<double, 3, 3> U;
	  cv::Matx<double, 3, 3> Vt;
	  cv::Matx<double, 3, 3> S;
	  cv::SVD::compute(m3H, w, U, Vt);
	  for (int i = 0; i < 3; i++) {
		  S(i, 0) = S(i, 1) = S(i, 2) = 0;
		  if (w[i] != 0) S(i, i) = 1 / w[i];
	  }

	  mm3HInv = Vt.t() * S * U.t(); // pseudo-inverse
  }
  else {
	  mm3HInv = CvUtils::M3Inverse(m3H); // TODO: Make sure there's no mistake in this!!!!
											 //cv::invert(m3H, mm3HInv, cv::DECOMP_CHOLESKY);
  }

  mv2SubPixPos = mv2CoarsePos; // Start the sub-pixel search at the result of the coarse search..
  mdMeanDiff = 0.0;
}

// Iterate inverse composition until convergence. Since it should never have 
// to travel more than a pixel's distance, set a max number of iterations; 
// if this is exceeded, consider the IC to have failed.
bool PatchFinder::IterateSubPixToConvergence(KeyFrame &kf, int nMaxIts)
{
	const double dConvLimit = 0.03;
	bool bConverged = false;
	int nIts;
	for (nIts = 0; nIts < nMaxIts && !bConverged; nIts++)
	{
		double dUpdateSquared = IterateSubPix(kf);
		if (dUpdateSquared < 0) // went off edge of image
			return false;
		if (dUpdateSquared < dConvLimit*dConvLimit)
			return true;
	}
	return false;
}

// Single iteration of inverse composition. This compares integral image positions in the 
// template image to floating point positions in the target keyframe. Interpolation is
// bilinear, and performed manually (rather than using CVD::image_interpolate) since 
// this is a special case where the mixing fractions for each pixel are identical.
double PatchFinder::IterateSubPix(KeyFrame &kf)
{
	// Search level pos of patch center
	cv::Vec2d v2Center = LevelNPos(mv2SubPixPos, mnSearchLevel);
	cv::Mat_<uchar> im = kf.aLevels[mnSearchLevel].im;
	if (!CvUtils::in_image_with_border(std::round(v2Center[0]), std::round(v2Center[1]),
		im, mnPatchSize / 2 + 1, mnPatchSize / 2 + 1))
		return -1.0;       // Negative return value indicates off edge of image 

	  // Position of top-left corner of patch in search level
	cv::Vec2d v2Base(v2Center[0] - mirCenter.x,
		             v2Center[1] - mirCenter.y);

	// I.C. JT*d accumulator
	cv::Vec3d v3Accum(0, 0, 0);
	
	//byte* pTopLeftPixel;

	// Each template pixel will be compared to an interpolated target pixel
	// The target value is made using bilinear interpolation as the weighted sum
	// of four target image pixels. Calculate mixing fractions:
	double dX = v2Base[0] - floor(v2Base[0]); // Distances from pixel center of TL pixel
	double dY = v2Base[1] - floor(v2Base[1]);
	float fMixTL = (1.0 - dX) * (1.0 - dY);
	float fMixTR = (dX)       * (1.0 - dY);
	float fMixBL = (1.0 - dX) * (dY);
	float fMixBR = (dX)       * (dY);

	uchar * pImPatchRow, *pImPatchRow1, *pTempImRow;
	cv::Vec2d *pGradRow;

	for (int i = 1; i < mnPatchSize - 1; i++) {
		
		pImPatchRow = im.ptr<uchar>((int)v2Base[1] + i, (int)v2Base[0]);
		pImPatchRow1 = im.ptr<uchar>((int)v2Base[1] + i + 1, (int)v2Base[0]);
		pGradRow = mimJacs.ptr<cv::Vec2d>(i - 1);
		pTempImRow = mimTemplate.ptr<uchar>(i);

		for (int j = 1; j < mnPatchSize - 1; j++) {
			float fPixel = fMixTL * pImPatchRow[j] + fMixTR * pImPatchRow[j+1] +
						   fMixBL * pImPatchRow1[j] + fMixBR * pImPatchRow1[j+1];
			double dDiff = fPixel - pTempImRow[j] + mdMeanDiff;
			v3Accum[0] += dDiff * pGradRow[j - 1][0];
			v3Accum[1] += dDiff * pGradRow[j - 1][1];
			v3Accum[2] += dDiff;
		}
	}

	// All done looping over image - find JTJ^-1 * JTd:
	cv::Vec3d v3Update = mm3HInv * v3Accum;
	mv2SubPixPos -= cv::Vec2d(v3Update[0], v3Update[1]) * LevelScale(mnSearchLevel);
	mdMeanDiff -= v3Update[2];

	return v3Update[0] * v3Update[0] + v3Update[1] * v3Update[1]; // dPixelUpdateSquared
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
// 
//
//              ZMSSDatpoint, which is SSE optimised, follows
//
// The top version is the SSE version for 8x8 patches. It is compiled
// only if CVD_HAVE_XMMINTRIN is true, also you need to give your 
// compiler the appropriate flags (e.g. -march=core2 -msse3 for g++.)
// The standard c++ version, which is about half as quick (not a disaster
// by any means) is below.
//
// The 8x8 SSE version looks long because it has been unrolled, 
// it just does the same thing eight times. Both versions are one-pass
// and need pre-calculated template sums and sum-squares.
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

#if HAVE_XMMINTRIN
// Horizontal sum of uint16s stored in an XMM register
inline int SumXMM_16(__m128i &target)
{
  unsigned short int sums_store[8];    
  _mm_storeu_si128((__m128i*)sums_store, target);
  return sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3] +
    sums_store[4] + sums_store[5] + sums_store[6] + sums_store[7];
}
// Horizontal sum of uint32s stored in an XMM register
inline int SumXMM_32(__m128i &target)
{
  unsigned int sums_store[4];    
  _mm_storeu_si128((__m128i*)sums_store, target);
  return sums_store[0] + sums_store[1] + sums_store[2] + sums_store[3];
}
#endif

// Calculate the Zero-mean SSD of the coarse patch and a target imate at a specific 
// point.
int PatchFinder::ZMSSDAtPoint(const cv::Mat_<uchar> &im, const cv::Point &ir)
{
  if(!CvUtils::in_image_with_border(ir.y, ir.x, im,  mirCenter.x, mirCenter.y))
    return mnMaxSSD + 1;
  
  cv::Point irImgBase = ir - mirCenter;
  uchar *imagepointer;
  uchar *templatepointer;
  
  int nImageSumSq = 0;
  int nImageSum = 0;
  int nCrossSum = 0;

#if HAVE_XMMINTRIN
  if (mnPatchSize == 8)
  {
	  long unsigned int imagepointerincrement;

	  __m128i xImageAsEightBytes;
	  __m128i xImageAsWords;
	  __m128i xTemplateAsEightBytes;
	  __m128i xTemplateAsWords;
	  __m128i xZero;
	  __m128i xImageSums; // These sums are 8xuint16
	  __m128i xImageSqSums; // These sums are 4xint32
	  __m128i xCrossSums;   // These sums are 4xint32
	  __m128i xProduct;


	  xImageSums = _mm_setzero_si128();
	  xImageSqSums = _mm_setzero_si128();
	  xCrossSums = _mm_setzero_si128();
	  xZero = _mm_setzero_si128();

	  imagepointer = &im[irImgBase + ImageRef(0, 0)];
	  templatepointer = &mimTemplate[ImageRef(0, 0)];
	  imagepointerincrement = &im[irImgBase + ImageRef(0, 1)] - imagepointer;

	  xImageAsEightBytes = _mm_loadl_epi64((__m128i*) imagepointer);
	  imagepointer += imagepointerincrement;
	  xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
	  xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
	  xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
	  xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
	  xTemplateAsEightBytes = _mm_load_si128((__m128i*) templatepointer);
	  templatepointer += 16;
	  xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes, xZero);
	  xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
	  xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
	  xImageAsEightBytes = _mm_loadl_epi64((__m128i*) imagepointer);
	  imagepointer += imagepointerincrement;
	  xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
	  xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
	  xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
	  xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
	  xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes, xZero);
	  xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
	  xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

	  xImageAsEightBytes = _mm_loadl_epi64((__m128i*) imagepointer);
	  imagepointer += imagepointerincrement;
	  xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
	  xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
	  xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
	  xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
	  xTemplateAsEightBytes = _mm_load_si128((__m128i*) templatepointer);
	  templatepointer += 16;
	  xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes, xZero);
	  xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
	  xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
	  xImageAsEightBytes = _mm_loadl_epi64((__m128i*) imagepointer);
	  imagepointer += imagepointerincrement;
	  xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
	  xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
	  xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
	  xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
	  xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes, xZero);
	  xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
	  xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

	  xImageAsEightBytes = _mm_loadl_epi64((__m128i*) imagepointer);
	  imagepointer += imagepointerincrement;
	  xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
	  xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
	  xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
	  xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
	  xTemplateAsEightBytes = _mm_load_si128((__m128i*) templatepointer);
	  templatepointer += 16;
	  xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes, xZero);
	  xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
	  xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
	  xImageAsEightBytes = _mm_loadl_epi64((__m128i*) imagepointer);
	  imagepointer += imagepointerincrement;
	  xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
	  xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
	  xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
	  xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
	  xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes, xZero);
	  xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
	  xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

	  xImageAsEightBytes = _mm_loadl_epi64((__m128i*) imagepointer);
	  imagepointer += imagepointerincrement;
	  xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
	  xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
	  xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
	  xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
	  xTemplateAsEightBytes = _mm_load_si128((__m128i*) templatepointer);
	  templatepointer += 16;
	  xTemplateAsWords = _mm_unpacklo_epi8(xTemplateAsEightBytes, xZero);
	  xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
	  xCrossSums = _mm_add_epi32(xProduct, xCrossSums);
	  xImageAsEightBytes = _mm_loadl_epi64((__m128i*) imagepointer);
	  xImageAsWords = _mm_unpacklo_epi8(xImageAsEightBytes, xZero);
	  xImageSums = _mm_adds_epu16(xImageAsWords, xImageSums);
	  xProduct = _mm_madd_epi16(xImageAsWords, xImageAsWords);
	  xImageSqSums = _mm_add_epi32(xProduct, xImageSqSums);
	  xTemplateAsWords = _mm_unpackhi_epi8(xTemplateAsEightBytes, xZero);
	  xProduct = _mm_madd_epi16(xImageAsWords, xTemplateAsWords);
	  xCrossSums = _mm_add_epi32(xProduct, xCrossSums);

	  nImageSum = SumXMM_16(xImageSums);
	  nCrossSum = SumXMM_32(xCrossSums);
	  nImageSumSq = SumXMM_32(xImageSqSums);
  }
  else
#endif 
  {
	  for (int nRow = 0; nRow < mnPatchSize; nRow++)
	  {
		  imagepointer = im.data + (irImgBase.y + nRow) * im.step + irImgBase.x;
		  templatepointer = mimTemplate.data + nRow * mimTemplate.step;

		  for (int nCol = 0; nCol < mnPatchSize; nCol++)
		  {
			  int n = imagepointer[nCol];
			  nImageSum += n;
			  nImageSumSq += n*n;
			  nCrossSum += n * templatepointer[nCol];
		  }
	  }
  }
  
  int SA = mnTemplateSum;
  int SB = nImageSum;
  
  int N = mnPatchSize * mnPatchSize;
  return ((2*SA*SB - SA*SA - SB*SB)/N + nImageSumSq + mnTemplateSumSq - 2*nCrossSum);
}
