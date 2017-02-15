#include "OpenGL.h"
#include "CalibImage.h"
#include <stdlib.h>
#include <cmath>
#include "GCVD/GLHelpers.h"
#include "FAST/fast_corner.h"

#include "Persistence/instances.h"

using namespace std;
using namespace FAST;
using namespace RigidTransforms;
using namespace GLXInterface;
using namespace CvUtils;

inline bool isCorner(cv::Mat_<uchar> &im, cv::Point ir, int nGate)
{
	int nSum = 0;
	static int abPixels[16];
	for (int i = 0; i < 16; i++)
	{
		abPixels[i] = im.ptr<uchar>(ir.y + FAST::fast_pixel_ring[i].y)[ir.x + FAST::fast_pixel_ring[i].x];
		nSum += abPixels[i];
	}

	int nMean = nSum / 16;
	int nHiThresh = nMean + nGate;
	int nLoThresh = nMean - nGate;

	int nCenter = im.at<uchar>(ir);
	if (nCenter <= nLoThresh || nCenter >= nHiThresh)
		return false;

	bool bState = (abPixels[15] > nMean);
	int nSwaps = 0;

	for (int i = 0; i<16; i++)
	{
		uchar bValNow = abPixels[i];
		if (bState)
		{
			if (bValNow < nLoThresh)
			{
				bState = false;
				nSwaps++;
			}
		}
		else
			if (bValNow > nHiThresh)
			{
				bState = true;
				nSwaps++;
			};
	}
	return (nSwaps == 4);
}

cv::Vec2d GuessInitialAngles(cv::Mat_<uchar> &im, cv::Point irCenter)
{
	double dBestAngle = 0;
	double dBestGradMag = 0;
	double dGradAtBest = 0;
	for (double dAngle = 0.0; dAngle < CV_PI; dAngle += 0.1)
	{
		cv::Vec2d v2Dirn;
		v2Dirn[0] = cos(dAngle);      v2Dirn[1] = sin(dAngle);
		cv::Vec2d v2Perp;
		v2Perp[1] = -v2Dirn[0];      v2Perp[0] = v2Dirn[1];
		cv::Vec2d vec_irCenter(irCenter.x, irCenter.y);
		double dG = getSubpix(im, vec_irCenter + v2Dirn * 3.0 + v2Perp * 0.1) -
			getSubpix(im, vec_irCenter + v2Dirn * 3.0 - v2Perp * 0.1)
			+ getSubpix(im, vec_irCenter - v2Dirn * 3.0 - v2Perp * 0.1) -
			getSubpix(im, vec_irCenter - v2Dirn * 3.0 + v2Perp * 0.1);

		if (fabs(dG) > dBestGradMag)
		{
			dBestGradMag = fabs(dG);
			dGradAtBest = dG;
			dBestAngle = dAngle;
		};
	}

	cv::Vec2d v2Ret;
	if (dGradAtBest < 0)
	{
		v2Ret[0] = dBestAngle; v2Ret[1] = dBestAngle + CV_PI / 2.0;
	}
	else
	{
		v2Ret[1] = dBestAngle; v2Ret[0] = dBestAngle - CV_PI / 2.0;
	}
	return v2Ret;
}
 
bool CalibImage::MakeFromImage(cv::Mat_<uchar> &im, cv::Mat &cim)
{
	static Persistence::pvar3<int> gvnCornerPatchSize("CameraCalibrator.CornerPatchPixelSize", 20, Persistence::SILENT);
	mvCorners.clear();
	mvGridCorners.clear();

	im.copyTo(mim);
	rgbmim = cim;

	{
		cv::Mat_<uchar> imBlurred = mim.clone();
		//convolveGaussian(imBlurred, GV2.GetDouble("CameraCalibrator.BlurSigma", 1.0, Persistence::SILENT));
		int ksize = (int)ceil(Persistence::PV3::get<double>("CameraCalibrator.BlurSigma", 1.0, Persistence::SILENT) * 3.0);
		ksize += ksize % 2 == 0 ? 1 : 0;
		cv::GaussianBlur(imBlurred, imBlurred, cv::Size(ksize, ksize), Persistence::PV3::get<double>("CameraCalibrator.BlurSigma", 1.0, Persistence::SILENT), 3.0);

		cv::Point irTopLeft(5, 5);
		cv::Point irBotRight = cv::Point(mim.size()) - irTopLeft;
		cv::Point ir = irTopLeft;
		glPointSize(1);
		glColor3f(1, 0, 1);
		glBegin(GL_POINTS);
		int nGate = Persistence::PV3::get<int>("CameraCalibrator.MeanGate", 10, Persistence::SILENT);

		for (int i = irTopLeft.y; i < irBotRight.y; i++) {
			for (int j = irTopLeft.x; j < irBotRight.x; j++) {
				if (isCorner(imBlurred, cv::Point(j, i), nGate))
				{
					mvCorners.push_back(cv::Point(j, i));
					glVertex(cv::Point(j, i));
				}
			}
		}
		glEnd();
	}

	if ((int)mvCorners.size() < Persistence::PV3::get<int>("CameraCalibrator.MinCornersForGrabbedImage", 20, Persistence::SILENT))
		return false;

	// Pick a central corner point...
	
	cv::Point irCenterOfImage(mim.size() / 2);
	cv::Point irBestCenterPos;
	unsigned int nBestDistSquared = 99999999;
	for (unsigned int i = 0; i < mvCorners.size(); i++)
	{
		unsigned int nDist = mag_squared(mvCorners[i] - irCenterOfImage);
		if (nDist < nBestDistSquared)
		{
			nBestDistSquared = nDist;
			irBestCenterPos = mvCorners[i];
		}
	}

	// ... and try to fit a corner-patch to that.
	CalibCornerPatch Patch(*gvnCornerPatchSize);
	CalibCornerPatch::Params Params;
	Params.v2Pos = cv::Vec2d(irBestCenterPos.x, irBestCenterPos.y);
	Params.v2Angles = GuessInitialAngles(mim, irBestCenterPos);
	Params.dGain = 80.0;
	Params.dMean = 120.0;

	if (!Patch.IterateOnImageWithDrawing(Params, mim))
		return false;

	// The first found corner patch becomes the origin of the detected grid.
	CalibGridCorner cFirst;
	cFirst.Params = Params;
	mvGridCorners.push_back(cFirst);
	cFirst.Draw();

	// Next, go in two compass directions from the origin patch, and see if 
	// neighbors can be found.
	if (!(ExpandByAngle(0, 0) || ExpandByAngle(0, 2)))
		return false;
	if (!(ExpandByAngle(0, 1) || ExpandByAngle(0, 3)))
		return false;

	mvGridCorners[1].mInheritedSteps = mvGridCorners[2].mInheritedSteps = mvGridCorners[0].GetSteps(mvGridCorners);

	// The three initial grid elements are enough to find the rest of the grid.
	int nNext;
	int nSanityCounter = 0; // Stop it getting stuck in an infinite loop...
	const int nSanityCounterLimit = 500;
	while ((nNext = NextToExpand()) >= 0 && nSanityCounter < nSanityCounterLimit)
	{
		ExpandByStep(nNext);
		nSanityCounter++;
	}
	if (nSanityCounter == nSanityCounterLimit)
		return false;

	DrawImageGrid();
	return true;

}

bool CalibImage::ExpandByAngle(int nSrc, int nDirn)
{
	static Persistence::pvar3<int> gvnCornerPatchSize("CameraCalibrator.CornerPatchPixelSize", 20, Persistence::SILENT);
	CalibGridCorner &gSrc = mvGridCorners[nSrc];

	cv::Point irBest;
	double dBestDist = 99999;
	cv::Vec2d v2TargetDirn(gSrc.Params.m2Warp()(0, nDirn % 2), gSrc.Params.m2Warp()(1, nDirn % 2));

	if (nDirn >= 2)
		v2TargetDirn *= -1;
	for (unsigned int i = 0; i < mvCorners.size(); i++)
	{
		cv::Vec2d v2Diff = cv::Vec2d(mvCorners[i].x, mvCorners[i].y) - gSrc.Params.v2Pos;
		if (v2Diff[0] * v2Diff[0] + v2Diff[1] * v2Diff[1] < 100)
			continue;
		if (v2Diff[0] * v2Diff[0] + v2Diff[1] * v2Diff[1] > dBestDist * dBestDist)
			continue;
		cv::Vec2d v2Dirn = cv::normalize(v2Diff);
		if (v2Dirn[0] * v2TargetDirn[0] + v2Dirn[1] * v2TargetDirn[1] < cos(CV_PI / 18.0))
			continue;
		dBestDist = cv::norm(v2Diff);
		irBest = mvCorners[i];
	}

	CalibGridCorner gTarget;
	gTarget.Params = gSrc.Params;
	gTarget.Params.v2Pos = cv::Vec2d(irBest.x, irBest.y);
	gTarget.Params.dGain *= -1;

	CalibCornerPatch Patch(*gvnCornerPatchSize);
	if (!Patch.IterateOnImageWithDrawing(gTarget.Params, mim))
	{
		gSrc.aNeighborStates[nDirn].val = N_FAILED;
		return false;
	}

	//gTarget.irGridPos = gSrc.irGridPos;
	int tmpIrGridPos[2] = { gSrc.irGridPos.x, gSrc.irGridPos.y };
	if (nDirn < 2)
		//gTarget.irGridPos[nDirn]++;
		tmpIrGridPos[nDirn]++;
	//else gTarget.irGridPos[nDirn % 2]--;
	else tmpIrGridPos[nDirn % 2] --;
	gTarget.irGridPos = cv::Point(tmpIrGridPos[0], tmpIrGridPos[1]);
	// Update connection states:
	mvGridCorners.push_back(gTarget); // n.b. This invalidates gSrc!
	mvGridCorners.back().aNeighborStates[(nDirn + 2) % 4].val = nSrc;
	mvGridCorners[nSrc].aNeighborStates[nDirn].val = mvGridCorners.size() - 1;

	mvGridCorners.back().Draw();
	return true;
}

void CalibGridCorner::Draw()
{
	glColor3f(0, 1, 0);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glBegin(GL_LINES);

	// right vertex
	cv::Vec2d vertex1(Params.v2Pos[0] + Params.m2Warp()(0, 0) * 10 + Params.m2Warp()(0, 1) * 0.0,
		Params.v2Pos[1] + Params.m2Warp()(1, 0) * 10 + Params.m2Warp()(1, 1) * 0.0);
	// left vertex
	cv::Vec2d vertex2(Params.v2Pos[0] + Params.m2Warp()(0, 0) * (-10) + Params.m2Warp()(0, 1) * 0.0,
		Params.v2Pos[1] + Params.m2Warp()(1, 0) * (-10) + Params.m2Warp()(1, 1) * 0.0);
	// upper vertex
	cv::Vec2d vertex3(Params.v2Pos[0] + Params.m2Warp()(0, 0) * 0.0 + Params.m2Warp()(0, 1) * 10,
		Params.v2Pos[1] + Params.m2Warp()(1, 0) * 0.0 + Params.m2Warp()(1, 1) * 10);
	// lower vertex
	cv::Vec2d vertex4(Params.v2Pos[0] + Params.m2Warp()(0, 0) * 0.0 + Params.m2Warp()(0, 1) * (-10),
		Params.v2Pos[1] + Params.m2Warp()(1, 0) * 0.0 + Params.m2Warp()(1, 1) * (-10));

	// 'horizontal' line 
	glVertex2d(vertex1[0], vertex1[1]);
	glVertex2d(vertex2[0], vertex2[1]);
	// 'vertical' line
	glVertex2d(vertex3[0], vertex3[1]);
	glVertex2d(vertex4[0], vertex4[1]);
	glEnd();
};

double CalibGridCorner::ExpansionPotential()
{
  // Scoring function. How good would this grid corner be at finding a neighbor?
  // The best case is if it's already surrounded by three neighbors and only needs
  // to find the last one (because it'll have the most accurate guess for where
  // the last one should be) and so on.
  int nMissing = 0;
  for(int i=0; i<4; i++)
    if(aNeighborStates[i].val == N_NOT_TRIED)
      nMissing++;

  if(nMissing == 0)
    return 0.0;
  
  if(nMissing == 1)
    return 100.0;
  
  if(nMissing == 3)
    return 1.0;

  if(nMissing == 2)
    {
      int nFirst = 0;
      while(aNeighborStates[nFirst].val != N_NOT_TRIED)
	nFirst++;
      if(aNeighborStates[(nFirst + 2) % 4].val == N_NOT_TRIED)
	return 10.0;
      else
	return 20.0;
    }
  assert(0); // should never get here
  return 0.0;
};


cv::Matx<double, 2, 2> CalibGridCorner::GetSteps(vector<CalibGridCorner> &vgc)
{
  cv::Matx<double, 2, 2> m2Steps;
  for(int dirn=0; dirn<2; dirn++)
    {
      cv::Vec2d v2Dirn(0, 0);
      int nFound = 0;
      if(aNeighborStates[dirn].val >=0)
	{
	  v2Dirn += vgc[aNeighborStates[dirn].val].Params.v2Pos - Params.v2Pos;
	  nFound++;
	}
      if(aNeighborStates[dirn+2].val >=0)
	{
	  v2Dirn -= vgc[aNeighborStates[dirn+2].val].Params.v2Pos - Params.v2Pos;
	  nFound++;
	}
	  if (nFound == 0) {
		  m2Steps(dirn, 0) = mInheritedSteps(dirn, 0);
		  m2Steps(dirn, 1) = mInheritedSteps(dirn, 1);
	  }
	  else {
		  m2Steps(dirn, 0) = v2Dirn[0] / nFound;
		  m2Steps(dirn, 1) = v2Dirn[1] / nFound;
	  }
    }
  return m2Steps;
}

int CalibImage::NextToExpand()
{
  int nBest = -1;
  double dBest = 0.0;
  
  for(unsigned int i=0; i<mvGridCorners.size(); i++)
    {
      double d = mvGridCorners[i].ExpansionPotential();
      if(d > dBest)
	{
	  nBest = i;
	  dBest = d;
	}
    }
  return nBest;
}

void CalibImage::ExpandByStep(int n)
{
  static Persistence::pvar3<double> gvdMaxStepDistFraction("CameraCalibrator.ExpandByStepMaxDistFrac", 0.3, Persistence::SILENT);
  static Persistence::pvar3<int> gvnCornerPatchSize("CameraCalibrator.CornerPatchPixelSize", 20, Persistence::SILENT);
  
  CalibGridCorner &gSrc = mvGridCorners[n];
  
  // First, choose which direction to expand in...
  // Ideally, choose a dirn for which the Step calc is good!
  int nDirn = -10;
  for(int i=0; nDirn == -10 && i<4; i++)
    {
      if(gSrc.aNeighborStates[i].val == N_NOT_TRIED &&
	 gSrc.aNeighborStates[(i+2) % 4].val >= 0)
	nDirn = i;
    }
  if(nDirn == -10)
  for(int i=0; nDirn == -10 && i<4; i++)
    {
      if(gSrc.aNeighborStates[i].val == N_NOT_TRIED)
	nDirn = i;
    }
  assert(nDirn != -10);
  
  cv::Point irGridStep = IR_from_dirn(nDirn);
  cv::Matx<double, 2, 2> M = gSrc.GetSteps(mvGridCorners);

  cv::Vec2d v2Step(M(0, 0) * irGridStep.x + M(1, 0) * irGridStep.y,
	  M(0, 1) * irGridStep.x + M(1, 1) * irGridStep.y);

  cv::Vec2d v2SearchPos = gSrc.Params.v2Pos + v2Step;
  
  // Before the search: pre-fill the failure result for easy returns.
  gSrc.aNeighborStates[nDirn].val = N_FAILED;
  
  //ImageRef irBest;
  cv::Point irBest;
  double dBestDist = 99999;
  for(unsigned int i=0; i<mvCorners.size(); i++)
    {
      cv::Vec2d v2Diff = cv::Vec2d(mvCorners[i].x, mvCorners[i].y) - v2SearchPos;
	  double v2DiffNorm = cv::norm(v2Diff);
	  if (v2DiffNorm > dBestDist)
		  continue;
	  dBestDist = v2DiffNorm;
      irBest = mvCorners[i];
    }
  
  double dStepDist= cv::norm(v2Step);
  if (dBestDist > *gvdMaxStepDistFraction * dStepDist)
	  return;
  
  CalibGridCorner gTarget;
  gTarget.Params = gSrc.Params;
  gTarget.Params.v2Pos = cv::Vec2d(irBest.x, irBest.y);
  gTarget.Params.dGain *= -1;
  gTarget.irGridPos = gSrc.irGridPos + irGridStep;
  gTarget.mInheritedSteps = gSrc.GetSteps(mvGridCorners);
  CalibCornerPatch Patch(*gvnCornerPatchSize);
  if(!Patch.IterateOnImageWithDrawing(gTarget.Params, mim))
    return;
  
  // Update connection states:
  int nTargetNum = mvGridCorners.size();
  for(int dirn = 0; dirn<4; dirn++)
    {
      //ImageRef irSearch = gTarget.irGridPos + IR_from_dirn(dirn);
	  cv::Point irSearch = gTarget.irGridPos + IR_from_dirn(dirn);

      for (unsigned int i=0; i < mvGridCorners.size(); i++)
	if(mvGridCorners[i].irGridPos == irSearch)
	  {
	    gTarget.aNeighborStates[dirn].val = i;
	    mvGridCorners[i].aNeighborStates[(dirn + 2) % 4].val = nTargetNum;
	  }
    }
  mvGridCorners.push_back(gTarget);
  mvGridCorners.back().Draw();
}

void CalibImage::DrawImageGrid()
{
  glLineWidth(2);
  glColor3f(0,0,1);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBegin(GL_LINES);
  
  for(int i=0; i< (int) mvGridCorners.size(); i++)
    {
      for(int dirn=0; dirn<4; dirn++)
	if(mvGridCorners[i].aNeighborStates[dirn].val > i)
	  {
	    glVertex2d(mvGridCorners[i].Params.v2Pos[0], mvGridCorners[i].Params.v2Pos[1]);
	    glVertex2d(mvGridCorners[mvGridCorners[i].aNeighborStates[dirn].val].Params.v2Pos[0],
			       mvGridCorners[mvGridCorners[i].aNeighborStates[dirn].val].Params.v2Pos[1]);
	  }
    }
  glEnd();
  
  glPointSize(5);
  glEnable(GL_POINT_SMOOTH);
  glColor3f(1,1,0);
  glBegin(GL_POINTS);
  for(unsigned int i=0; i<mvGridCorners.size(); i++)
    glVertex2d(mvGridCorners[i].Params.v2Pos[0],mvGridCorners[i].Params.v2Pos[1]);
  glEnd();
};

void CalibImage::Draw3DGrid(ATANCamera &Camera, bool bDrawErrors)
{
  glLineWidth(2);
  glColor3f(0,0,1);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBegin(GL_LINES);
  
  for(int i=0; i< (int) mvGridCorners.size(); i++)
    {
      for(int dirn=0; dirn<4; dirn++)
	if(mvGridCorners[i].aNeighborStates[dirn].val > i)
	  {
	    cv::Vec3d v3(mvGridCorners[i].irGridPos.x,
			      mvGridCorners[i].irGridPos.y,
			      0.0);
		cv::Vec3d cvec = mse3CamFromWorld * v3;
		cv::Vec2d cvec_proj = cv::Vec2d(cvec[0] / cvec[2], cvec[1] / cvec[2]);;

		cv::Vec2d m = Camera.Project(cvec_proj);

	    glVertex2d(m[0], m[1]);
	    v3[0] = mvGridCorners[mvGridCorners[i].aNeighborStates[dirn].val].irGridPos.x;
		v3[1] = mvGridCorners[mvGridCorners[i].aNeighborStates[dirn].val].irGridPos.y;

		cvec = mse3CamFromWorld * v3; 
		cvec_proj = cv::Vec2d(cvec[0] / cvec[2], cvec[1] / cvec[2]);
		m = Camera.Project(cvec_proj);
		glVertex2d(m[0], m[1]);
	  }
    }
  glEnd();

  if (bDrawErrors)
  {
	  glColor3f(1, 0, 0);
	  glLineWidth(1);
	  glBegin(GL_LINES);
	  for (int i = 0; i < (int)mvGridCorners.size(); i++)
	  {
		  cv::Vec3d v3(mvGridCorners[i].irGridPos.x,
			  mvGridCorners[i].irGridPos.y,
			  0.0);
		  cv::Vec3d cvec = mse3CamFromWorld * v3;
		  cv::Vec2d cvec_proj(cvec[0] / cvec[2], cvec[1] / cvec[2]);
		  cv::Vec2d m = Camera.Project(cvec_proj);

		  cv::Vec2d v2pixBackProjection = Camera.Project(m);
		  cv::Vec2d v2Error = mvGridCorners[i].Params.v2Pos - v2pixBackProjection;

		  glVertex2d(v2pixBackProjection[0], v2pixBackProjection[1]);
		  glVertex2d(v2pixBackProjection[0] + 10.0 * v2Error[0], v2pixBackProjection[1] + 10.0 * v2Error[1]);
	  }
	  glEnd();
  }
};

cv::Point CalibImage::IR_from_dirn(int nDirn)
{
	int ir[2] = { 0,0 };
	ir[nDirn % 2] = (nDirn < 2) ? 1 : -1;
	return cv::Point(ir[0], ir[1]);
}

void CalibImage::GuessInitialPose(ATANCamera &Camera)
{

	// number of registered grid points
	int nPoints = mvGridCorners.size();
	// Doing the 9x9 gram-matrix accumulator instead of the data matrix. Its better.
	cv::Matx<double, 9, 9> m9D = cv::Matx<double, 9, 9>::zeros();
	for (int n = 0; n<nPoints; n++) {
		// First, beck-project the image locations of the recovered grid corners onto the normalized Euclidean plane (z = 1)
		cv::Vec2d v2UnProj = Camera.UnProject(mvGridCorners[n].Params.v2Pos);
		double x2 = v2UnProj[0];
		double y2 = v2UnProj[1];
		// So, now u and v are 2D Euclidean coordinates on the projection ray for z = 1.

		// Then fill in the matrix..
		double x1 = mvGridCorners[n].irGridPos.x; // corner x-location in the grid! (assuming unit length in the grid!)
		double y1 = mvGridCorners[n].irGridPos.y; // corner y-locatin in the grid!  

												  // filling ONLY the Upper triangle of m9D...
												  //
												  // 1. Filling the upper triangle of the upper 3x3 block (which is equal to  the second 3x3 diagonal block)
		m9D(0, 0) += x1*x1;  m9D(0, 1) += x1*y1; m9D(0, 2) += x1;
		m9D(1, 1) += y1*y1; m9D(1, 2) += y1;
		m9D(2, 2) += 1.0;

		// 2. Now filling the 3 columns from 7 to 8 down-to and including the diagonal:
		m9D(0, 6) += -x1*x1*x2;             m9D(0, 7) += -x1*x2*y1;              m9D(0, 8) += -x1*x2;
		m9D(1, 6) += -x1*x2*y1;             m9D(1, 7) += -x2*y1*y1;              m9D(1, 8) += -x2*y1;
		m9D(2, 6) += -x1*x2;                m9D(2, 7) += -x2*y1;                 m9D(2, 8) += -x2;
		m9D(3, 6) += -x1*x1*y2;             m9D(3, 7) += -x1*y1*y2;              m9D(3, 8) += -x1*y2;
		m9D(4, 6) += -x1*y1*y2;             m9D(4, 7) += -y1*y1*y2;              m9D(4, 8) += -y1*y2;
		m9D(5, 6) += -x1*y2;                m9D(5, 7) += -y1*y2;                 m9D(5, 8) += -y2;
		m9D(6, 6) += x1*x1*(x2*x2 + y2*y2); m9D(6, 7) += x1*y1*(x2*x2 + y2*y2);  m9D(6, 8) += x1*(x2*x2 + y2*y2);
		m9D(7, 7) += y1*y1*(x2*x2 + y2*y2);  m9D(7, 8) += y1*(x2*x2 + y2*y2);
		m9D(8, 8) += x2*x2 + y2*y2;

	}
	// Sow no filling-in the gaps (left-out due to symmetry):
	// 1. Filling the missing lower partof the upper 3x3 diagonal block and copying tho the second diagonal 3x3 block
	m9D(1, 0) = m9D(4, 3) = m9D(3, 4) = m9D(0, 1);
	m9D(2, 0) = m9D(5, 3) = m9D(3, 5) = m9D(0, 2);   m9D(2, 1) = m9D(5, 4) = m9D(4, 5) = m9D(1, 2);
	// and the diagonalelements from 3 - 5 are the same ones from 0-2:
	m9D(3, 3) = m9D(0, 0); m9D(4, 4) = m9D(1, 1); m9D(5, 5) = m9D(2, 2);

	// 2. Now copying the last 3 columns (down-to and exluding the diagonal) to the last 3 rows...
	m9D(6, 0) = m9D(0, 6); m9D(6, 1) = m9D(1, 6); m9D(6, 2) = m9D(2, 6); m9D(6, 3) = m9D(3, 6); m9D(6, 4) = m9D(4, 6); m9D(6, 5) = m9D(5, 6);
	m9D(7, 0) = m9D(0, 7); m9D(7, 1) = m9D(1, 7); m9D(7, 2) = m9D(2, 7); m9D(7, 3) = m9D(3, 7); m9D(7, 4) = m9D(4, 7); m9D(7, 5) = m9D(5, 7); m9D(7, 6) = m9D(6, 7);
	m9D(8, 0) = m9D(0, 8); m9D(8, 1) = m9D(1, 8); m9D(8, 2) = m9D(2, 8); m9D(8, 3) = m9D(3, 8); m9D(8, 4) = m9D(4, 8); m9D(8, 5) = m9D(5, 8); m9D(8, 6) = m9D(6, 8); m9D(8, 7) = m9D(7, 8);



	// In any case (null-space or smallest singular value), we need the last row of V^t
	// The right null-space or th last eigenvector of m3D9 gives the homography...
	cv::Matx<double, 9, 9> U, Vt;
	cv::Matx<double, 9, 1> w;
	cv::SVD::compute(m9D, w, U, Vt);

	cv::Matx<double, 3, 3> m3Homography;
	m3Homography(0, 0) = Vt(8, 0); m3Homography(0, 1) = Vt(8, 1); m3Homography(0, 2) = Vt(8, 2);
	m3Homography(1, 0) = Vt(8, 3); m3Homography(1, 1) = Vt(8, 4); m3Homography(1, 2) = Vt(8, 5);
	m3Homography(2, 0) = Vt(8, 6); m3Homography(2, 1) = Vt(8, 7); m3Homography(2, 2) = Vt(8, 8);

	//cout <<"Difference between homographies "<<m3H - cv::Mat(m3Homography)<<endl;

	// Fix up possibly poorly conditioned bits of the homography
	// This appears to be essentially the scaling-down of the homography by the largest singular values of its upper- left 2x2 block
	{
		cv::Matx<double, 2, 2> Htl = m3Homography.get_minor<2, 2>(0, 0);

		cv::Matx<double, 2, 1> v2Diagonal;
		cv::Matx<double, 2, 2> v2Vt;
		cv::Matx<double, 2, 2> v2U;

		cv::SVD::compute(Htl, v2Diagonal, v2U, v2Vt);
		double smax = v2Diagonal(0, 0);


		// scaling down the entire homography by the largest singular value of H11
		m3Homography = (1 / smax) * m3Homography;
		// scaling down the singular values as well...
		v2Diagonal = (1 / smax) * v2Diagonal;
		// store second largest singular value in dLambda2
		double dLambda2 = v2Diagonal(1, 0);

		// *********** I am keeping old PTAM instructions in comments for double-checking in order to be on the safe side. 
		// *********** Pose extraction from homography entails ambiguities and the slighhtest mistakes can make one's life really mizerable...

		// v2b is one hypothesis for v2b ; the other is the negative.
		cv::Vec2d v2b(0.0,
			sqrt(1.0 - (dLambda2 * dLambda2)));

		//Vector<2> v2aprime = v2b * svdTopLeftBit.get_VT();
		cv::Vec2d v2aprime(v2b[0] * Vt(0, 0) + v2b[1] * Vt(1, 0),
			v2b[0] * Vt(0, 1) + v2b[1] * Vt(1, 1));


		//Vector<2> v2a = m3Homography[2].slice<0,2>();

		// double dDotProd = v2a * v2aprime;
		double dDotProd = m3Homography(2, 0) * v2aprime[0] + m3Homography(2, 1) * v2aprime[1];

		if (dDotProd>0) {
			//m3Homography[2].slice<0,2>() = v2aprime;
			m3Homography(2, 0) = v2aprime[0]; m3Homography(2, 1) = v2aprime[1];
		}
		else {
			//m3Homography[2].slice<0,2>() = -v2aprime;
			m3Homography(2, 0) = -v2aprime[0]; m3Homography(2, 1) = -v2aprime[1];
		}

	}


	// OK, now turn homography into something 3D ...simple gram-schmidt ortho-norm
	// Take 3x3 matrix H with column: abt
	// And add a new 3rd column: abct
	cv::Matx<float, 3, 3> mRotation(3, 3);
	cv::Vec3d vTranslation;
	//double dMag1 = sqrt(m3Homography.T()[0] * m3Homography.T()[0]);
	double dMag1 = sqrt(m3Homography(0, 0) * m3Homography(0, 0) +
		m3Homography(1, 0) * m3Homography(1, 0) +
		m3Homography(2, 0) * m3Homography(2, 0));
	// 1. Scale the entire homography (it could have been just the first column)
	m3Homography = (1 / dMag1) * m3Homography;

	// Store the first column of the homography in the 1st column of the rotation
	//mRotation.T()[0] = m3Homography.T()[0];
	mRotation(0, 0) = m3Homography(0, 0);
	mRotation(1, 0) = m3Homography(1, 0);
	mRotation(2, 0) = m3Homography(2, 0);

	// 2. Now subtract the projection of the second column onto the first from the second. Store result in the second column of the rotation matrix
	double dot12 = m3Homography(0, 0) * m3Homography(0, 1) + m3Homography(1, 0) * m3Homography(1, 1) + m3Homography(2, 0) * m3Homography(2, 1);
	mRotation(0, 1) = m3Homography(0, 1) - dot12 * m3Homography(0, 0);
	mRotation(1, 1) = m3Homography(1, 1) - dot12 * m3Homography(1, 0);
	mRotation(2, 1) = m3Homography(2, 1) - dot12 * m3Homography(2, 0);

	// 3. Normalize the second column of the rotation matrix...
	double norm2 = sqrt(mRotation(0, 1) * mRotation(0, 1) + mRotation(1, 1) * mRotation(1, 1) + mRotation(2, 1) * mRotation(2, 1));
	mRotation(0, 1) /= norm2; mRotation(1, 1) /= norm2; mRotation(2, 1) /= norm2;

	// 3. Store the cross product of the first and second column of the rotation matrix in the third,
	// Although i have an overload read ("^"), I 'd rather play it safe and embed the operation below...
	mRotation(0, 2) = -mRotation(2, 0) * mRotation(1, 1) + mRotation(1, 0) * mRotation(2, 1);
	mRotation(1, 2) = mRotation(2, 0) * mRotation(0, 1) - mRotation(0, 0) * mRotation(2, 1);
	mRotation(2, 2) = -mRotation(1, 0) * mRotation(0, 1) + mRotation(0, 0) * mRotation(1, 1);

	// Obtaining the translation from the 3d column of the homography
	//vTranslation = m3Homography.T()[2];
	vTranslation[0] = m3Homography(0, 2);
	vTranslation[1] = m3Homography(1, 2);
	vTranslation[2] = m3Homography(2, 2);




	// Finally, store everything in the SE3 object the takes world points to the camera
	mse3CamFromWorld.get_rotation().get_matrix() = mRotation;
	mse3CamFromWorld.get_translation() = vTranslation;

	//cout << "recovered rotation : " << mse3CamFromWorld.get_rotation().get_matrix() << endl;
	//cout << "recovered translation : " << mse3CamFromWorld.get_translation() << endl;
};

vector<CalibImage::ErrorAndJacobians> CalibImage::Project(ATANCamera &Camera)
{
  vector<CalibImage::ErrorAndJacobians> vResult;
  for(unsigned int n=0; n<mvGridCorners.size(); n++)
    {
      ErrorAndJacobians EAJ;
      
      // First, project into image...
      cv::Vec3d v3World(mvGridCorners[n].irGridPos.x, mvGridCorners[n].irGridPos.y, 0.0);
      cv::Vec3d v3Cam = mse3CamFromWorld * v3World;
      if(v3Cam[2] <= 0.001)
	continue;
      
      cv::Vec2d v2Image = Camera.Project(CvUtils::pproject(v3Cam));
	  if (Camera.Invalid())
		  continue;
      
      EAJ.v2Error = mvGridCorners[n].Params.v2Pos - v2Image;
      
      // Now find motion jacobian..
      double dOneOverCameraZ = 1.0 / v3Cam[2];
      cv::Matx<double, 2, 2> m2CamDerivs = Camera.GetProjectionDerivs();
      
	  for (int dof = 0; dof < 6; dof++)
	  {
		  const cv::Vec4d v4Motion = RigidTransforms::SE3<>::generator_field(dof, CvUtils::backproject(v3Cam));
		  cv::Vec2d v2CamFrameMotion((v4Motion[0] - v3Cam[0] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ,
			  (v4Motion[1] - v3Cam[1] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ);
		  EAJ.m26PoseJac(0, dof) = m2CamDerivs(0, 0) * v2CamFrameMotion[0] + m2CamDerivs(0, 1) * v2CamFrameMotion[1];
		  EAJ.m26PoseJac(1, dof) = m2CamDerivs(1, 0) * v2CamFrameMotion[0] + m2CamDerivs(1, 1) * v2CamFrameMotion[1];
	  }

      // Finally, the camera provids its own jacobian
      //EAJ.m2NCameraJac = Camera.GetCameraParameterDerivs();
	  EAJ.m2NCameraJac = Camera.GetCamParamAnalyticalDerivs();
      vResult.push_back(EAJ);
    }
  return vResult;
};
