// Copyright 2008 Isis Innovation Limited
#include "OpenGL.h"
#include "CalibImage.h"
#include <stdlib.h>
#include <gvars3/instances.h>

#include <TooN/se3.h>
#include <TooN/SVD.h>
#include <TooN/wls.h>
#include "FAST/fast_corner.h"

using namespace std;
using namespace GVars3;
using namespace FAST;

inline bool isCorner(cv::Mat &im, cv::Point ir, int nGate)
{
	int nSum = 0;
	static int abPixels[16];
	for (int i = 0; i < 16; i++)
	{
		abPixels[i] = im.ptr<uchar>(ir.y + FAST::fast_pixel_ring[i].y)[ir.x + FAST::fast_pixel_ring[i].x];
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

Vector<2> GuessInitialAngles(cv::Mat &im, cv::Point irCenter)
{
	double dBestAngle = 0;
	double dBestGradMag = 0;
	double dGradAtBest = 0;
	for (double dAngle = 0.0; dAngle < M_PI; dAngle += 0.1)
	{
		Vector<2> v2Dirn;
		v2Dirn[0] = cos(dAngle);      v2Dirn[1] = sin(dAngle);
		Vector<2> v2Perp;
		v2Perp[1] = -v2Dirn[0];      v2Perp[0] = v2Dirn[1];

		double dG = getSubpix(im, size2Vec(irCenter) + v2Dirn * 3.0 + v2Perp * 0.1) -
			getSubpix(im, size2Vec(irCenter) + v2Dirn * 3.0 - v2Perp * 0.1)
			+ getSubpix(im, size2Vec(irCenter) - v2Dirn * 3.0 - v2Perp * 0.1) -
			getSubpix(im, size2Vec(irCenter) - v2Dirn * 3.0 + v2Perp * 0.1);

		if (fabs(dG) > dBestGradMag)
		{
			dBestGradMag = fabs(dG);
			dGradAtBest = dG;
			dBestAngle = dAngle;
		};
	}

	Vector<2> v2Ret;
	if (dGradAtBest < 0)
	{
		v2Ret[0] = dBestAngle; v2Ret[1] = dBestAngle + M_PI / 2.0;
	}
	else
	{
		v2Ret[1] = dBestAngle; v2Ret[0] = dBestAngle - M_PI / 2.0;
	}
	return v2Ret;
}

bool CalibImage::MakeFromImage(cv::Mat &im)
{
	static gvar3<int> gvnCornerPatchSize("CameraCalibrator.CornerPatchPixelSize", 20, SILENT);
	mvCorners.clear();
	mvGridCorners.clear();

	cv::Mat mim = im.clone();

	{
		cv::Mat imBlurred = mim.clone();
		//convolveGaussian(imBlurred, GV2.GetDouble("CameraCalibrator.BlurSigma", 1.0, SILENT));
		int ksize = (int)ceil(GV2.GetDouble("CameraCalibrator.BlurSigma", 1.0, SILENT) * 3.0);
		cv::GaussianBlur(imBlurred, imBlurred, cv::Size(ksize, ksize), GV2.GetDouble("CameraCalibrator.BlurSigma", 1.0, SILENT), 3.0);

		cv::Point irTopLeft(5, 5);
		cv::Point irBotRight = cv::Point(mim.size()) - irTopLeft;
		cv::Point ir = irTopLeft;
		glPointSize(1);
		glColor3f(1, 0, 1);
		glBegin(GL_POINTS);
		int nGate = GV2.GetInt("CameraCalibrator.MeanGate", 10, SILENT);

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

	if ((int)mvCorners.size() < GV2.GetInt("CameraCalibrator.MinCornersForGrabbedImage", 20, SILENT))
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
	Params.v2Pos = size2Vec(irBestCenterPos);
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
	static gvar3<int> gvnCornerPatchSize("CameraCalibrator.CornerPatchPixelSize", 20, SILENT);
	CalibGridCorner &gSrc = mvGridCorners[nSrc];

	cv::Point irBest;
	double dBestDist = 99999;
	TooN::Vector<2> v2TargetDirn = gSrc.Params.m2Warp().T()[nDirn % 2];
	if (nDirn >= 2)
		v2TargetDirn *= -1;
	for (unsigned int i = 0; i < mvCorners.size(); i++)
	{
		TooN::Vector<2> v2Diff = size2Vec(mvCorners[i]) - gSrc.Params.v2Pos;
		if (v2Diff * v2Diff < 100)
			continue;
		if (v2Diff * v2Diff > dBestDist * dBestDist)
			continue;
		TooN::Vector<2> v2Dirn = v2Diff;
		TooN::normalize(v2Dirn);
		if (v2Dirn * v2TargetDirn < cos(M_PI / 18.0))
			continue;
		dBestDist = sqrt(v2Diff * v2Diff);
		irBest = mvCorners[i];
	}

	CalibGridCorner gTarget;
	gTarget.Params = gSrc.Params;
	gTarget.Params.v2Pos = size2Vec(irBest);
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
	glVertex(Params.v2Pos + Params.m2Warp() * size2Vec(cv::Point(10, 0)));
	glVertex(Params.v2Pos + Params.m2Warp() * size2Vec(cv::Point(-10, 0)));
	glVertex(Params.v2Pos + Params.m2Warp() * size2Vec(cv::Point(0, 10)));
	glVertex(Params.v2Pos + Params.m2Warp() * size2Vec(cv::Point(0, -10)));
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


Matrix<2> CalibGridCorner::GetSteps(vector<CalibGridCorner> &vgc)
{
  Matrix<2> m2Steps;
  for(int dirn=0; dirn<2; dirn++)
    {
      Vector<2> v2Dirn;
      int nFound = 0;
      v2Dirn = Zeros;
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
      if(nFound == 0)
	m2Steps[dirn] = mInheritedSteps[dirn];
      else
	m2Steps[dirn] = v2Dirn / nFound;
    }
  return m2Steps;
};

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
  static gvar3<double> gvdMaxStepDistFraction("CameraCalibrator.ExpandByStepMaxDistFrac", 0.3, SILENT);
  static gvar3<int> gvnCornerPatchSize("CameraCalibrator.CornerPatchPixelSize", 20, SILENT);
  
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

  Vector<2> v2Step;
  //ImageRef irGridStep = IR_from_dirn(nDirn);
  
  cv::Point irGridStep = IR_from_dirn(nDirn);
  v2Step = gSrc.GetSteps(mvGridCorners).T() * size2Vec(irGridStep);
  
  Vector<2> v2SearchPos = gSrc.Params.v2Pos + v2Step;
  
  // Before the search: pre-fill the failure result for easy returns.
  gSrc.aNeighborStates[nDirn].val = N_FAILED;
  
  //ImageRef irBest;
  cv::Point irBest;
  double dBestDist = 99999;
  for(unsigned int i=0; i<mvCorners.size(); i++)
    {
      Vector<2> v2Diff = size2Vec(mvCorners[i]) - v2SearchPos;
      if(v2Diff * v2Diff > dBestDist * dBestDist)
	continue;
      dBestDist = sqrt(v2Diff * v2Diff);
      irBest = mvCorners[i];
    }
  
  double dStepDist= sqrt(v2Step * v2Step);
  if(dBestDist > *gvdMaxStepDistFraction * dStepDist)
    return;
  
  CalibGridCorner gTarget;
  gTarget.Params = gSrc.Params;
  gTarget.Params.v2Pos = size2Vec(irBest);
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
	    glVertex(mvGridCorners[i].Params.v2Pos);
	    glVertex(mvGridCorners[mvGridCorners[i].aNeighborStates[dirn].val].Params.v2Pos);
	  }
    }
  glEnd();
  
  glPointSize(5);
  glEnable(GL_POINT_SMOOTH);
  glColor3f(1,1,0);
  glBegin(GL_POINTS);
  for(unsigned int i=0; i<mvGridCorners.size(); i++)
    glVertex(mvGridCorners[i].Params.v2Pos);
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
	    Vector<3> v3; v3[2] = 0.0;
	    v3.slice<0,2>() = size2Vec(mvGridCorners[i].irGridPos);
	    glVertex(Camera.Project(project(mse3CamFromWorld * v3)));
	    v3.slice<0,2>() = size2Vec(mvGridCorners[mvGridCorners[i].aNeighborStates[dirn].val].irGridPos);
	    glVertex(Camera.Project(project(mse3CamFromWorld * v3)));
	  }
    }
  glEnd();

  if(bDrawErrors)
    {
      glColor3f(1,0,0);
      glLineWidth(1);
      glBegin(GL_LINES);
      for(int i=0; i< (int) mvGridCorners.size(); i++)
	{
	  Vector<3> v3; v3[2] = 0.0;
	  v3.slice<0,2>() = size2Vec(mvGridCorners[i].irGridPos);
	  Vector<2> v2Pixels_Projected = Camera.Project(project(mse3CamFromWorld * v3));
	  Vector<2> v2Error = mvGridCorners[i].Params.v2Pos - v2Pixels_Projected;
	  glVertex(v2Pixels_Projected);
	  glVertex(v2Pixels_Projected + 10.0 * v2Error);
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
  // First, find a homography which maps the grid to the unprojected image coords
  // Use the standard null-space-of-SVD-thing to find 9 homography parms
  // (c.f. appendix of thesis)
  
  int nPoints = mvGridCorners.size();
  Matrix<> m2Nx9(2*nPoints, 9);
  for(int n=0; n<nPoints; n++)
    {
      // First, un-project the points to the image plane
      Vector<2> v2UnProj = Camera.UnProject(mvGridCorners[n].Params.v2Pos);
      double u = v2UnProj[0];
      double v = v2UnProj[1];
      // Then fill in the matrix..
      double x = mvGridCorners[n].irGridPos.x;
      double y = mvGridCorners[n].irGridPos.y;
      
      m2Nx9[n*2+0][0] = x;
      m2Nx9[n*2+0][1] = y;
      m2Nx9[n*2+0][2] = 1;
      m2Nx9[n*2+0][3] = 0;
      m2Nx9[n*2+0][4] = 0;
      m2Nx9[n*2+0][5] = 0;
      m2Nx9[n*2+0][6] = -x*u;
      m2Nx9[n*2+0][7] = -y*u;
      m2Nx9[n*2+0][8] = -u;

      m2Nx9[n*2+1][0] = 0;
      m2Nx9[n*2+1][1] = 0;
      m2Nx9[n*2+1][2] = 0;
      m2Nx9[n*2+1][3] = x;
      m2Nx9[n*2+1][4] = y;
      m2Nx9[n*2+1][5] = 1;
      m2Nx9[n*2+1][6] = -x*v;
      m2Nx9[n*2+1][7] = -y*v;
      m2Nx9[n*2+1][8] = -v;
    }

  // The right null-space (should only be one) of the matrix gives the homography...
  TooN::SVD<> svdHomography(m2Nx9);
  Vector<9> vH = svdHomography.get_VT()[8];
  Matrix<3> m3Homography;
  m3Homography[0] = vH.slice<0,3>();
  m3Homography[1] = vH.slice<3,3>();
  m3Homography[2] = vH.slice<6,3>();
  
  
  // Fix up possibly poorly conditioned bits of the homography
  {
    TooN::SVD<2> svdTopLeftBit(m3Homography.slice<0,0,2,2>());
    Vector<2> v2Diagonal = svdTopLeftBit.get_diagonal();
    m3Homography = m3Homography / v2Diagonal[0];
    v2Diagonal = v2Diagonal / v2Diagonal[0];
    double dLambda2 = v2Diagonal[1];
    
    Vector<2> v2b;   // This is one hypothesis for v2b ; the other is the negative.
    v2b[0] = 0.0;
    v2b[1] = sqrt( 1.0 - (dLambda2 * dLambda2)); 
    
    Vector<2> v2aprime = v2b * svdTopLeftBit.get_VT();
    
    Vector<2> v2a = m3Homography[2].slice<0,2>();
    double dDotProd = v2a * v2aprime;
    
    if(dDotProd>0) 
      m3Homography[2].slice<0,2>() = v2aprime;
    else
      m3Homography[2].slice<0,2>() = -v2aprime;
  }
 
  
  // OK, now turn homography into something 3D ...simple gram-schmidt ortho-norm
  // Take 3x3 matrix H with column: abt
  // And add a new 3rd column: abct
  Matrix<3> mRotation;
  Vector<3> vTranslation;
  double dMag1 = sqrt(m3Homography.T()[0] * m3Homography.T()[0]);
  m3Homography = m3Homography / dMag1;
  
  mRotation.T()[0] = m3Homography.T()[0];
  
  // ( all components of the first vector are removed from the second...
  
  mRotation.T()[1] = m3Homography.T()[1] - m3Homography.T()[0]*(m3Homography.T()[0]*m3Homography.T()[1]); 
  mRotation.T()[1] /= sqrt(mRotation.T()[1] * mRotation.T()[1]);
  mRotation.T()[2] = mRotation.T()[0]^mRotation.T()[1];
  vTranslation = m3Homography.T()[2];
  
  // Store result
  mse3CamFromWorld.get_rotation()=mRotation;
  mse3CamFromWorld.get_translation() = vTranslation;
};

vector<CalibImage::ErrorAndJacobians> CalibImage::Project(ATANCamera &Camera)
{
  vector<ErrorAndJacobians> vResult;
  for(unsigned int n=0; n<mvGridCorners.size(); n++)
    {
      ErrorAndJacobians EAJ;
      
      // First, project into image...
      Vector<3> v3World;
      v3World[2] = 0.0;
      v3World.slice<0,2>() = size2Vec(mvGridCorners[n].irGridPos);
      
      Vector<3> v3Cam = mse3CamFromWorld * v3World;
      if(v3Cam[2] <= 0.001)
	continue;
      
      Vector<2> v2Image = Camera.Project(project(v3Cam));
      if(Camera.Invalid())
	continue;
      
      EAJ.v2Error = mvGridCorners[n].Params.v2Pos - v2Image;
      
      // Now find motion jacobian..
      double dOneOverCameraZ = 1.0 / v3Cam[2];
      Matrix<2> m2CamDerivs = Camera.GetProjectionDerivs();
      
      for(int dof=0; dof<6; dof++)
	{
	  const Vector<4> v4Motion = SE3<>::generator_field(dof, unproject(v3Cam));
	  Vector<2> v2CamFrameMotion;
	  v2CamFrameMotion[0] = (v4Motion[0] - v3Cam[0] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
	  v2CamFrameMotion[1] = (v4Motion[1] - v3Cam[1] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
	  EAJ.m26PoseJac.T()[dof] = m2CamDerivs * v2CamFrameMotion;
	};

      // Finally, the camera provids its own jacobian
      EAJ.m2NCameraJac = Camera.GetCameraParameterDerivs();
      vResult.push_back(EAJ);
    }
  return vResult;
};
