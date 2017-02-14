#include "ATANCamera.h"
#include <iostream>
#include <algorithm>
#include "Persistence/instances.h" 

using namespace std;
using namespace Persistence;

const cv::Vec<double, NUMTRACKERCAMPARAMETERS> ATANCamera::mvDefaultParams = cv::Vec<double, NUMTRACKERCAMPARAMETERS>(0.5, 0.75, 0.5, 0.5, 0.1);

ATANCamera::ATANCamera(string sName, const cv::Size imgsize)
{
  // The camera name is used to find the camera's parameters in a GVar.
  msName = sName;
  // Need to do a "get" in order to put the tag msName+".Parameters" in the list. value is loaded either from the file, or from the defaults
  Persistence::PV3::get<cv::Vec<double, NUMTRACKERCAMPARAMETERS> >(msName + ".Parameters", mvDefaultParams, Persistence::SILENT);
  Persistence::PV3.Register(mpvvCameraParams, sName + ".Parameters", mvDefaultParams, HIDDEN | FATAL_IF_NOT_DEFINED);
  mvImageSize = imgsize;
  RefreshParams();
}

void ATANCamera::RefreshParams() 
{
  // This updates internal member variables according to the current camera parameters,
  // and the currently selected target image size.
  //
  
  // First: Focal length and image center in pixel coordinates
  mvFocal[0] = mvImageSize.width * (*mpvvCameraParams)[0];
  mvFocal[1] = mvImageSize.height * (*mpvvCameraParams)[1];
  mvCenter[0] = mvImageSize.width * (*mpvvCameraParams)[2] - 0.5;
  mvCenter[1] = mvImageSize.height * (*mpvvCameraParams)[3] - 0.5;
  
  // One over focal length
  mvInvFocal[0] = 1.0 / mvFocal[0];
  mvInvFocal[1] = 1.0 / mvFocal[1];

  // Some radial distortion parameters..
  mdW =  (*mpvvCameraParams)[4];
  if(mdW != 0.0)
    {
      md2Tan = 2.0 * tan(mdW / 2.0);
      mdOneOver2Tan = 1.0 / md2Tan;
      mdWinv = 1.0 / mdW;
      mdDistortionEnabled = 1.0;
    }
  else
    {
      mdWinv = 0.0;
      md2Tan = 0.0;
      mdDistortionEnabled = 0.0;
    }
  
  // work out biggest radius in image
  cv::Vec2d v2( std::max((*mpvvCameraParams)[2], 1.0 - (*mpvvCameraParams)[2]) / (*mpvvCameraParams)[0],
                std::max((*mpvvCameraParams)[3], 1.0 - (*mpvvCameraParams)[3]) / (*mpvvCameraParams)[1]);
  mdLargestRadius = invrtrans(cv::norm(v2));
  
  // At what stage does the model become invalid?
  mdMaxR = 1.5 * mdLargestRadius; // (pretty arbitrary)

  // work out world radius of one pixel
  // (This only really makes sense for square-ish pixels)
  {
    cv::Vec2d v2Center = UnProject(mvImageSize.width * 0.5, mvImageSize.height * 0.5);
    //TooN::Vector<2> v2RootTwoAway = UnProject(mvImageSize / 2 + vec(ImageRef(1,1)));
	cv::Vec2d v2RootTwoAway = UnProject(mvImageSize.width * 0.5 + 1, mvImageSize.height * 0.5 + 1);
	cv::Vec2d v2Diff = v2Center - v2RootTwoAway;
    mdOnePixelDist = cv::norm(v2Diff) / sqrt(2.0);
  }
  
  // Work out the linear projection values for the UFB
  {
    // First: Find out how big the linear bounding rectangle must be
    std::vector<cv::Vec2d > vv2Verts;
    vv2Verts.push_back(UnProject(-0.5, -0.5));
    vv2Verts.push_back(UnProject(mvImageSize.width - 0.5, -0.5));
    vv2Verts.push_back(UnProject(mvImageSize.width - 0.5, mvImageSize.height-0.5));
    vv2Verts.push_back(UnProject(-0.5, mvImageSize.height - 0.5));
    cv::Vec2d v2Min = vv2Verts[0];
    cv::Vec2d v2Max = vv2Verts[0];
    for(int i=0; i<4; i++)
      for(int j=0; j<2; j++)
	{
	  if(vv2Verts[i][j] < v2Min[j]) v2Min[j] = vv2Verts[i][j];
	  if(vv2Verts[i][j] > v2Max[j]) v2Max[j] = vv2Verts[i][j];
	}
    mvImplaneTL = v2Min;
    mvImplaneBR = v2Max;
    
    // Store projection parameters to fill this bounding box
    cv::Vec2d v2Range = v2Max - v2Min;
    mvUFBLinearInvFocal = v2Range;
    mvUFBLinearFocal[0] = 1.0 / mvUFBLinearInvFocal[0];
    mvUFBLinearFocal[1] = 1.0 / mvUFBLinearInvFocal[1];
    mvUFBLinearCenter[0] = -1.0 * v2Min[0] * mvUFBLinearFocal[0];
    mvUFBLinearCenter[1] = -1.0 * v2Min[1] * mvUFBLinearFocal[1];
  }
  
}

// Utility function for easy drawing with OpenGL
// C.f. comment in top of ATANCamera.h
cv::Matx<double, 4, 4> ATANCamera::MakeUFBLinearFrustumMatrix(double _near, double _far)
{
  cv::Matx<double, 4, 4> m4 = cv::Matx<double, 4, 4>::zeros();
  

  double left = mvImplaneTL[0] * _near;
  double right = mvImplaneBR[0] * _near;
  double top = mvImplaneTL[1] * _near;
  double bottom = mvImplaneBR[1] * _near;
  
  // The openGhelL frustum manpage is A PACK OF LIES!!
  // Two of the elements are NOT what the manpage says they should be.
  // Anyway, below code makes a frustum projection matrix
  // Which projects a RHS-coord frame with +z in front of the camera
  // Which is what I usually want, instead of glFrustum's LHS, -z idea.
  m4(0, 0) = (2 * _near) / (right - left);
  m4(1, 1) = (2 * _near) / (top - bottom);
  
  m4(0, 2) = (right + left) / (left - right);
  m4(1, 2) = (top + bottom) / (bottom - top);
  m4(2, 2) = (_far + _near) / (_far - _near);
  m4(3, 2) = 1;
  
  m4(2, 3) = 2 * _near * _far / (_near - _far);

  return m4;
};

cv::Matx<double, 2, 2> ATANCamera::GetProjectionDerivs()
{
  // get the derivative of image frame wrt camera z=1 frame at the last computed projection
  // in the form (d im1/d cam1, d im1/d cam2)
  //             (d im2/d cam1, d im2/d cam2)
  
  double dFracBydx;
  double dFracBydy;
  
  double &k = md2Tan;
  double &x = mvLastCam[0];
  double &y = mvLastCam[1];
  double r = mdLastR * mdDistortionEnabled;
  
  if(r < 0.01)
    {
      dFracBydx = 0.0;
      dFracBydy = 0.0;
    }
  else
    {
      dFracBydx = 
	mdWinv * (k * x) / (r*r*(1 + k*k*r*r)) - x * mdLastFactor / (r*r); 
      dFracBydy = 
	mdWinv * (k * y) / (r*r*(1 + k*k*r*r)) - y * mdLastFactor / (r*r); 
    }
  
  cv::Matx<double, 2, 2> m2Derivs;
  
  m2Derivs(0, 0) = mvFocal[0] * (dFracBydx * x + mdLastFactor);  
  m2Derivs(1, 0) = mvFocal[1] * (dFracBydx * y);  
  m2Derivs(0, 1) = mvFocal[0] * (dFracBydy * x);  
  m2Derivs(1, 1) = mvFocal[1] * (dFracBydy * y + mdLastFactor);  
  return m2Derivs;
}

cv::Matx<double, 2, NUMTRACKERCAMPARAMETERS> ATANCamera::GetCamParamAnalyticalDerivs()
{
	// Differentials wrt to the camera parameters
	// Use these to calibrate the camera
	// No need for this to be quick, so do them numerically

	cv::Matx<double, 2, NUMTRACKERCAMPARAMETERS> m2nJ;

	cv::Vec2d v2Cam = mvLastCam;
	double ru = cv::norm(v2Cam); 	// undistorted radius
	double xe = v2Cam[0], ye = v2Cam[1]; // Euclidean normalized coordinates

	double imwidth = mvImageSize.width;
	double imheight = mvImageSize.height;

	double fx = imwidth * (*mpvvCameraParams)[0];
	double fy = imheight * (*mpvvCameraParams)[2];
	double distort = 1.0;
	double rd;
	double w = (*mpvvCameraParams)[4];

	double DrdDw = 0; // the derivative of the distorted radius wrt w
					  // **** Note that:
					  //
					  //   rd = atan(2 * ru * tan(w/2) ) / w
					  //
	if (w != 0 && mdW != 0) {
		double cc = 2 * ru*tan(w / 2);
		rd = atan(cc) / w;
		distort = rd / ru;

		DrdDw = (ru / (cos(w / 2)*cos(w / 2) * (1 + cc*cc)) - rd) / w;
	}

	// 1. Derivatives wrt sx = fx / w
	m2nJ(0, 0) = imwidth * xe * distort;
	m2nJ(1, 0) = 0;

	// 2. Derivatives wrt kx = cx / w
	m2nJ(0, 2) = imwidth;
	m2nJ(1, 2) = 0;

	// 3. Derivatives wrt sy = fy / h
	m2nJ(0, 1) = 0;
	m2nJ(1, 1) = imheight * ye * distort;

	// 4. Derivatives wrt ky = cy / h
	m2nJ(0, 3) = 0;
	m2nJ(1, 3) = imheight;

	// 5. Derivatives wrt w
	m2nJ(0, 4) = fx * xe * DrdDw / ru;
	m2nJ(1, 4) = fy * ye * DrdDw / ru;

	return m2nJ;
}


cv::Matx<double, 2, NUMTRACKERCAMPARAMETERS> ATANCamera::GetCameraParameterDerivs()
{
  // Differentials wrt to the camera parameters
  // Use these to calibrate the camera
  // No need for this to be quick, so do them numerically
  
  cv::Matx<double, 2, NUMTRACKERCAMPARAMETERS> m2NNumDerivs;
  cv::Vec<double, NUMTRACKERCAMPARAMETERS> vNNormal = *mpvvCameraParams;
  cv::Vec2d v2Cam = mvLastCam;
  cv::Vec2d v2Out = Project(v2Cam);
  for(int i=0; i<NUMTRACKERCAMPARAMETERS; i++)
    {
      if(i == NUMTRACKERCAMPARAMETERS-1 && mdW == 0.0)
	continue;
      cv::Vec<double, NUMTRACKERCAMPARAMETERS> vNUpdate;
      vNUpdate = cv::Vec<double, NUMTRACKERCAMPARAMETERS>::all(0);
      vNUpdate[i] += 0.001;
      UpdateParams(vNUpdate); 
      cv::Vec2d v2Out_B = Project(v2Cam);
	  cv::Vec2d DparamsByDpi = (v2Out_B - v2Out) / 0.001;
	  m2NNumDerivs(0, i) = DparamsByDpi[0];
	  m2NNumDerivs(1, i) = DparamsByDpi[1];
      *mpvvCameraParams = vNNormal;
      RefreshParams();
    }

  if (mdW == 0.0) {
	  m2NNumDerivs(0, NUMTRACKERCAMPARAMETERS - 1) = 0;
	  m2NNumDerivs(1, NUMTRACKERCAMPARAMETERS - 1) = 0;
  }
  return m2NNumDerivs;
}

void ATANCamera::UpdateParams(cv::Vec<double, NUMTRACKERCAMPARAMETERS> vUpdate)
{
  // Update the camera parameters; use this as part of camera calibration.
  (*mpvvCameraParams) = (*mpvvCameraParams) + vUpdate;
  RefreshParams();
}

void ATANCamera::DisableRadialDistortion()
{
  // Set the radial distortion parameter to zero
  // This disables radial distortion and also disables its differentials
  (*mpvvCameraParams)[NUMTRACKERCAMPARAMETERS-1] = 0.0;
  RefreshParams();
}

cv::Vec2d ATANCamera::UFBProject(const cv::Vec2d &vCam)
{
  // Project from camera z=1 plane to UFB, storing intermediate calc results.
  mvLastCam = vCam;
  mdLastR = cv::norm(vCam);
  mbInvalid = (mdLastR > mdMaxR);
  mdLastFactor = rtrans_factor(mdLastR);
  mdLastDistR = mdLastFactor * mdLastR;
  mvLastDistCam = mdLastFactor * mvLastCam;
  
  mvLastIm[0] = (*mpvvCameraParams)[2]  + (*mpvvCameraParams)[0] * mvLastDistCam[0];
  mvLastIm[1] = (*mpvvCameraParams)[3]  + (*mpvvCameraParams)[1] * mvLastDistCam[1];
  return mvLastIm;
}

cv::Vec2d ATANCamera::UFBUnProject(const cv::Vec2d &v2Im)
{
  mvLastIm = v2Im;
  mvLastDistCam[0] = (mvLastIm[0] - (*mpvvCameraParams)[2]) / (*mpvvCameraParams)[0];
  mvLastDistCam[1] = (mvLastIm[1] - (*mpvvCameraParams)[3]) / (*mpvvCameraParams)[1];
  mdLastDistR = cv::norm(mvLastDistCam);
  mdLastR = invrtrans(mdLastDistR);
  double dFactor;
  if(mdLastDistR > 0.01)
    dFactor =  mdLastR / mdLastDistR;
  else
    dFactor = 1.0;
  mdLastFactor = 1.0 / dFactor;
  mvLastCam = dFactor * mvLastDistCam;
  return mvLastCam;
}
