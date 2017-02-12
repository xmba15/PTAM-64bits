// *-* c++ *-*
// Copyright 2008 Isis Innovation Limited

// N-th implementation of a camera model
// GK 2007
// Evolved a half dozen times from the CVD-like model I was given by
// TWD in 2000
// 
// This one uses the ``FOV'' distortion model of
// Deverneay and Faugeras, Straight lines have to be straight, 2001
//
// BEWARE: This camera model caches intermediate results in member variables
// Some functions therefore depend on being called in order: i.e.
// GetProjectionDerivs() uses data stored from the last Project() or UnProject()
// THIS MEANS YOU MUST BE CAREFUL WITH MULTIPLE THREADS
// Best bet is to give each thread its own version of the camera!
//
// Camera parameters are stored in a GVar, but changing the gvar has no effect
// until the next call to RefreshParams() or SetImageSize().
//
// Pixel conventions are as follows:
// For Project() and Unproject(),
// round pixel values - i.e. (0.0, 0.0) - refer to pixel centers
// I.e. the top left pixel in the image covers is centered on (0,0)
// and covers the area (-.5, -.5) to (.5, .5)
//
// Be aware that this is not the same as what opengl uses but makes sense
// for acessing pixels using ImageRef, especially ir_rounded.
//
// What is the UFB?
// This is for projecting the visible image area
// to a unit square coordinate system, with the top-left at 0,0,
// and the bottom-right at 1,1
// This is useful for rendering into textures! The top-left pixel is NOT
// centered at 0,0, rather the top-left corner of the top-left pixel is at 
// 0,0!!! This is the way OpenGL thinks of pixel coords.
// There's the Linear and the Distorting version - 
// For the linear version, can use 
// glMatrixMode(GL_PROJECTION); glLoadIdentity();
// glMultMatrix(Camera.MakeUFBLinearFrustumMatrix(near,far));
// To render un-distorted geometry with full frame coverage.
//
#pragma once

#include <cmath>
#include "Persistence/PVars.h"
#include "additionalUtility.h"

using namespace additionalUtility;

#define NUMTRACKERCAMPARAMETERS 5
#define CAPTURE_SIZE_X	640
#define CAPTURE_SIZE_Y	480

class CameraCalibrator;
class CalibImage;

// The parameters are:
// 0 - normalized x focal length 
// 1 - normalized y focal length
// 2 - normalized x offset
// 3 - normalized y offset
// 4 - w (distortion parameter)

class ATANCamera {
 public:

	 static cv::Vec<double, NUMTRACKERCAMPARAMETERS> mvDefaultParams;
	 ATANCamera(std::string sName, const cv::Size imgsize = cv::Size(CAPTURE_SIZE_X, CAPTURE_SIZE_Y));

  // Image size get/set: updates the internal projection params to that target image size.
	 inline void SetImageSize(const cv::Size &imgsize) { mvImageSize = imgsize; RefreshParams(); }
	 inline cv::Size GetImageSize() { return mvImageSize; };
	 void RefreshParams();
  
  // Various projection functions
  inline cv::Vec2d Project(const cv::Vec2d &vNormEuc) {
	  mvLastCam = vNormEuc;
	  // get the distance from the origin in the Euclidean projection plane n mdLastR
	  mdLastR = cv::norm(mvLastCam); // This is the undistorted (presumably) radius of the normalized Euclidean coordinates
	  mbInvalid = (mdLastR > mdMaxR); // We cant have a radius beyond the maximum radius 
									  // (as estimated from image border back-projections/un-projections in refreshparams()
	  mdLastFactor = rtrans_factor(mdLastR); // so rtrans_factor is the DISTORTION factor function 
	  mdLastDistR = mdLastFactor * mdLastR;  // Get the distorted radius and chache it for potential use in Jacobian computations
	  mvLastDistCam = mdLastFactor * mvLastCam; // Now get the distorted coordinates

												// having the distorted normalized Euclidean coordinates, we can now project on the image (and chache the result in mvLastIm)...
	  mvLastIm[0] = mvCenter[0] + mvFocal[0] * mvLastDistCam[0];
	  mvLastIm[1] = mvCenter[1] + mvFocal[1] * mvLastDistCam[1];

	  return mvLastIm;
  }

  inline cv::Vec2d Project(double xe, double ye) {
	  mvLastCam = cv::Vec2d(xe, ye);
	  // get the distance from the origin in the Euclidean projection plane n mdLastR
	  mdLastR = cv::norm(mvLastCam); // This is the undistorted (presumably) radius of the normalized Euclidean coordinates
	  mbInvalid = (mdLastR > mdMaxR); // We cant have a radius beyond the maximum radius 
									  // (as estimated from image border back-projections/un-projections in refreshparams()
	  mdLastFactor = rtrans_factor(mdLastR); // so rtrans_factor is the DISTORTION factor function 
	  mdLastDistR = mdLastFactor * mdLastR;  // Get the distorted radius and chache it for potential use in Jacobian computations
	  mvLastDistCam = mdLastFactor * mvLastCam; // Now get the distorted coordinates

												// having the distorted normalized Euclidean coordinates, we can now project on the image (and chache the result in mvLastIm)...
	  mvLastIm[0] = mvCenter[0] + mvFocal[0] * mvLastDistCam[0];
	  mvLastIm[1] = mvCenter[1] + mvFocal[1] * mvLastDistCam[1];

	  return mvLastIm;
  }

  // Un-project from image pixel coords to the  normalized Euclidean (z=1) camera  plane
  // while storing intermediate calculation results in member variables
  inline cv::Vec2d UnProject(const cv::Vec2d &v2Im) {

	  // store image location
	  mvLastIm = v2Im;
	  // Now unproject to a distorted Euclidean space
	  mvLastDistCam[0] = (mvLastIm[0] - mvCenter[0]) * mvInvFocal[0];
	  mvLastDistCam[1] = (mvLastIm[1] - mvCenter[1]) * mvInvFocal[1];
	  // Now, mvLastDistCam contains the DISTORTED Euclidean coordinates of the imaged point

	  // So now we compensate for radial distortion. Store the distorted radius in mdLastDistR .
	  mdLastDistR = cv::norm(mvLastDistCam);
	  // mdLastR now becomes the undistorted radius
	  mdLastR = invrtrans(mdLastDistR);  // tan(rd * w) / (2 * tan(w/2))
	  double dFactor; // the undistortion factor it should be ru/rd = mdLastR / mdLastDistR
	  if (mdLastDistR > 0.01) // if very far from the center (hence distortion is probably heavy)
		  dFactor = mdLastR / mdLastDistR;
	  else
		  dFactor = 1.0;
	  // storing the inverse undistortion factor (Yeah I know... Variable names couldn't get any worse....)
	  mdLastFactor = 1.0 / dFactor;
	  // storing the undistorted Euclidean coordinates
	  mvLastCam = dFactor * mvLastDistCam;

	  // return undistorted normalized Euclidean coordinates
	  return mvLastCam;
  }

  inline cv::Vec2d UnProject(double x, double y) {

	  // store image location
	  mvLastIm = cv::Vec2d(x, y);
	  // Now unproject to a distorted Euclidean space
	  mvLastDistCam[0] = (mvLastIm[0] - mvCenter[0]) * mvInvFocal[0];
	  mvLastDistCam[1] = (mvLastIm[1] - mvCenter[1]) * mvInvFocal[1];
	  // Now, mvLastDistCam contains the DISTORTED Euclidean coordinates of the imaged point

	  // So now we compensate for radial distortion. Store the distorted radius in mdLastDistR .
	  mdLastDistR = cv::norm(mvLastDistCam);
	  // mdLastR now becomes the undistorted radius
	  mdLastR = invrtrans(mdLastDistR);  // tan(rd * w) / (2 * tan(w/2))
	  double dFactor; // the undistortion factor it should be ru/rd = mdLastR / mdLastDistR
	  if (mdLastDistR > 0.01) // if very far from the center (hence distortion is probably heavy)
		  dFactor = mdLastR / mdLastDistR;
	  else
		  dFactor = 1.0;
	  // storing the inverse undistortion factor (Yeah I know... Variable names couldn't get any worse....)
	  mdLastFactor = 1.0 / dFactor;
	  // storing the undistorted Euclidean coordinates
	  mvLastCam = dFactor * mvLastDistCam;

	  // return undistorted normalized Euclidean coordinates
	  return mvLastCam;
  }

  cv::Vec2d UFBProject(const cv::Vec2d &camframe);
  cv::Vec2d UFBUnProject(const cv::Vec2d &camframe);
  inline cv::Vec2d UFBLinearProject(const cv::Vec2d &camframe);
  inline cv::Vec2d UFBLinearUnProject(const cv::Vec2d &fbframe);
    
  cv::Matx<double, 2, 2> GetProjectionDerivs(); // 2x2 Projection jacobian

  inline bool Invalid() {  return mbInvalid;}
  inline double LargestRadiusInImage() {  return mdLargestRadius; }
  inline double OnePixelDist() { return mdOnePixelDist; }
  
  // The z=1 plane bounding box of what the camera can see
  cv::Vec2d ImplaneTL(); 
  cv::Vec2d ImplaneBR(); 

  // OpenGL helper function
  cv::Matx<double, 4, 4> MakeUFBLinearFrustumMatrix(double _near, double _far); // Returns A 4x4 matrix

  // Feedback for Camera Calibrator
  double PixelAspectRatio() { return mvFocal[1] / mvFocal[0];}
    
 protected:
  Persistence::pvar3<cv::Vec<double, NUMTRACKERCAMPARAMETERS> > mpvvCameraParams; //The actual camera parameters
  cv::Matx<double, 2, NUMTRACKERCAMPARAMETERS> GetCamParamAnalyticalDerivs();
  cv::Matx<double, 2, NUMTRACKERCAMPARAMETERS> GetCameraParameterDerivs(); // 2x NUMTRACKERCAMPARAMETERS
  void UpdateParams(cv::Vec<double, NUMTRACKERCAMPARAMETERS> vUpdate);
  void DisableRadialDistortion();
  
  // Cached from the last project/unproject:
  cv::Vec2d mvLastCam;      // Last z=1 coord
  cv::Vec2d mvLastIm;       // Last image/UFB coord
  cv::Vec2d mvLastDistCam;  // Last distorted z=1 coord
  double mdLastR;           // Last z=1 radius
  double mdLastDistR;       // Last z=1 distorted radius
  double mdLastFactor;      // Last ratio of z=1 radii
  bool mbInvalid;           // Was the last projection invalid?
  
  // Cached from last RefreshParams:
  double mdLargestRadius; // Largest R in the image
  double mdMaxR;          // Largest R for which we consider projection valid
  double mdOnePixelDist;  // z=1 distance covered by a single pixel offset (a rough estimate!)
  double md2Tan;          // distortion model coeff
  double mdOneOver2Tan;   // distortion model coeff
  double mdW;             // distortion model coeff
  double mdWinv;          // distortion model coeff
  double mdDistortionEnabled; // One or zero depending on if distortion is on or off.
  cv::Vec2d mvCenter;     // Pixel projection center
  cv::Vec2d mvFocal;      // Pixel focal length
  cv::Vec2d mvInvFocal;   // Inverse pixel focal length
  cv::Size mvImageSize;
  cv::Vec2d mvUFBLinearFocal;
  cv::Vec2d mvUFBLinearInvFocal;
  cv::Vec2d mvUFBLinearCenter;
  cv::Vec2d mvImplaneTL;
  cv::Vec2d mvImplaneBR;
  
  // Radial distortion transformation factor: returns ration of distorted / undistorted radius.
  inline double rtrans_factor(double r)
  {
    if(r < 0.001 || mdW == 0.0)
      return 1.0;
    else 
      return (mdWinv* atan(r * md2Tan) / r);
  };

  // Inverse radial distortion: returns un-distorted radius from distorted.
  inline double invrtrans(double r)
  {
    if(mdW == 0.0)
      return r;
    return(tan(r * mdW) * mdOneOver2Tan);
  };
  
  std::string msName;

  friend class CameraCalibrator;   // friend declarations allow access to calibration jacobian and camera update function.
  friend class CalibImage;
};

// Some inline projection functions:
inline cv::Vec2d ATANCamera::UFBLinearProject(const cv::Vec2d &camframe)
{
  cv::Vec2d v2Res;
  v2Res[0] = camframe[0] * mvUFBLinearFocal[0] + mvUFBLinearCenter[0];
  v2Res[1] = camframe[1] * mvUFBLinearFocal[1] + mvUFBLinearCenter[1];
  return v2Res;
}

inline cv::Vec2d ATANCamera::UFBLinearUnProject(const cv::Vec2d &fbframe)
{
  cv::Vec2d v2Res;
  v2Res[0] = (fbframe[0] - mvUFBLinearCenter[0]) * mvUFBLinearInvFocal[0];
  v2Res[1] = (fbframe[1] - mvUFBLinearCenter[1]) * mvUFBLinearInvFocal[1];
  return v2Res;
}
