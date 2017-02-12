// Copyright 2008 Isis Innovation Limited

#include "OpenGL.h"
#include "GCVD/GLHelpers.h"
#include "Persistence/instances.h"
#include "CameraCalibrator.h"
#include <fstream>
#include <stdlib.h>

using namespace Persistence;

int main()
{
  std::cout << "  Welcome to CameraCalibrator " << std::endl;
  std::cout << "  -------------------------------------- " << std::endl;
  std::cout << "  Parallel tracking and mapping for Small AR workspaces" << std::endl;
  std::cout << "  Copyright (C) Isis Innovation Limited 2008 " << std::endl;
  std::cout << std::endl;  
  std::cout << "  Parsing calibrator_settings.cfg ...." << std::endl;
  
  GUI.LoadFile("calibrator_settings.cfg");

  GUI.StartParserThread();
  atexit(GUI.StopParserThread); // Clean up readline when program quits
  
  Persistence::PV3::get<cv::Vec<double, NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, Persistence::SILENT);

  try
    {
      CameraCalibrator c;
      c.Run();
    }
  catch (cv::Exception e)
  {
	  cout << endl;
	  cout << "!! Failed to run CameraCalibrator; got exception. " << endl;
	  cout << "   Exception was: " << endl;
	  cout << "At line : " << e.line << endl << e.msg << endl;
  }
}

CameraCalibrator::CameraCalibrator()
  :mGLWindow(mVideoSource.imgSize(), "Camera Calibrator"), mCamera("Camera")
{
  mbDone = false;
  GUI.RegisterCommand("CameraCalibrator.GrabNextFrame", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.Reset", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.ShowNext", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.SaveCalib", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);
  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  PV3::Register(mgvnOptimizing, "CameraCalibrator.Optimize", 0, SILENT);
  PV3::Register(mgvnShowImage, "CameraCalibrator.Show", 0, SILENT);
  PV3::Register(mgvnDisableDistortion, "CameraCalibrator.NoDistortion", 0, SILENT);
  GUI.ParseLine("GLWindow.AddMenu CalibMenu");
  GUI.ParseLine("CalibMenu.AddMenuButton Live GrabFrame CameraCalibrator.GrabNextFrame");
  GUI.ParseLine("CalibMenu.AddMenuButton Live Reset CameraCalibrator.Reset");
  GUI.ParseLine("CalibMenu.AddMenuButton Live Optimize \"CameraCalibrator.Optimize=1\"");
  GUI.ParseLine("CalibMenu.AddMenuToggle Live NoDist CameraCalibrator.NoDistortion");
  GUI.ParseLine("CalibMenu.AddMenuSlider Opti \"Show Img\" CameraCalibrator.Show 0 10");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti \"Show Next\" CameraCalibrator.ShowNext");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti \"Grab More\" CameraCalibrator.Optimize=0 ");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti Reset CameraCalibrator.Reset");
  GUI.ParseLine("CalibMenu.AddMenuToggle Opti NoDist CameraCalibrator.NoDistortion");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti Save CameraCalibrator.SaveCalib");
  Reset();
}

void CameraCalibrator::Run()
{
  while(!mbDone)
    {
      // We use two versions of each video frame:
      // One black and white (for processing by the tracker etc)
      // and one RGB, for drawing.

	  cv::Mat imFrameRGB;
	  cv::Mat imFrameBW;

      // Grab new video frame...
      mVideoSource.GetAndFillFrameBWandRGB(imFrameBW, imFrameRGB);  
      
      // Set up openGL
      mGLWindow.SetupViewport();
      mGLWindow.SetupVideoOrtho();
      mGLWindow.SetupVideoRasterPosAndZoom();
      
      if(mvCalibImgs.size() < 1)
	*mgvnOptimizing = 0;
      
      if(!*mgvnOptimizing)
	{
	  GUI.ParseLine("CalibMenu.ShowMenu Live");
	  GLXInterface::glDrawPixelsGRAY(imFrameBW);
	  
	  CalibImage c;
	  if(c.MakeFromImage((cv::Mat_<uchar>) imFrameBW, imFrameRGB))
	    {
	      if(mbGrabNextFrame)
		{
		  mvCalibImgs.push_back(c);
		  mvCalibImgs.back().GuessInitialPose(mCamera);
		  mvCalibImgs.back().Draw3DGrid(mCamera, false);
		  mbGrabNextFrame = false;
		};
	    }
	}
      else
	{
	  OptimizeOneStep();
      
	  GUI.ParseLine("CalibMenu.ShowMenu Opti");
	  int nToShow = *mgvnShowImage - 1;
	  if(nToShow < 0)
	    nToShow = 0;
	  if(nToShow >= (int) mvCalibImgs.size())
	    nToShow = mvCalibImgs.size()-1;
	  *mgvnShowImage = nToShow + 1;
	  GLXInterface::glDrawPixelsGRAY(mvCalibImgs[nToShow].mim);
	  mvCalibImgs[nToShow].Draw3DGrid(mCamera,true);
	}
      
      ostringstream ost;
      ost << "Camera Calibration: Grabbed " << mvCalibImgs.size() << " images." << endl;
      if(!*mgvnOptimizing)
	{
	  ost << "Take snapshots of the calib grid with the \"GrabFrame\" button," << endl;
	  ost << "and then press \"Optimize\"." << endl;
	  ost << "Take enough shots (4+) at different angles to get points " << endl;
	  ost << "into all parts of the image (corners too.) The whole grid " << endl;
	  ost << "doesn't need to be visible so feel free to zoom in." << endl;
	}
      else
	{
	  ost << "Current RMS pixel error is " << mdMeanPixelError << endl;
	  ost << "Current camera params are  " << PV3::get_var("Camera.Parameters") << endl;
	  ost << "(That would be a pixel aspect ratio of " 
	      <<  mCamera.PixelAspectRatio() << ")" << endl;
	  ost << "Check fit by looking through the grabbed images." << endl;
	  ost << "RMS should go below 0.5, typically below 0.3 for a wide lens." << endl;
	  ost << "Press \"save\" to save calibration to camera.cfg file and exit." << endl;
	}

      mGLWindow.DrawCaption(ost.str());
      mGLWindow.DrawMenus();
      mGLWindow.HandlePendingEvents();
      mGLWindow.swap_buffers();
    }
}

void CameraCalibrator::Reset()
{
  *mCamera.mpvvCameraParams = ATANCamera::mvDefaultParams;
  if(*mgvnDisableDistortion) mCamera.DisableRadialDistortion();
  
  mCamera.SetImageSize(mVideoSource.imgSize());
  mbGrabNextFrame =false;
  *mgvnOptimizing = false;
  mvCalibImgs.clear();
}

void CameraCalibrator::GUICommandCallBack(void* ptr, string sCommand, string sParams)
{
  ((CameraCalibrator*) ptr)->GUICommandHandler(sCommand, sParams);
}

void CameraCalibrator::GUICommandHandler(string sCommand, string sParams)  // Called by the callback func..
{
  if(sCommand=="CameraCalibrator.Reset")
    {
      Reset();
      return;
    };
  if(sCommand=="CameraCalibrator.GrabNextFrame")
    {
      mbGrabNextFrame = true;
      return;
    }
  if(sCommand=="CameraCalibrator.ShowNext")
    {
      int nToShow = (*mgvnShowImage - 1 + 1) % mvCalibImgs.size();
      *mgvnShowImage = nToShow + 1;
      return;
    }
  if(sCommand=="CameraCalibrator.SaveCalib")
    {
      std::cout << "  Camera calib is " << PV3::get_var("Camera.Parameters") << std::endl;
      std::cout << "  Saving camera calib to camera.cfg..." << std::endl;
      ofstream ofs("camera.cfg");
      if(ofs.good())
	{
	  PV3::PrintVar("Camera.Parameters", ofs);
	  ofs.close();
	  cout << "  .. saved."<< endl;
	}
      else
	{
	  cout <<"! Could not open camera.cfg for writing." << endl;
	  PV3::PrintVar("Camera.Parameters", cout);
	  cout <<"  Copy-paste above line to settings.cfg or camera.cfg! " << endl;
	}
      mbDone = true;
    }
  if(sCommand=="exit" || sCommand=="quit")
    {
      mbDone = true;
    }
}

void CameraCalibrator::OptimizeOneStep()
{
  int nViews = mvCalibImgs.size();
  int nDim = 6 * nViews + NUMTRACKERCAMPARAMETERS;
  int nCamParamBase = nDim - NUMTRACKERCAMPARAMETERS;
  
  cv::Mat_<double> mJTJ = cv::Mat_<double>::eye(nDim, nDim); // a matrix vector... Smells like Least Squares....
  cv::Mat_<double> vJTe = cv::Mat_<double>::zeros(nDim, 1);

  //Matrix<> mJTJ(nDim, nDim);
  //Vector<> vJTe(nDim);
  //mJTJ = Identity; // Weak stabilizing prior
  //vJTe = Zeros;

  if(*mgvnDisableDistortion) mCamera.DisableRadialDistortion();

  
  double dSumSquaredError = 0.0;
  int nTotalMeas = 0;
  
  for (int n = 0; n < nViews; n++)
  {
	  int nMotionBase = n * 6;
	  vector<CalibImage::ErrorAndJacobians> vEAJ = mvCalibImgs[n].Project(mCamera);

	  if (vEAJ.size() == 0) {
		  cout << "All point projections are invalid with current parameters. Leaving image out of the optimization..." << endl;

		  continue;

	  }

	  for (unsigned int i = 0; i < vEAJ.size(); i++)
	  {
		  int r_, c_;
		  CalibImage::ErrorAndJacobians &EAJ = vEAJ[i];
		  // All the below should be +=, but the MSVC compiler doesn't seem to understand that. :(
		  for (r_ = 0; r_ < 6; r_++) {
			  mJTJ(nMotionBase + r_, nMotionBase + r_) += EAJ.m26PoseJac(0, r_) * EAJ.m26PoseJac(0, r_) +
				  EAJ.m26PoseJac(1, r_) * EAJ.m26PoseJac(1, r_);
			  for (c_ = 0; c_ < r_; c_++)
				  mJTJ(r_ + nMotionBase, c_ + nMotionBase) = (mJTJ(c_ + nMotionBase, r_ + nMotionBase) += EAJ.m26PoseJac(0, r_) * EAJ.m26PoseJac(0, c_) +
					  EAJ.m26PoseJac(1, r_) * EAJ.m26PoseJac(1, c_));
		  }

		  for (r_ = 0; r_ < NUMTRACKERCAMPARAMETERS; r_++) {

			  mJTJ(nCamParamBase + r_, nCamParamBase + r_) += EAJ.m2NCameraJac(0, r_) * EAJ.m2NCameraJac(0, r_) +
				  EAJ.m2NCameraJac(1, r_) * EAJ.m2NCameraJac(1, r_);

			  for (c_ = 0; c_ < r_; c_++)
				  mJTJ(r_ + nCamParamBase, c_ + nCamParamBase) = (mJTJ(c_ + nCamParamBase, r_ + nCamParamBase) +=
					  EAJ.m2NCameraJac(0, r_) * EAJ.m2NCameraJac(0, c_) +
					  EAJ.m2NCameraJac(1, r_) * EAJ.m2NCameraJac(1, c_));
		  }

		  for (r_ = 0; r_ < 6; r_++)
			  for (c_ = 0; c_ < NUMTRACKERCAMPARAMETERS; c_++)
				  mJTJ(r_ + nMotionBase, c_ + nCamParamBase) = (mJTJ(c_ + nCamParamBase, r_ + nMotionBase) +=
					  EAJ.m26PoseJac(0, r_) * EAJ.m2NCameraJac(0, c_) +
					  EAJ.m26PoseJac(1, r_) * EAJ.m2NCameraJac(1, c_));


		  for (r_ = 0; r_ < 6; r_++)
			  vJTe(r_ + nMotionBase) += EAJ.m26PoseJac(0, r_) * EAJ.v2Error[0] +
			  EAJ.m26PoseJac(1, r_) * EAJ.v2Error[1];

		  for (c_ = 0; c_ < NUMTRACKERCAMPARAMETERS; c_++)
			  vJTe(c_ + nCamParamBase, 0) += EAJ.m2NCameraJac(0, c_) * EAJ.v2Error[0] +
			  EAJ.m2NCameraJac(1, c_) * EAJ.v2Error[1];

		  dSumSquaredError += EAJ.v2Error[0] * EAJ.v2Error[0] + EAJ.v2Error[1] * EAJ.v2Error[1];

		  ++nTotalMeas;
	  }
  };

  if (nTotalMeas == 0) {
	  cout << "Did not manage to include a single grid corner in the optimization ! Skipping updates !" << endl;
	  return;
  }

  mdMeanPixelError = sqrt(dSumSquaredError / nTotalMeas);

  cv::Mat_<double> vUpdate(nDim, 1);
  cv::solve(mJTJ, vJTe, vUpdate, cv::DECOMP_CHOLESKY);
  vUpdate *= 0.1; // Slow down because highly nonlinear...
  for (int n = 0; n < nViews; n++)
  {
	  SE3<> Dse3 = SE3<>::exp(cv::Vec<double, 6>(vUpdate(n * 6 + 0, 0),
		  vUpdate(n * 6 + 1, 0),
		  vUpdate(n * 6 + 2, 0),
		  vUpdate(n * 6 + 3, 0),
		  vUpdate(n * 6 + 4, 0),
		  vUpdate(n * 6 + 5, 0))
	  );
	  mvCalibImgs[n].mse3CamFromWorld = Dse3 * mvCalibImgs[n].mse3CamFromWorld;
  }
  cv::Vec<double, NUMTRACKERCAMPARAMETERS> Dparams;
  for (int k = 0; k<NUMTRACKERCAMPARAMETERS; k++) Dparams[k] = vUpdate(nCamParamBase + k, 0);
  mCamera.UpdateParams(Dparams);
};