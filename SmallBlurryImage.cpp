// Copyright 2008 Isis Innovation Limited
#include "SmallBlurryImage.h"
#include "GCVD/GraphSLAM.h"
#include "GCVD/Addedutils.h"

using namespace std;

cv::Size SmallBlurryImage::mirSize(-1,-1);

SmallBlurryImage::SmallBlurryImage(KeyFrame &kf, double dBlur)
{
  mbMadeJacs = false;
  MakeFromKF(kf, dBlur);
}

SmallBlurryImage::SmallBlurryImage()
{
  mbMadeJacs = false;
}

// Make a SmallBlurryImage from a KeyFrame This fills in the mimSmall
// image (Which is just a small un-blurred version of the KF) and
// mimTemplate (which is a floating-point, zero-mean blurred version
// of the above)
void SmallBlurryImage::MakeFromKF(KeyFrame &kf, double dBlur)
{
  if(mirSize.width == -1)
    mirSize = kf.aLevels[3].im.size() / 2;

  mbMadeJacs = false;
  
  mimTemplate.create(mirSize);
  
  CvUtils::halfSample(kf.aLevels[3].im, mimSmall);
  
  unsigned int nSum = 0;
  
  double fMean = cv::mean(mimSmall)[0];
  
  for (int i = 0; i < mimTemplate.rows; i++) {
	  for (int j = 0; j < mimTemplate.cols; j++) {
		  mimTemplate.ptr<double>(i)[j] = mimSmall.ptr<uchar>(i)[j] - fMean;
	  }
  }

  int gkerSize = (int)ceil(dBlur*3.0);
  gkerSize += gkerSize % 2 == 0 ? 1 : 0;
  cv::GaussianBlur(mimTemplate, mimTemplate, cv::Size(gkerSize, gkerSize), dBlur);
}

// Make the jacobians (actually, no more than a gradient image)
// of the blurred template
void SmallBlurryImage::MakeJacs()
{
	mimImageJacs.create(mirSize);
	// Fill in the gradient image
	cv::Vec2d *jRowPtr;
	for (int i = 0; i < mirSize.height; i++) {
		jRowPtr = mimImageJacs.ptr<cv::Vec2d>(i);
		for (int j = 0; j < mirSize.width; j++) {
			if ((i == 0) || (j == 0) || (i == mirSize.height - 1) || (j == mirSize.width - 1))
				jRowPtr[j] = cv::Vec2d(0, 0);
			else {
				jRowPtr[j][0] = mimTemplate.ptr<double>(i)[j + 1] - mimTemplate.ptr<double>(i)[j - 1];
				jRowPtr[j][1] = mimTemplate.ptr<double>(i + 1)[j] - mimTemplate.ptr<double>(i - 1)[j];
			}
		}
	}
	mbMadeJacs = true;
};

// Calculate the zero-mean SSD between one image and the next.
// Since both are zero mean already, just calculate the SSD...
double SmallBlurryImage::ZMSSD(SmallBlurryImage &other)
{
  double dSSD = 0.0;
  for (int i = 0; i < mirSize.height; i++) {
	  for (int j = 0; j < mirSize.width; j++) {
		  double dDiff = this->mimTemplate.ptr<double>(i)[j] 
			           - other.mimTemplate.ptr<double>(i)[j];
		  dSSD += dDiff * dDiff;
	  }
  }
  return dSSD;
}


// Find an SE2 which best aligns an SBI to a target
// Do this by ESM-tracking a la Benhimane & Malis
std::pair<RigidTransforms::SE2<>, double> SmallBlurryImage::IteratePosRelToTarget(SmallBlurryImage &other, int nIterations)
{
	RigidTransforms::SE2<> se2CtoC;
	RigidTransforms::SE2<> se2WfromC;

	cv::Point irCenter(mirSize.width / 2, mirSize.height / 2);
	se2WfromC.get_translation() = cv::Vec2f(irCenter.x, irCenter.y);

	std::pair<RigidTransforms::SE2<>, double> result_pair;
	
	if (!other.mbMadeJacs)
	{
		std::cerr << "You spanner, you didn't make the jacs for the target." << endl;
		assert(other.mbMadeJacs);
	};

	double dMeanOffset = 0.0;
	cv::Vec4d v4Accum;

	cv::Vec<double, 10> v10Triangle;
	cv::Mat_<double> imWarped(mirSize);

	double dFinalScore = 0.0;
	for (int it = 0; it < nIterations; it++)
	{
		dFinalScore = 0.0;
		v4Accum = cv::Vec4d::all(0);
		v10Triangle = cv::Vec<double, 10>::all(0); // Holds the bottom-left triangle of JTJ
		cv::Vec4d v4Jac;
		v4Jac[3] = 1.0;

		RigidTransforms::SE2<> se2XForm = se2WfromC * se2CtoC * se2WfromC.inverse();

		// Make the warped current image template:
		cv::Vec2d v2Zero(0, 0);
		CvUtils::transform<>(mimTemplate,
			imWarped,
			se2XForm.get_rotation().get_matrix(),
			se2XForm.get_translation(),
			v2Zero, -9e20f * 1.0);


		// Now compare images, calc differences, and current image jacobian:
		for (int i = 0; i < mirSize.height; i++) {
			for (int j = 0; j < mirSize.width; j++) {
				if (i == 0 && j == 0 && i == mirSize.height - 1 && j == mirSize.width - 1)
					continue;
				double l, r, u, d, here;
				l = imWarped.ptr<double>(i)[j - 1];
				r = imWarped.ptr<double>(i)[j + 1];
				u = imWarped.ptr<double>(i - 1)[j];
				d = imWarped.ptr<double>(i + 1)[j];
				here = imWarped.ptr<double>(i)[j];

				if ((l + r + u + d + here) < -9999.9) continue;

				cv::Vec2d v2CurrentGrad(r - l, d - u);
				cv::Vec2d v2SumGrad = 0.25 * (v2CurrentGrad +
					other.mimImageJacs.ptr<cv::Vec2d>(i)[j]);

				v4Jac[0] = v2SumGrad[0];
				v4Jac[1] = v2SumGrad[1];
				v4Jac[2] = -(i - irCenter.y) * v2SumGrad[0] + (j - irCenter.x) * v2SumGrad[1];
				v4Jac[3] = 1.0;

				double dDiff = here - other.mimTemplate.ptr<double>(i)[j] + dMeanOffset;
				dFinalScore += dDiff * dDiff;
				v4Accum += dDiff * v4Jac;

				// Speedy fill of the LL triangle of JTJ:
				double *p = &v10Triangle[0];
				*p++ += v4Jac[0] * v4Jac[0]; // [0]
				*p++ += v4Jac[1] * v4Jac[0]; // [1]
				*p++ += v4Jac[1] * v4Jac[1]; // [2]
				*p++ += v4Jac[2] * v4Jac[0]; // [3]
				*p++ += v4Jac[2] * v4Jac[1]; // [4]
				*p++ += v4Jac[2] * v4Jac[2]; // [5]
				*p++ += v4Jac[0];            // [6] 
				*p++ += v4Jac[1];            // [7]
				*p++ += v4Jac[2];            // [8]
				*p++ += 1.0;                 // [9]  
			}
		}

		cv::Vec4d v4Update;

		// Solve for JTJ-1JTv;
		cv::Matx<double, 4, 4> m4;

		m4(0, 0) = v10Triangle[0];
		m4(1, 0) = m4(0, 1) = v10Triangle[1]; m4(1, 1) = v10Triangle[4]; m4(1, 2) = m4(2, 1) = v10Triangle[5]; m4(1, 3) = m4(3, 1) = v10Triangle[6];
		m4(2, 0) = m4(0, 2) = v10Triangle[2]; m4(2, 2) = v10Triangle[7]; m4(2, 3) = m4(3, 2) = v10Triangle[8];
		m4(3, 0) = m4(0, 3) = v10Triangle[3]; m4(3, 3) = v10Triangle[9];

		RigidTransforms::SE2<> se2Update;
		se2Update.get_translation() = cv::Vec2d(-v4Update[0], -v4Update[1]);
		se2Update.get_rotation() = RigidTransforms::SO2<>::exp(-v4Update[2]);
		se2CtoC = se2CtoC * se2Update;
		dMeanOffset -= v4Update[3];
	}

	result_pair.first = se2CtoC;
	result_pair.second = dFinalScore;
	return result_pair;
}


// What is the 3D camera rotation (zero trans) SE3<> which causes an
// input image SO2 rotation?
RigidTransforms::SE3<> SmallBlurryImage::SE3fromSE2(RigidTransforms::SE2<> se2, ATANCamera camera)
{
	// Do this by projecting two points, and then iterating the SE3<> (SO3
	// actually) until convergence. It might seem stupid doing this so
	// precisely when the whole SE2-finding is one big hack, but hey.

	camera.SetImageSize(mirSize);

	cv::Vec2d av2Turned[2];   // Our two warped points in pixels

	cv::Matx<double, 2, 2> m2R = se2.get_rotation().get_matrix();
	cv::Vec<double, 2> v2t = se2.get_translation();

	av2Turned[0] = cv::Vec2d(mirSize.width / 2 + m2R(0, 0) * 5 + v2t[0],
		                     mirSize.height / 2 + m2R(1, 0) * 5 + v2t[1]);

	av2Turned[1] = cv::Vec2d(mirSize.width / 2 + m2R(0, 0)  * (-5) + v2t[0],
		                     mirSize.height / 2 + m2R(1, 0) * (-5) + v2t[1]);

	cv::Vec3d av3OrigPoints[2];   // 3D versions of these points.
	av3OrigPoints[0] = CvUtils::backproject(camera.UnProject(mirSize.width / 2 + 5, mirSize.height / 2));
	av3OrigPoints[1] = CvUtils::backproject(camera.UnProject(mirSize.width / 2 - 5, mirSize.height / 2));

	RigidTransforms::SO3<> so3;
	
	for (int it = 0; it < 3; it++)
	{

		// simply inline least squares here. No need to use WLS
		cv::Matx<double, 3, 3> m3Omega = 10.0 * cv::Matx<double, 3, 3>::eye(); // information matrix. initialize with 10*I3
		cv::Vec<double, 3> v3ksi(0, 0, 0); // information vector

		for (int i = 0; i < 2; i++)
		{
			// Project into the image to find error
			cv::Vec3d v3Cam = so3 * av3OrigPoints[i];
			cv::Vec2d v2Implane = CvUtils::pproject(v3Cam);
			cv::Vec2d v2Pixels = camera.Project(v2Implane);
			cv::Vec2d v2Error = av2Turned[i] - v2Pixels;

			cv::Matx<double, 2, 2> m2CamDerivs = camera.GetProjectionDerivs();
			cv::Matx<double, 2, 3> m23Jacobian;
			double dOneOverCameraZ = 1.0 / v3Cam[2];
			for (int m = 0; m < 3; m++)
			{
				const cv::Vec3d v3Motion = RigidTransforms::SO3<>::generator_field(m, v3Cam);
				cv::Vec2d v2CamFrameMotion;
				v2CamFrameMotion[0] = (v3Motion[0] - v3Cam[0] * v3Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
				v2CamFrameMotion[1] = (v3Motion[1] - v3Cam[1] * v3Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
				m23Jacobian(0, m) = m2CamDerivs(0, 0) * v2CamFrameMotion[0]
					+ m2CamDerivs(0, 1) * v2CamFrameMotion[1];
				m23Jacobian(1, m) = m2CamDerivs(1, 0) * v2CamFrameMotion[0]
					+ m2CamDerivs(1, 1) * v2CamFrameMotion[1];
			}

			// 1. updating the information matrix (m3Omega)
			m3Omega += 1.0 * m23Jacobian.t() * m23Jacobian;
			// 2. updating the information vector
			v3ksi += 1.0 * m23Jacobian.t() * v2Error;

		};

		// solve the linear system
		cv::Vec<double, 3> v3Res;
		cv::solve(m3Omega, v3ksi, v3Res, cv::DECOMP_CHOLESKY);
		so3 = RigidTransforms::SO3<>::exp(v3Res) * so3;
	};

	RigidTransforms::SE3<> se3Result;
	se3Result.get_rotation() = so3;
	return se3Result;
}
