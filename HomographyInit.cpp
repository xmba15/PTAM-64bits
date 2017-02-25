#include "HomographyInit.h"
#include <utility>
#include "GCVD/Addedutils.h"
#include "GCVD/SE3.h"
#include "MEstimator.h"

bool HomographyInit::IsHomographyInlier(cv::Matx<double, 3, 3> m3Homography, HomographyMatch match)
{
  cv::Vec2d v2Projected = CvUtils::pproject(m3Homography * CvUtils::backproject(match.v2CamPlaneFirst));
  cv::Vec2d v2Error = match.v2CamPlaneSecond - v2Projected;
  cv::Vec2d v2PixelError = match.m2PixelProjectionJac * v2Error;
  double dSquaredError = v2PixelError[0] * v2PixelError[0] + v2PixelError[1] * v2PixelError[1];
  return (dSquaredError < mdMaxPixelErrorSquared);
}

double HomographyInit::MLESACScore(cv::Matx<double, 3, 3> m3Homography, HomographyMatch match)
{
  cv::Vec2d v2Projected = CvUtils::pproject(m3Homography * CvUtils::backproject(match.v2CamPlaneFirst));
  cv::Vec2d v2Error = match.v2CamPlaneSecond - v2Projected;
  cv::Vec2d v2PixelError = match.m2PixelProjectionJac * v2Error;
  double dSquaredError = v2PixelError[0] * v2PixelError[0] + v2PixelError[1] * v2PixelError[1];
  if(dSquaredError > mdMaxPixelErrorSquared)
    return mdMaxPixelErrorSquared;
  else 
    return dSquaredError;
}

bool HomographyInit::Compute(std::vector<HomographyMatch> vMatches, double dMaxPixelError, RigidTransforms::SE3<> &se3SecondFromFirst)
{
  mdMaxPixelErrorSquared = dMaxPixelError * dMaxPixelError;
  mvMatches = vMatches;
  
  // Find best homography from minimal sets of image matches
  BestHomographyFromMatches_MLESAC();
  
  // Generate the inlier set, and refine the best estimate using this
  mvHomographyInliers.clear();
  for(unsigned int i=0; i<mvMatches.size(); i++)
    if(IsHomographyInlier(mm3BestHomography, mvMatches[i]))
      mvHomographyInliers.push_back(mvMatches[i]);
  for(int iteration = 0; iteration < 5; iteration++)
    RefineHomographyWithInliers();
  
  // Decompose the best homography into a set of possible decompositions
  DecomposeHomography();

  // At this stage should have eight decomposition options, if all went according to plan
  if(mvDecompositions.size() != 8)
    return false;
  
  // And choose the best one based on visibility constraints
  ChooseBestDecomposition();
  
  se3SecondFromFirst = mvDecompositions[0].se3SecondFromFirst;
  return true;
}

cv::Matx<double, 3, 3> HomographyInit::HomographyFromMatches(std::vector<HomographyMatch> vMatches)
{
	unsigned int nPoints = vMatches.size();
	assert(nPoints >= 4);
	cv::Matx<double, 9, 9> m9D = cv::Matx<double, 9, 9>::zeros();
	for (unsigned int n = 0; n < nPoints; n++)
	{
		double x2 = vMatches[n].v2CamPlaneSecond[0];
		double y2 = vMatches[n].v2CamPlaneSecond[1];

		double x1 = vMatches[n].v2CamPlaneFirst[0];
		double y1 = vMatches[n].v2CamPlaneFirst[1];

		// [u v]T = H [x y]T
		m9D(0, 0) += x1*x1;  m9D(0, 1) += x1*y1; m9D(0, 2) += x1;
		m9D(1, 1) += y1*y1;   m9D(1, 2) += y1;
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

	// Sow no filling-in the gaps:
	// 1. Filling the missing lower part of the upper 3x3 diagonal block and copying to the second diagonal 3x3 block
	m9D(1, 0) = m9D(4, 3) = m9D(3, 4) = m9D(0, 1);
	m9D(2, 0) = m9D(5, 3) = m9D(3, 5) = m9D(0, 2);   m9D(2, 1) = m9D(5, 4) = m9D(4, 5) = m9D(1, 2);

	// 2. the diagonal elements from 3 - 5 are the same ones from 0-2:
	m9D(3, 3) = m9D(0, 0); m9D(4, 4) = m9D(1, 1); m9D(5, 5) = m9D(2, 2);

	// 3. Now copying the last 3 columns (down-to and exluding the diagonal) to the last 3 rows...
	m9D(6, 0) = m9D(0, 6); m9D(6, 1) = m9D(1, 6); m9D(6, 2) = m9D(2, 6); m9D(6, 3) = m9D(3, 6); m9D(6, 4) = m9D(4, 6); m9D(6, 5) = m9D(5, 6);
	m9D(7, 0) = m9D(0, 7); m9D(7, 1) = m9D(1, 7); m9D(7, 2) = m9D(2, 7); m9D(7, 3) = m9D(3, 7); m9D(7, 4) = m9D(4, 7); m9D(7, 5) = m9D(5, 7); m9D(7, 6) = m9D(6, 7);
	m9D(8, 0) = m9D(0, 8); m9D(8, 1) = m9D(1, 8); m9D(8, 2) = m9D(2, 8); m9D(8, 3) = m9D(3, 8); m9D(8, 4) = m9D(4, 8); m9D(8, 5) = m9D(5, 8); m9D(8, 6) = m9D(6, 8); m9D(8, 7) = m9D(7, 8);

	// The right null-space of the matrix gives the homography...
	cv::Matx<double, 9, 9> U, Vt;
	cv::Matx<double, 9, 1> w;
	cv::SVD::compute(m9D, w, U, Vt);
	cv::Matx<float, 3, 3> m3Homography(3, 3); // the 3x3 homography
	m3Homography(0, 0) = Vt(8, 0); m3Homography(0, 1) = Vt(8, 1); m3Homography(0, 2) = Vt(8, 2);
	m3Homography(1, 0) = Vt(8, 3); m3Homography(1, 1) = Vt(8, 4); m3Homography(1, 2) = Vt(8, 5);
	m3Homography(2, 0) = Vt(8, 6); m3Homography(2, 1) = Vt(8, 7); m3Homography(2, 2) = Vt(8, 8);

	return m3Homography;
}

// Throughout the whole thing,
// SecondView = Homography * FirstView

void HomographyInit::RefineHomographyWithInliers()
{
  // identity prior 
  cv::Matx<double, 9, 9> m9Omega = cv::Matx<double, 9, 9>::eye(); // Identity for prior 
  cv::Vec<double, 9> v9ksi(0, 0, 0, 0, 0, 0, 0, 0, 0);
  
  std::vector<cv::Matx<double, 2, 9> > vmJacobians;
  std::vector<cv::Vec2d > vvErrors;
  std::vector<double> vdErrorSquared;
  double avgSqError = 0;

  for (unsigned int i = 0; i < mvHomographyInliers.size(); i++)
  {
	  // First, find error.
	  cv::Vec2d v2First = mvHomographyInliers[i].v2CamPlaneFirst;
	  cv::Vec3d v3Second = mm3BestHomography * CvUtils::backproject(v2First);
	  cv::Vec2d v2Second = CvUtils::pproject(v3Second);
	  cv::Vec2d v2Second_real = mvHomographyInliers[i].v2CamPlaneSecond;
	  cv::Vec2d v2Error = mvHomographyInliers[i].m2PixelProjectionJac * (v2Second_real - v2Second);

	  avgSqError += v2Error[0] * v2Error[0] + v2Error[1] * v2Error[1];
	  vdErrorSquared.push_back(v2Error[0] * v2Error[0] + v2Error[1] * v2Error[1]);
	  vvErrors.push_back(v2Error);

	  cv::Matx<double, 2, 9> m29Jacobian;
	  double dDenominator = v3Second[2];

	  m29Jacobian(0, 0) = v2First[0] / dDenominator;
	  m29Jacobian(0, 1) = v2First[1] / dDenominator;
	  m29Jacobian(0, 2) = 1.0 / dDenominator;

	  m29Jacobian(0, 3) = m29Jacobian(0, 4) = m29Jacobian(0, 5) = 0.0;
	  m29Jacobian(0, 6) = -v2Second[0] * v2First[0] / dDenominator;
	  m29Jacobian(0, 7) = -v2Second[0] * v2First[1] / dDenominator;
	  m29Jacobian(0, 8) = -v2Second[0] * 1.0 / dDenominator;

	  m29Jacobian(1, 0) = m29Jacobian(1, 1) = m29Jacobian(1, 2) = 0;

	  m29Jacobian(1, 3) = v2First[0] / dDenominator;
	  m29Jacobian(1, 4) = v2First[1] / dDenominator;
	  m29Jacobian(1, 5) = 1.0 / dDenominator;

	  m29Jacobian(1, 6) = -v2Second[1] * v2First[0] / dDenominator;
	  m29Jacobian(1, 7) = -v2Second[1] * v2First[1] / dDenominator;
	  m29Jacobian(1, 8) = -v2Second[1] * 1.0 / dDenominator;

	  vmJacobians.push_back(mvHomographyInliers[i].m2PixelProjectionJac * m29Jacobian);
  }
  
  // Calculate robust sigma:
  vector<double> vdd = vdErrorSquared;
  double dSigmaSquared = Tukey::FindSigmaSquared(vdd);
  
  avgSqError /= mvHomographyInliers.size();

  // Add re-weighted measurements to WLS:
  for(unsigned int i=0; i<mvHomographyInliers.size(); i++)
    {
      double dWeight = Tukey::Weight(vdErrorSquared[i], dSigmaSquared);
	  m9Omega += dWeight * (vmJacobians[i].t() * vmJacobians[i]);
	  v9ksi += dWeight * (vmJacobians[i].t() * vvErrors[i]);
  }

  cv::Matx<double, 9, 1> v9Update;
  cv::solve(m9Omega, v9ksi, v9Update, cv::DECOMP_CHOLESKY);

  cv::Matx<double, 3, 3> m3Update;          // create a 3x3 matrix by which to perturb the homography
  m3Update(0, 0) = v9Update(0, 0); m3Update(0, 1) = v9Update(1, 0); m3Update(0, 2) = v9Update(2, 0);
  m3Update(1, 0) = v9Update(3, 0); m3Update(1, 1) = v9Update(4, 0); m3Update(1, 2) = v9Update(5, 0);
  m3Update(2, 0) = v9Update(6, 0); m3Update(2, 1) = v9Update(7, 0); m3Update(2, 2) = v9Update(8, 0);

  // update homography and cut-out...
  mm3BestHomography += m3Update;
}

void HomographyInit::BestHomographyFromMatches_MLESAC()
{
	// Not many matches? Don't do ransac, throw them all in a pot and see what comes out.
	if (mvMatches.size() < 10)
	{
		mm3BestHomography = HomographyFromMatches(mvMatches);
		return;
	}

	// Enough matches? Run MLESAC.
	int anIndices[4];

	mm3BestHomography = cv::Matx<double, 3, 3>::eye();
	double dBestError = 999999999999999999.9;

	// Do 300 MLESAC trials.
	for (int nR = 0; nR < 300; nR++)
	{
		// Find set of four unique matches
		for (int i = 0; i < 4; i++)
		{
			bool isUnique = false;
			int n;
			while (!isUnique)
			{
				n = rand() % mvMatches.size();
				isUnique = true;
				for (int j = 0; j < i && isUnique; j++)
					if (anIndices[j] == n)
						isUnique = false;
			};
			anIndices[i] = n;
		}
		std::vector<HomographyMatch> vMinimalMatches;
		for (int i = 0; i < 4; i++)
			vMinimalMatches.push_back(mvMatches[anIndices[i]]);

		// Find a homography from the minimal set..
		cv::Matx<double, 3, 3> m3Homography = HomographyFromMatches(vMinimalMatches);

		//..and sum resulting MLESAC score
		double dError = 0.0;
		for (unsigned int i = 0; i < mvMatches.size(); i++)
			dError += MLESACScore(m3Homography, mvMatches[i]);

		if (dError < dBestError)
		{
			mm3BestHomography = m3Homography;
			dBestError = dError;
		}
	}
}

void HomographyInit::DecomposeHomography()
{
	mvDecompositions.clear();

	if (CvUtils::M3Det(mm3BestHomography) == 0) {
		std::cout << "Singular homography! Try again..." << std::endl;
		return;
	}

	cv::Vec3d w;
	cv::Matx<double, 3, 3> U;
	cv::Matx<double, 3, 3> Vt;

	cv::SVD::compute(mm3BestHomography, w, U, Vt);

	double d1 = w[0]; // The paper suggests the square of these (e.g. the evalues of AAT)
	double d2 = w[1]; // should be used, but this is wrong. c.f. Faugeras' book.
	double d3 = w[2];

	if (d1 - d2 < 10e-6 && d2 - d3 < 10e-6) {
		cout << "Homography is a pure rotation or you somehow transported the camera  across the room! Either way, reconstruction is not possible!" << endl;
		return;
	}

	// Now we need the sign of the determinant of V: (HtH = V*w^2*V')
	float s = CvUtils::M3Det(Vt); // This can be 1 or -1...
	Vt = s * Vt;

	// Now remove arbitrary (decomposition by-product) scale from everythying
	d1 /= d2;
	float s1 = d1*d1;
	//float s2 = 1.0; // not necessary so commenting out
	d3 /= d2;
	float s3 = d3 * d3;
	// Remove arbitrary scale from the homography. We will be needing it...
	mm3BestHomography *= 1.0 / d2;
	// now setting d2 to 1.0. Arbitrary scale has been removed
	d2 = 1.0;

	// Now, to business:

	float alphasq, betasq, alpha, beta;

	//
	// Cases #2 and #3: All singular values not equal (the most usual situation - 4 solutions)
	//		      or exactly 2 singular valuies equal (2 solutions)
	//

	// Obtaining two oposite vectors on the plane (to which n is normal)
	// See Ma, Soato, Kosecka, et al for the formulas. The idea is to obtain
	// these vectors as a linear combinations of the eigenvectors of HtH that are not 
	// mutually perpendicular to the normal. 
	// alpha and beta

	alphasq = (1 - s3) / (s1 - s3);
	betasq = (s1 - 1) / (s1 - s3);
	alpha = sqrt(alphasq);
	beta = sqrt(betasq);

	// So, if u = alpha * v1 + beta * v3 (recall v2 is mutually perpendicular to n and t)
	// then in order for Hu = R*u it must that norm(H*u) = 1; with a little pen - and-paper 
	// we get two solutions (u1 and u2) as follows:
	cv::Vec3d u1 = cv::Vec3d(alpha * Vt(0, 0) + beta * Vt(2, 0),
		alpha * Vt(0, 1) + beta * Vt(2, 1),
		alpha * Vt(0, 2) + beta * Vt(2, 2)
		);

	cv::Vec3d u2 = cv::Vec3d(alpha * Vt(0, 0) - beta * Vt(2, 0),
		alpha * Vt(0, 1) - beta * Vt(2, 1),
		alpha * Vt(0, 2) - beta * Vt(2, 2)
		);

	cv::Matx<double, 3, 3> H = mm3BestHomography; // NOT a reference; just a shorter name
												 // for the sake of shorter expressions...
												 // **** U1
	cv::Matx<double, 3, 3> U1;
	// **** W1
	cv::Matx<double, 3, 3> W1;
	// *** W2...
	cv::Matx<double, 3, 3> W2;
	// **** U2
	cv::Matx<double, 3, 3> U2;
	// if alpha or beta is zero, then the solutions we get with either U1, W1 or U2, W2 arte the same (thus, only 2),
	// so we can drop two (I arbitrarily chose to drop U1, W2 when alpha==0 and U2, W2 whene beta == 0 (could be the other way around of course...)
	if (alphasq > 10e-5) {
		U1(0, 0) = Vt(1, 0); U1(1, 0) = Vt(1, 1); U1(2, 0) = Vt(1, 2);

		// column 2: U1(:, 1) = u1
		U1(0, 1) = u1[0]; U1(1, 1) = u1[1]; U1(2, 1) = u1[2];

		// column 3: U1(:, 0) x U1(:, 1) (cross product of the above two columns)
		U1(0, 2) = -U1(2, 0) * U1(1, 1) + U1(1, 0) * U1(2, 1);
		U1(1, 2) = U1(2, 0) * U1(0, 1) - U1(0, 0) * U1(2, 1);
		U1(2, 2) = -U1(1, 0) * U1(0, 1) + U1(0, 0) * U1(1, 1);

		// B. W1
		//
		// column 1: W1(:, 0) = H * Vt(1, :)'
		//
		W1(0, 0) = H(0, 0) * Vt(1, 0) + H(0, 1) * Vt(1, 1) + H(0, 2) * Vt(1, 2);
		W1(1, 0) = H(1, 0) * Vt(1, 0) + H(1, 1) * Vt(1, 1) + H(1, 2) * Vt(1, 2);
		W1(2, 0) = H(2, 0) * Vt(1, 0) + H(2, 1) * Vt(1, 1) + H(2, 2) * Vt(1, 2);

		// column 2
		W1(0, 1) = H(0, 0) * u1[0] + H(0, 1) * u1[1] + H(0, 2) * u1[2];
		W1(1, 1) = H(1, 0) * u1[0] + H(1, 1) * u1[1] + H(1, 2) * u1[2];
		W1(2, 1) = H(2, 0) * u1[0] + H(2, 1) * u1[1] + H(2, 2) * u1[2];

		// column 3 (cross product of the above two columns)
		W1(0, 2) = -W1(2, 0) * W1(1, 1) + W1(1, 0) * W1(2, 1);
		W1(1, 2) = W1(2, 0) * W1(0, 1) - W1(0, 0) * W1(2, 1);
		W1(2, 2) = -W1(1, 0) * W1(0, 1) + W1(0, 0) * W1(1, 1);

		// 1. Homography decomposition #1
		HomographyDecomposition decomposition1;

		cv::Matx<double, 3, 3> m3R = W1 * U1.t();
		// n = v2 x u1
		decomposition1.v3n = cv::Vec3d(-Vt(1, 2) * u1[1] + Vt(1, 1) * u1[2],
			Vt(1, 2) * u1[0] - Vt(1, 0) * u1[2],
			-Vt(1, 1) * u1[0] + Vt(1, 0) * u1[1]);
		// (1/d) * t = (H - R) * n   (translation up to scale)
		cv::Vec3d v3t = (H - m3R) * decomposition1.v3n;

		// So Just copying results into the SE3 member of decomposition1...
		decomposition1.se3SecondFromFirst.get_rotation() = RigidTransforms::SO3<>(m3R);
		decomposition1.se3SecondFromFirst.get_translation() = v3t;

		// 2. Decomposition #3 (negation of #1)
		HomographyDecomposition decomposition3;
		decomposition3.v3n = -decomposition1.v3n;
		// So Just copying results into the SE3 member
		decomposition3.se3SecondFromFirst.get_rotation() = RigidTransforms::SO3<>(m3R);
		decomposition3.se3SecondFromFirst.get_translation() = -v3t;

		mvDecompositions.push_back(decomposition1);
		mvDecompositions.push_back(decomposition3);
	}

	if (betasq > 10e-5) {
		// A. U2
		//
		// column 1: U2(:, 0) = Vt(1, :)'
		U2(0, 0) = Vt(1, 0); U2(1, 0) = Vt(1, 1); U2(2, 0) = Vt(1, 2);

		// column 2: U2(:, 1) = u2
		U2(0, 1) = u2[0]; U2(1, 1) = u2[1]; U2(2, 1) = u2[2];

		// column 3: U(:, 2) = U2(:, 0) x U2(:, 1) (cross product of the above two columns)
		U2(0, 2) = -U2(2, 0) * U2(1, 1) + U2(1, 0) * U2(2, 1);
		U2(1, 2) = U2(2, 0) * U2(0, 1) - U2(0, 0) * U2(2, 1);
		U2(2, 2) = -U2(1, 0) * U2(0, 1) + U2(0, 0) * U2(1, 1);

		// B. W2
		//
		// column 1: H*v2 
		W2(0, 0) = H(0, 0) * Vt(1, 0) + H(0, 1) * Vt(1, 1) + H(0, 2) * Vt(1, 2);
		W2(1, 0) = H(1, 0) * Vt(1, 0) + H(1, 1) * Vt(1, 1) + H(1, 2) * Vt(1, 2);
		W2(2, 0) = H(2, 0) * Vt(1, 0) + H(2, 1) * Vt(1, 1) + H(2, 2) * Vt(1, 2);

		// column 2: H*u2
		W2(0, 1) = H(0, 0) * u2[0] + H(0, 1) * u2[1] + H(0, 2) * u2[2];
		W2(1, 1) = H(1, 0) * u2[0] + H(1, 1) * u2[1] + H(1, 2) * u2[2];
		W2(2, 1) = H(2, 0) * u2[0] + H(2, 1) * u2[1] + H(2, 2) * u2[2];

		// column 3: (H*v2) x (H*u2) (cross product of the above two columns)
		W2(0, 2) = -W2(2, 0) * W2(1, 1) + W2(1, 0) * W2(2, 1);
		W2(1, 2) = W2(2, 0) * W2(0, 1) - W2(0, 0) * W2(2, 1);
		W2(2, 2) = -W2(1, 0) * W2(0, 1) + W2(0, 0) * W2(1, 1);

		// 2. Decomposition #2
		HomographyDecomposition decomposition2;

		cv::Matx<double, 3, 3> m3R = W2 * U2.t();
		decomposition2.v3n = cv::Vec3d(-Vt(1, 2) * u2[1] + Vt(1, 1) * u2[2],
			Vt(1, 2) * u2[0] - Vt(1, 0) * u2[2],
			-Vt(1, 1) * u2[0] + Vt(1, 0) * u2[1]);
		//decomposition2.v3Tp = (H - decomposition2.m3Rp) * decomposition2.v3n;
		cv::Vec3d v3t = (H - m3R) * decomposition2.v3n;
		// Just copying results into the SE3 member
		decomposition2.se3SecondFromFirst.get_rotation() = RigidTransforms::SO3<>(m3R);
		decomposition2.se3SecondFromFirst.get_translation() = v3t;

		// 2. Decomposition #4 (negation of #2)
		HomographyDecomposition decomposition4;
		decomposition4.v3n = -decomposition2.v3n;
		decomposition4.se3SecondFromFirst.get_rotation() = RigidTransforms::SO3<>(decomposition2.se3SecondFromFirst.get_rotation());
		decomposition4.se3SecondFromFirst.get_translation() = -decomposition2.se3SecondFromFirst.get_translation();

		mvDecompositions.push_back(decomposition2);
		mvDecompositions.push_back(decomposition4);
	}
}

bool operator<(const HomographyDecomposition lhs, const HomographyDecomposition rhs)
{
  return lhs.nScore < rhs.nScore;
}

static double SampsonusError(cv::Vec2d &v2Dash, const cv::Matx<double, 3, 3> &m3Essential, cv::Vec2d &v2)
{
	cv::Vec3d v3Dash = CvUtils::backproject(v2Dash);
	cv::Vec3d v3 = CvUtils::backproject(v2);

	double dError = v3Dash.dot(m3Essential * v3);

	cv::Vec3d fv3 = m3Essential * v3;
	cv::Vec3d fTv3Dash = m3Essential.t() * v3Dash;

	cv::Vec2d fv3Slice(fv3[0], fv3[1]);
	cv::Vec2d fTv3DashSlice(fTv3Dash[0], fTv3Dash[1]);

	return (dError * dError / (fv3Slice.dot(fv3Slice) + fTv3DashSlice.dot(fTv3DashSlice)));
}


void HomographyInit::ChooseBestDecomposition()
{
	assert(mvDecompositions.size() > 1);
	for (unsigned int i = 0; i < mvDecompositions.size(); i++)
	{
		HomographyDecomposition &decom = mvDecompositions[i];
		int nPositive = 0;
		for (unsigned int m = 0; m < mvHomographyInliers.size(); m++)
		{
			cv::Vec2d v2 = mvHomographyInliers[m].v2CamPlaneFirst;
			double dVisibilityTest = (mm3BestHomography(2, 0) * v2[0] + mm3BestHomography(2, 1) * v2[1] + mm3BestHomography(2, 2));
			if (dVisibilityTest > 0.0)
				nPositive++;
		}
		decom.nScore = -nPositive;
	}

	std::sort(mvDecompositions.begin(), mvDecompositions.end());
	mvDecompositions.resize(4);

	for (unsigned int i = 0; i < mvDecompositions.size(); i++)
	{
		HomographyDecomposition &decom = mvDecompositions[i];
		int nPositive = 0;
		for (unsigned int m = 0; m < mvHomographyInliers.size(); m++)
		{
			cv::Vec3d v3 = CvUtils::backproject(mvHomographyInliers[m].v2CamPlaneFirst);
			double dVisibilityTest = v3.dot(decom.v3n);
			if (dVisibilityTest > 0.0)
				nPositive++;
		}
		decom.nScore = -nPositive;
	}

	std::sort(mvDecompositions.begin(), mvDecompositions.end());
	mvDecompositions.resize(2);

	// According to Faugeras and Lustman, ambiguity exists if the two scores are equal
	// but in practive, better to look at the ratio!
	double dRatio = (double)mvDecompositions[1].nScore / (double)mvDecompositions[0].nScore;

	if (dRatio < 0.9) // no ambiguity!
		mvDecompositions.erase(mvDecompositions.begin() + 1);
	else             // two-way ambiguity. Resolve by sampsonus score of all points.
	{
		double dErrorSquaredLimit = mdMaxPixelErrorSquared * 4;
		double adSampsonusScores[2];
		for (int i = 0; i < 2; i++)
		{
			RigidTransforms::SE3<> se3 = mvDecompositions[i].se3SecondFromFirst;
			cv::Matx<double, 3, 3> m3Essential;
			cv::Vec<float, 3> v3t = mvDecompositions[i].se3SecondFromFirst.get_translation();
			cv::Matx<float, 3, 3> m3R = mvDecompositions[i].se3SecondFromFirst.get_rotation().get_matrix();
			// E = [t]x * R
			for (int j = 0; j < 3; j++) {

				m3Essential(0, j) = -v3t[2] * m3R(1, j) + v3t[1] * m3R(2, j);
				m3Essential(1, j) = v3t[2] * m3R(0, j) - v3t[0] * m3R(2, j);
				m3Essential(2, j) = -v3t[1] * m3R(0, j) + v3t[0] * m3R(1, j);
			}

			double dSumError = 0;
			for (unsigned int m = 0; m < mvMatches.size(); m++)
			{
				double d = SampsonusError(mvMatches[m].v2CamPlaneSecond, m3Essential, mvMatches[m].v2CamPlaneFirst);
				if (d > dErrorSquaredLimit)
					d = dErrorSquaredLimit;
				dSumError += d;
			}

			adSampsonusScores[i] = dSumError;
		}

		if (adSampsonusScores[0] <= adSampsonusScores[1])
			mvDecompositions.erase(mvDecompositions.begin() + 1);
		else
			mvDecompositions.erase(mvDecompositions.begin());
	}
}
