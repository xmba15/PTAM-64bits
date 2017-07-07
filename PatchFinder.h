// This header declares the PatchFinder class.
// This is quite a low-level class.
// 
// The purpose of the PatchFinder is to find a map point in a new view. 
// This proceeds in multiple stages:
// 1. A warping matrix appropriate for the current view is calculated,
// 2. A `coarse' matching template of the map point is generated by warping;
// 3. The new view is searched for this template at corner locations;
// 4. An inverse-composition (`fine') matching template is generated;
// 5. A sub-pixel accurate position is calculated using inverse composition.
//
// To clarify points 1 and 2 above: Each map point does _not_ store a `patch' 
// in the style of many other SLAM implementations! Each map point came from 
// a key-frame, and that key-frame is stored in the map. Each map point only
// stores the information on where it came from in what key-frame. Patches 
// are then generated from the pixels of the source key-frame. According to the
// relative camera poses now and of the source key-frame, a warp has to be 
// applied to generate an NxN pixel square template appropriate for the current 
// view. Thus, the matching template is always NxN, but the region of source 
// pixels used can change size and shape! This is all described in the 
// Klein & Murray ISMAR 2007.
//
// Most of the above stages are optional, and there are different versions
// of stages 1/2, including some for operating when there is not yet a 3D map 
// point. The class provides no safety checks to ensure that the operations
// are carried out in the right order - it's the caller's responsibility.
//
// The patch finder uses zero-mean SSD as its difference metric.
//
// Although PatchFinder can use arbitrary-sized search templates (it's determined
// at construction), the use of 8x8 pixel templates (the default) is highly 
// recommended, as the coarse search for this size is SSE-optimised.

#pragma once
#include "additionalUtility.h"
using namespace additionalUtility;

#include "GCVD/SE3.h"
#include "MapPoint.h"
#include "LevelHelpers.h"

using namespace RigidTransforms;

class PatchFinder
{

public:
	// Constructor defines size of search patch.
	PatchFinder(int nPatchSize = 8);

	// Step 1 Function.
	// This calculates the warping matrix appropriate for observing point p
	// from the current view (described as an SE3.) This also needs the camera
	// projection derivates at level zero for the point's projection in the new view.
	// It also calculates which pyramid level we should search in, and this is
	// returned as an int. Negative level returned denotes an inappropriate 
	// transformation.
	int CalcSearchLevelAndWarpMatrix(MapPoint::Ptr p, SE3<> se3CFromW, cv::Matx<float, 2, 2> &m2CamDerivs);
	inline int GetLevel() { return mnSearchLevel; }
	inline int GetLevelScale() { return LevelScale(mnSearchLevel); }

	// Step 2 Functions
	// Generates the NxN search template either from the pre-calculated warping matrix,
	// or an identity transformation.
	void MakeTemplateCoarseCont(MapPoint::Ptr p); // If the warping matrix has already been pre-calced, use this.
	void MakeTemplateCoarse(MapPoint::Ptr p, SE3<> se3CFromW, cv::Matx<float, 2, 2> &m2CamDerivs); // This also calculates the warp.
	void MakeTemplateCoarseNoWarp(MapPoint::Ptr p);  // Identity warp: just copies pixels from the source KF.
	void MakeTemplateCoarseNoWarp(std::shared_ptr<KeyFrame> k, int nLevel, cv::Point2i irLevelPos); // Identity warp if no MapPoint struct exists yet.

																									// If the template making failed (i.e. it needed pixels outside the source image),
																									// this bool will return false.
	inline bool TemplateBad() { return mbTemplateBad; }

	// Step 3 Functions
	// This is the raison d'etre of the class: finds the patch in the current input view,
	// centered around ir, Searching around FAST corner locations only within a radius nRange only.
	// Inputs are given in level-zero coordinates! Returns true if the patch was found.

	bool FindPatchCoarse(cv::Vec<float, 2> v2Pos, std::shared_ptr<KeyFrame> pKF, unsigned int nRange);
	int ZMSSDAtPoint(const cv::Mat_<uchar> &im, const cv::Point2i &ir); // This evaluates the score at one location
																		// Results from step 3:
																		// All positions are in the scale of level 0.
	inline cv::Point2i GetCoarsePos() { return cv::Point2i((int)mv2CoarsePos[0], (int)mv2CoarsePos[1]); }
	inline cv::Vec2f GetCoarsePosAsVector() { return mv2CoarsePos; }

	// Step 4
	void prepSubPixGNStep();  // This is the former "MakeSubPixTemplate". It precomputes the
							  // cumulative gram-matrix of the level image gradients in the patch.
							  // Just forget the original comments if you ever actually read them!!!!

							  // Step 5 Functions
	bool IterateSubPixToConvergence(std::shared_ptr<KeyFrame> kf, int nMaxIts);  // Run inverse composition till convergence
	double IterateSubPix(std::shared_ptr<KeyFrame> kf);     // Single iteration of IC. Returns sum-squared pixel update dist, or negative if out of imag
	inline cv::Vec2f GetSubPixPos() { return mv2SubPixPos; }  // Get result
	void SetSubPixPos(cv::Vec2f v2) { mv2SubPixPos = v2; }  // Set starting point

															// Get the uncertainty estimate of a found patch;
															// This for just returns an appropriately-scaled identity!
	inline cv::Matx<float, 2, 2> GetCov() // returns 2x2 matrix
	{
		return LevelScale(mnSearchLevel) * cv::Matx<float, 2, 2>::eye();
	}

	int mnMaxSSD; // This is the max ZMSSD for a valid match. It's set in the constructor.

protected:

	int mnPatchSize; // Size of one side of the matching template.

					 // Some values stored for the coarse template:
	int mnTemplateSum;    // Cached pixel-sum of the coarse template
	int mnTemplateSumSq;  // Cached pixel-squared sum of the coarse template
	inline void MakeTemplateSums(); // Calculate above values

	cv::Mat_<uchar> mimTemplate;   // The matching template
	cv::Mat_<cv::Vec<float, 3> > mimJacs;  // Inverse composition jacobians; stored as floats to save a bit of space.

	cv::Matx<float, 2, 2> mm2WarpInverse;   // 2x2 Warping matrix
	int mnSearchLevel;                // Search level in input pyramid
	cv::Matx<float, 3, 3> mm3JtJInv;          // 3x3 Inverse composition JtJ^-1

	cv::Vec2f mv2SubPixPos;     // In the scale of level 0
	double mdMeanDiff;          // Updated during inverse composition

	cv::Vec<float, 2> mv2PredictedPos;  // Search center location of FindPatchCoarse in L0
										// This member had no use in the original PTAM. 
										// I am using it as cache inside "FindPatchCoarse" for the actual predicted position.
	cv::Vec2f mv2CoarsePos;     // In the scale of level 0; hence the use of vector rather than cv::Point2i
	cv::Point2i mirCenter;    // Quantized location of the center pixel of the NxN pixel template
	bool mbFound;               // Was the patch found?
	bool mbTemplateBad;         // Error during template generation?

								// Some cached values to avoid duplicating work if the camera is stopped:
	MapPoint::Ptr mpLastTemplateMapPoint;  // Which was the last map point this PatchFinder used?
	cv::Matx<float, 2, 2> mm2LastWarpMatrix;       // What was the last 2x2 warp matrix this PatchFinder used?
};
