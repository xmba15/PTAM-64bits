#include "KeyFrame.h"
#include "ShiTomasi.h"
#include "SmallBlurryImage.h"
#include "FAST/prototypes.h"
#include "FAST/fast_corner.h"
#include "GCVD/Addedutils.h"

using namespace std;

void KeyFrame::MakeKeyFrame_Lite(cv::Mat_<uchar> &im)
{
  // Perpares a Keyframe from an image. Generates pyramid levels, does FAST detection, etc.
  // Does not fully populate the keyframe struct, but only does the bits needed for the tracker;
  // e.g. does not perform FAST nonmax suppression. Things like that which are needed by the 
  // mapmaker but not the tracker go in MakeKeyFrame_Rest();
  
  // First, copy out the image data to the pyramid's zero level.
  im.copyTo(aLevels[0].im);

  // Then, for each level...
  for (int i = 0; i < LEVELS; i++)
  {
	  Level &lev = aLevels[i];
	  if (i != 0)
	  {  // .. make a half-size image from the previous level..
		  CvUtils::halfSample(aLevels[i - 1].im, lev.im);
	  }

	  // .. and detect and store FAST corner points.
	  // I use a different threshold on each level; this is a bit of a hack
	  // whose aim is to balance the different levels' relative feature densities.
	  lev.vCorners.clear();
	  lev.vCandidates.clear();
	  lev.vMaxCorners.clear();

	  if (i == 0)
		  FAST::fast_corner_detect_plain_10(lev.im, lev.vCorners, 10);
	  if (i == 1)
		  FAST::fast_corner_detect_plain_10(lev.im, lev.vCorners, 15);
	  if (i == 2)
		  FAST::fast_corner_detect_plain_10(lev.im, lev.vCorners, 15);
	  if (i == 3)
		  FAST::fast_corner_detect_10(lev.im, lev.vCorners, 10);

	  // Generate row look-up-table for the FAST corner points: this speeds up 
	  // finding close-by corner points later on.
	  unsigned int v = 0;
	  lev.vCornerRowLUT.clear();
	  for (int y = 0; y < lev.im.rows; y++)
	  {
		  while (v < lev.vCorners.size() && y > lev.vCorners[v].y)
			  v++;
		  lev.vCornerRowLUT.push_back(v);
	  }
  }
}

void KeyFrame::MakeKeyFrame_Rest()
{
	// Fills the rest of the keyframe structure needed by the mapmaker:
	// FAST nonmax suppression, generation of the list of candidates for further map points,
	// creation of the relocaliser's SmallBlurryImage.
	static Persistence::pvar3<double> pvdCandidateMinSTScore("MapMaker.CandidateMinShiTomasiScore", 70, Persistence::SILENT);

	// For each level...
	for (int l = 0; l < LEVELS; l++)
	{
		Level &lev = aLevels[l];
		// .. find those FAST corners which are maximal..
		FAST::fast_nonmax(lev.im, lev.vCorners, 10, lev.vMaxCorners);
		// .. and then calculate the Shi-Tomasi scores of those, and keep the ones with
		// a suitably high score as Candidates, i.e. points which the mapmaker will attempt
		// to make new map points out of.
		for (std::vector<cv::Point>::iterator i = lev.vMaxCorners.begin(); i != lev.vMaxCorners.end(); i++)
		{
			if (!CvUtils::in_image_with_border(i->y, i->x, lev.im, 10, 10))
				continue;
			double dSTScore = FindShiTomasiScoreAtPoint(lev.im, 3, *i);
			if (dSTScore > *pvdCandidateMinSTScore)
			{
				Candidate c;
				c.irLevelPos = *i;
				c.dSTScore = dSTScore;
				lev.vCandidates.push_back(c);
			}
		}
	}

	// Also, make a SmallBlurryImage of the keyframe: The relocaliser uses these.
	pSBI = new SmallBlurryImage(*this);
	// Relocaliser also wants the jacobians..
	pSBI->MakeJacs();
}

// The keyframe struct is quite happy with default operator=, but Level needs its own
// to override CVD's reference-counting behaviour.
Level& Level::operator=(const Level &rhs)
{
  // Operator= should physically copy pixels, not use CVD's reference-counting image copy.
  //im.resize(rhs.im.size());
  rhs.im.copyTo(this->im);
  
  this->vCorners = rhs.vCorners;
  this->vMaxCorners = rhs.vMaxCorners;
  this->vCornerRowLUT = rhs.vCornerRowLUT;
  return *this;
}

// -------------------------------------------------------------
// Some useful globals defined in LevelHelpers.h live here:
cv::Vec3d gavLevelColors[LEVELS];

// These globals are filled in here. A single static instance of this struct is run before main()
struct LevelHelpersFiller // Code which should be initialised on init goes here; this runs before main()
{
	LevelHelpersFiller()
	{
		for (int i = 0; i < LEVELS; i++)
		{
			if (i == 0)  gavLevelColors[i] = cv::Vec3d(1.0, 0.0, 0.0);
			else if (i == 1)  gavLevelColors[i] = cv::Vec3d(1.0, 1.0, 0.0);
			else if (i == 2)  gavLevelColors[i] = cv::Vec3d(0.0, 1.0, 0.0);
			else if (i == 3)  gavLevelColors[i] = cv::Vec3d(0.0, 0.0, 0.7);
			else gavLevelColors[i] = cv::Vec3d(1.0, 1.0, 0.7); // In case I ever run with LEVELS > 4
		}
	}
};

static LevelHelpersFiller foo;
