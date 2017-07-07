// This header declares the Map class.
// This is pretty light-weight: All it contains is
// a vector of MapPoints and a vector of KeyFrames.
//
// N.b. since I don't do proper thread safety,
// everything is stored as lists of pointers,
// and map points are not erased if they are bad:
// they are moved to the trash list. That way
// old pointers which other threads are using are not 
// invalidated!

#pragma once
#include "additionalUtility.h"
using namespace additionalUtility;

#include <vector>
#include "GCVD/SE3.h"

#include <memory>

struct MapPoint;
struct KeyFrame;

struct Map
{
	Map();
	inline bool IsGood() { return bGood; }
	void Reset();

	void deleteBadPoints();
	void EmptyTrash();

	/// List of so-far good mappoints
	std::vector<std::shared_ptr<MapPoint> > vpPoints;

	// This is a list of points that were excluded from the map
	std::vector<std::shared_ptr<MapPoint> > vpPointsTrash;

	// list of keyframes
	std::vector<std::shared_ptr<KeyFrame> > vpKeyFrames;



	bool bGood;
};

