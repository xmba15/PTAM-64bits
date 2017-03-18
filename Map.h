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

#include <vector>
#include "GCVD/SE3.h"
#include "additionalUtility.h"
#include <boost/shared_ptr.hpp>

struct MapPoint;
struct KeyFrame;

typedef boost::shared_ptr<MapPoint> MapPointPtr;
typedef boost::shared_ptr<KeyFrame> KeyFramePtr;

struct Map
{
  Map();
  inline bool IsGood() {return bGood;}
  void Reset();
  
  void MoveBadPointsToTrash();
  void EmptyTrash();
  
  std::vector<MapPointPtr> vpPoints;
  std::vector<MapPointPtr> vpPointsTrash;
  std::vector<KeyFramePtr> vpKeyFrames;

  bool bGood;
};
