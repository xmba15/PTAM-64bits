// Defines the MapViewer class
//
// This defines a simple map viewer widget, which can draw the 
// current map and the camera/keyframe poses within it.
//

#pragma once

#include "Map.h"
#include "GCVD/SE3.h"
#include <sstream>
#include "GLWindow2.h"
#include "additionalUtility.h"

class Map;

class MapViewer
{
public:
  MapViewer(Map &map, GLWindow2 &glw);
  void DrawMap(RigidTransforms::SE3<> se3CamFromWorld);
  std::string GetMessageForUser();
  
protected:
  Map &mMap;
  GLWindow2 &mGLWindow;
  
  void DrawGrid();
  void DrawMapDots();
  void DrawCamera(RigidTransforms::SE3<> se3, bool bSmall=false);
  void SetupFrustum();
  void SetupModelView(RigidTransforms::SE3<> se3WorldFromCurrent = RigidTransforms::SE3<>());
  
  cv::Vec3d mv3MassCenter;
  RigidTransforms::SE3<> mse3ViewerFromWorld;

  std::ostringstream mMessageForUser;
};
