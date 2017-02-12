#pragma once

#include "GCVD/GLWindow.h"
#include "additionalUtility.h"

using namespace additionalUtility;
using namespace GLXInterface;
class GLWindowMenu;

enum GUICommand {
	ccmd_GrabNextFrame,
	ccmd_Reset,
	ccmd_ShowNext,
	ccmd_SaveCalib,
	ccmd_Quit,
	ccmd_Optimize,
	ccmd_ShowGrabbedFrame,
	ccmd_ToggleNoDist,
	ccmd_Exit
};

class GLWindow2 : public GLWindow, public GLWindow::EventHandler
{
public:
  GLWindow2(cv::Size irSize, std::string sTitle);
  
  // The preferred event handler..
  void HandlePendingEvents();
  
  // Menu interface:
  void AddMenu(std::string sName, std::string sTitle);
  void DrawMenus();
  
  // Some OpenGL helpers:
  void SetupViewport();
  void SetupVideoOrtho();
  void SetupUnitOrtho();
  void SetupWindowOrtho();
  void SetupVideoRasterPosAndZoom();

  // Text display functions:
  void PrintString(cv::Point irPos, std::string s);
  void DrawCaption(std::string s);
  
  // Map viewer mouse interface:
  std::pair<cv::Vec<double, 6>, cv::Vec<double, 6> > GetMousePoseUpdate();
  

protected:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  
  // User interface menus:
  std::vector<GLWindowMenu*> mvpGLWindowMenus;

  cv::Size mirVideoSize;   // The size of the source video material.
  
  // Event handling routines:
  virtual void on_key_down(GLWindow&, int key);
  virtual void on_mouse_move(GLWindow& win, cv::Point where, int state);
  virtual void on_mouse_down(GLWindow& win, cv::Point where, int state, int button);
  virtual void on_event(GLWindow& win, int event);
  cv::Point mirLastMousePos;

  // Storage for map viewer updates:
  cv::Vec<double, 6> mvMCPoseUpdate;
  cv::Vec<double, 6> mvLeftPoseUpdate;

};

