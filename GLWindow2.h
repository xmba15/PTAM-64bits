// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
#ifndef __GL_WINDOW_2_H
#define __GL_WINDOW_2_H
//
//  A class which wraps a CVD::GLWindow and provides some basic
//  user-interface funtionality: A gvars-driven clickable menu, and a
//  caption line for text display. Also provides some handy GL helpers
//  and a wrapper for CVD's text display routines.

#if !_WIN64
#include <cvd/glwindow.h>
#endif
#include <TooN/TooN.h>
#include "additionalUtility.h"

class GLWindowMenu;


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
  std::pair<TooN::Vector<6>, TooN::Vector<6> > GetMousePoseUpdate();
  

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
  TooN::Vector<6> mvMCPoseUpdate;
  TooN::Vector<6> mvLeftPoseUpdate;
  

};


#endif
