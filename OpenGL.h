#pragma once

#ifdef _LINUX
#include <GL/gl.h>
#include <GL/glext.h>
#endif

#ifdef _OSX
#include <OpenGL/gl.h>
#include <OpenGL/glext.h>
#endif

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <GL/glew.h>
#endif
