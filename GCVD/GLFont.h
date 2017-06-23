#ifndef GLFONT_H
#define GLFONT_H

#include <map>

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

#include "GLHelpers.h"

#include "FontStructs.h"

#include "sans.h"
#include "mono.h"
#include "serif.h"

namespace GLXInterface {

 

struct FontData {

    typedef map<string,Font *> FontMap;

    FontData() {
        fonts["sans"] = &sans_font;
        fonts["mono"] = &mono_font;
        fonts["serif"] = &serif_font;
        GLXInterface::glSetFont("sans");
	
    }
    inline Font * currentFont(){
        return fonts[currentFontName];
    }

    string currentFontName;
    FontMap fonts;
};

static struct FontData data;


} // end namespace GLXInterface


#endif