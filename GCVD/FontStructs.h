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

namespace GLXInterface {

struct Point {
    float x,y;
};

struct Font {
    typedef unsigned short int Index;

    struct Char {
        Index vertexOffset;
        Index triangleOffset;
        Index outlineOffset;
        GLsizei numTriangles;
        GLsizei numOutlines;
        float advance;
    };

    Point * vertices;
    Index * triangles;
    Index * outlines;
    Char * characters;
    string glyphs;

    const Char * findChar( const char c ) const {
        size_t ind = glyphs.find(c);
        if(ind == string::npos)
            return NULL;
        return characters + ind;
    }

    float getAdvance( const char c ) const {
        const Char * ch = findChar(c);
        if(!ch)
            return 0;
        return ch->advance;
    }

    void fill( const char c ) const {
        const Char * ch = findChar(c);
        if(!ch || !ch->numTriangles)
            return;
        glVertexPointer(2, GL_FLOAT, 0, vertices + ch->vertexOffset);
        glDrawElements(GL_TRIANGLES, ch->numTriangles, GL_UNSIGNED_SHORT, triangles + ch->triangleOffset);
    }

    void outline( const char c ) const {
        const Char * ch = findChar(c);
        if(!ch || !ch->numOutlines)
            return;
        glVertexPointer(2, GL_FLOAT, 0, vertices + ch->vertexOffset);
        glDrawElements(GL_LINES, ch->numOutlines, GL_UNSIGNED_SHORT, outlines + ch->outlineOffset);
    }

    void draw( const char c ) const {
        const Char * ch = findChar(c);
        if(!ch || !ch->numTriangles || !ch->numOutlines)
            return;
        glVertexPointer(2, GL_FLOAT, 0, vertices + ch->vertexOffset);
        glDrawElements(GL_TRIANGLES, ch->numTriangles, GL_UNSIGNED_SHORT, triangles + ch->triangleOffset);
        glDrawElements(GL_LINES, ch->numOutlines, GL_UNSIGNED_SHORT, outlines + ch->outlineOffset);
	
      
    }
};

}
