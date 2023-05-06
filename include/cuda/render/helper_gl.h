#ifndef __GL_HELPER_H__
 
#define __GL_HELPER_H__
 
#ifdef _WIN64
#define GLUT_NO_LIB_PRAGMA
#pragma comment (lib, "opengl32.lib")  /* link with Microsoft OpenGL lib */
#pragma comment (lib, "glut64.lib")    /* link with Win64 GLUT lib */
#endif //_WIN64
 
 
#ifdef _WIN32
/* On Windows, include the local copy of glut.h and glext.h */
#include "GL/glut.h"
#include "GL/glext.h"
 
#define GET_PROC_ADDRESS( str ) wglGetProcAddress( str )
 
#else
 
/* On Linux, include the system's copy of glut.h, glext.h, and glx.h */
#include <GL/glut.h>
#include <GL/glext.h>
#include <GL/glx.h>
 
#define GET_PROC_ADDRESS( str ) glXGetProcAddress( (const GLubyte *)str )
 
#endif //_WIN32
 
 
#endif //__GL_HELPER_H__'