#pragma once

//opengl header files
#define FREEGLUT_STATIC
#define GL_GLEXT_PROTOTYPES

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glext.h>

typedef GLvoid (*CallBack)(...);

#include <math.h>
#include <unistd.h>
#include <iostream>
#include <stdio.h>

// all right preserved by Richard S. Wright Jr.
#define MAX_SHADER_LENGTH   8192
#include <gl/shared/math3d.h>