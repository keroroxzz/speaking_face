/*
    Robot Face Animation for ROS
    Header file of the main program
    By Brian Tu
    Date : 2023/03/20
*/

#define _CRT_SECURE_NO_WARNINGS
#define FREEGLUT_STATIC

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Float32MultiArray.h>
#include <speaking_face/FaceParam.h>
#include <speaking_face/FaceConfig.h>
#include <speaking_face/ShapeConfig.h>

#include <stdlib.h> 
#include <pthread.h>
#include <chrono>

#include "opengl_basic.h"
#include "msg_tool.h"
#include "shader.h"

using namespace std;
using namespace chrono;
using namespace chrono::_V2;

//pkg path
std::string path = ros::package::getPath("speaking_face");

// shaders
Shader *eye_shader;

// glwindow
bool full_screen;
M3DVector3f pmouse = { 0.0, 0.0, 1.0 };
GLdouble aspect, horizontal, vertical;
M3DMatrix44f projection, model_view, model_view_proj;
GLint windowWidth = 1024;               // window size
GLint windowHeight = 512;

// rendering rect
GLuint facerect_vertexbuffer;
static const GLfloat face_rect_buffer_data[] = {
   10.0f, 10.0f, 0.0f,
   10.0f, -10.0f, 0.0f,
   -10.0f,  10.0f, 0.0f,
   -1.0f, -10.0f, 0.0f,
   10.0f, -10.0f, 0.0f,
   -10.0f,  10.0f, 0.0f,
};

// face configuration
speaking_face::ShapeConfig heading = shape(0.0, 0.0, 0.0, 0.0);
speaking_face::ShapeConfig eye_left = shape(1.0, 1.0, 0.3, 0.0);
speaking_face::ShapeConfig eye_right = shape(1.0, 1.0, 0.3, 0.0);
speaking_face::ShapeConfig mouth_shape = shape(1.0, 1.0, 0.2, 0.0);

speaking_face::ShapeConfig heading_target = shape(0.0, 0.0, 0.0, 0.0);
speaking_face::ShapeConfig eye_left_target = shape(1.0, 1.0, 0.3, 0.0);
speaking_face::ShapeConfig eye_right_target = shape(1.0, 1.0, 0.3, 0.0);
speaking_face::ShapeConfig mouth_shape_target = shape(1.0, 1.0, 0.2, 0.0);

speaking_face::FaceParam face_param;

// high resolution clock for performace measurement
// #define MEASURE_TIME
chrono::high_resolution_clock::time_point pclock = chrono::high_resolution_clock::now();

//funtions
void SetupRC();
void RenderScene(void);
void KeyPressFunc(unsigned char key, int x, int y);
void ReloadShaders();
void SpecialKeys(int key, int x, int y);
void ChangeSize(int w, int h);
void idle();
void mousePressed(int x_, int y_);
void mouseHover(int x_, int y_);
void mouse(int button, int state, int x, int y);