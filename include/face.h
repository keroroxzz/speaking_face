
#define _CRT_SECURE_NO_WARNINGS
#define FREEGLUT_STATIC

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <speaking_face/FaceParam.h>
#include <speaking_face/FaceConfig.h>
#include <speaking_face/ShapeConfig.h>
#include "msg_tool.h"

#include <stdlib.h> 
#include <pthread.h>
#include "opengl_basic.h"
#include "shader.h"
#include <chrono>

using namespace std;
using namespace chrono;
using namespace chrono::_V2;

// shaders
Shader *eye_shader;

// glwindow
M3DVector3f pmouse = { 0.0, 0.0, 1.0 };

bool full_screen;
GLdouble aspect, horizontal, vertical;
M3DMatrix44f projection, model_view, model_view_proj;
GLint windowWidth = 1024;               // window size
GLint windowHeight = 512;

GLuint facerect_vertexbuffer;
static const GLfloat face_rect_buffer_data[] = {
   10.0f, 10.0f, 0.0f,
   10.0f, -10.0f, 0.0f,
   -10.0f,  10.0f, 0.0f,
   -1.0f, -10.0f, 0.0f,
   10.0f, -10.0f, 0.0f,
   -10.0f,  10.0f, 0.0f,
};

// shape configuration
float mouth_sensitivity=0.5;
float eye_left_sensitivity=0.5;
float eye_right_sensitivity=0.5;

speaking_face::ShapeConfig eye_left = shape(1.0, 1.0, 0.3, 0.0);
speaking_face::ShapeConfig eye_right = shape(1.0, 1.0, 0.3, 0.0);
speaking_face::ShapeConfig mouth_shape = shape(1.0, 1.0, 0.2, 0.0);

speaking_face::ShapeConfig eye_left_target = shape(1.0, 1.0, 0.3, 0.0);
speaking_face::ShapeConfig eye_right_target = shape(1.0, 1.0, 0.3, 0.0);
speaking_face::ShapeConfig mouth_shape_target = shape(1.0, 1.0, 0.2, 0.0);

speaking_face::FaceParam face_param;

// int eye_status = 0;
// M3DVector4f eye_norm = {1.0, 1.0, 0.3, -0.25};
// M3DVector4f eye_wink = {1.0, 0.1, 0.3, 0.0};
// M3DVector4f eye_happy = {1.0, 0.3, 0.3, -0.6};
// M3DVector4f eye_confused = {1.0, 0.4, 0.15, -0.2};

// duration<int, std::micro> switch_count = microseconds(100);
// system_clock::time_point last_switch = system_clock::now();

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
// void mouth_shape_cb(const std_msgs::Float32MultiArray::ConstPtr msg);