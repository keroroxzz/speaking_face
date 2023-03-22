/*
    Robot Face Animation Msg Tool
    By Brian Tu
    Date : 2023/03/20
*/

#include <ros/ros.h>
#include <math.h>
#include <speaking_face/FaceParam.h>
#include <speaking_face/FaceConfig.h>
#include <speaking_face/ShapeConfig.h>

using namespace speaking_face;


ShapeConfig shape(float widht, float height, float size, float curve=0.0, float rot=0.0, float offset_x=0.0, float offset_y=0.0);
void copy(ShapeConfig &dst, const ShapeConfig &src);
ShapeConfig operator +(const ShapeConfig &a, const ShapeConfig &b);
ShapeConfig operator -(const ShapeConfig &a, const ShapeConfig &b);
ShapeConfig operator *(const ShapeConfig &a, float x);
ShapeConfig operator /(const ShapeConfig &a, float x);
bool validate(ShapeConfig &src, bool correct=true);