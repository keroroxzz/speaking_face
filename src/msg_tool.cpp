#include "msg_tool.h"

ShapeConfig shape(float widht, float height, float size, float curve, float rot, float offset_x, float offset_y){
    ShapeConfig ret;
    ret.curve = curve;
    ret.height = height;
    ret.rotation = rot;
    ret.size = size;
    ret.width = widht;
    ret.offset_x = offset_x;
    ret.offset_y = offset_y;
    return ret;
}

void copy(ShapeConfig &dst, const ShapeConfig &src){
    dst.curve = src.curve;
    dst.height = src.height;
    dst.rotation = src.rotation;
    dst.size = src.size;
    dst.width = src.width;
    dst.offset_x = src.offset_x;
    dst.offset_y = src.offset_y;
}

ShapeConfig operator +(const ShapeConfig &dst, const ShapeConfig &src){
    ShapeConfig ret;
    ret.curve = dst.curve+src.curve;
    ret.height = dst.height+src.height;
    ret.rotation = dst.rotation+src.rotation;
    ret.size = dst.size+src.size;
    ret.width = dst.width+src.width;
    ret.offset_x = dst.offset_x+src.offset_x;
    ret.offset_y = dst.offset_y+src.offset_y;
    return ret;
}
ShapeConfig operator -(const ShapeConfig &dst, const ShapeConfig &src){
    ShapeConfig ret;
    ret.curve = dst.curve+src.curve;
    ret.height = dst.height-src.height;
    ret.rotation = dst.rotation-src.rotation;
    ret.size = dst.size-src.size;
    ret.width = dst.width-src.width;
    ret.offset_x = dst.offset_x-src.offset_x;
    ret.offset_y = dst.offset_y-src.offset_y;
    return ret;
}

ShapeConfig operator *(const ShapeConfig &src, float x){
    ShapeConfig ret;
    ret.curve = x*src.curve;
    ret.height = x*src.height;
    ret.rotation = x*src.rotation;
    ret.size = x*src.size;
    ret.width = x*src.width;
    ret.offset_x = x*src.offset_x;
    ret.offset_y = x*src.offset_y;
    return ret;
}

ShapeConfig operator /(const ShapeConfig &src, float x){
    ShapeConfig ret;
    ret.curve = src.curve/x;
    ret.height = src.height/x;
    ret.rotation = src.rotation/x;
    ret.offset_x = src.offset_x/x;
    ret.offset_y = src.offset_y/x;
    return ret;
}
