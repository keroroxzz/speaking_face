/*
    Robot Face Animation for ROS
    By Brian Tu
    Date : 2023/03/20
*/

#version 330 core
in vec4 position;

out vec4 FragColor;

uniform vec4 eye_l; // {width, height, size, curve}
uniform vec4 eye_r;
uniform vec4 mouth;

uniform vec3 eye_l_t; // {rotation, offset x, offset y}
uniform vec3 eye_r_t;
uniform vec3 mouth_t;

uniform float eye_distance;

float sq(float x)
{
    return x*x;
}

vec2 translate(vec2 nc, vec3 tf){
    vec2 ret;
    nc -= tf.yz;
    ret.x = nc.x*cos(tf.x) - nc.y*sin(tf.x);
    ret.y = nc.x*sin(tf.x) + nc.y*cos(tf.x);

    return ret;
}

float shapeField(vec2 position, vec3 tf, vec3 offset, vec4 config){
    
    vec2 ln = translate(position.xy, tf+offset);
    float le_field = sq(ln.x/(config.x*config.z));
    return le_field + sq((ln.y/(config.y*config.z)) - config.w*le_field);
}

void main(void) {

    float le_field = shapeField(position.xy, eye_l_t, -vec3(0.0, eye_distance, 0.0), eye_l);
    float re_field = shapeField(position.xy, eye_r_t, vec3(0.0, eye_distance, 0.0), eye_r);
    float m_field = shapeField(position.xy, mouth_t, vec3(0.0, 0.0, 0.0), mouth);
    
    // fuse
    float c = min(min(le_field, re_field), m_field);
    float radius = 0.9;
    c = max(c, 1.0);
    c = exp((1.0-c)*60.0)+exp(-le_field/radius)+exp(-re_field/radius)+exp(-m_field/radius);

    FragColor = vec4(c*0.3, c*0.9, c*0.9, 1.0);
    return;
}
