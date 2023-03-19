#version 330 core
in vec4 position;

out vec4 FragColor;

uniform vec4 eye_l; // {width, height, size, curve}
uniform vec4 eye_r;
uniform vec4 mouth;

uniform vec3 eye_l_t; // {rotation, offset x, offset y}
uniform vec3 eye_r_t;
uniform vec3 mouth_t;

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

void main(void) {

    // FragColor = vec4(position.xyz, 1.0);
    // return;ㄋ

    // left eye
    vec2 ln = translate(position.xy, eye_l_t);
    float le_field = sq(ln.x/(eye_l.x*eye_l.z));
    le_field = le_field + sq((ln.y/(eye_l.y*eye_l.z)) - eye_l.w*le_field);

    //right eye
    vec2 rn = translate(position.xy, eye_r_t);
    float re_field = sq(rn.x/(eye_r.x*eye_r.z));
    re_field = re_field + sq((rn.y/(eye_r.y*eye_r.z)) - eye_r.w*re_field);

    // mouth
    vec2 mn = translate(position.xy, mouth_t);
    float m_field = sq(mn.x/(mouth.x*mouth.z));
    m_field = m_field + sq((mn.y/(mouth.y*mouth.z)) - mouth.w*m_field);
    
    // fuse
    float c = min(min(le_field, re_field), m_field);
    float radius = 0.9;
    c = max(c, 1.0);
    c = exp((1.0-c)*60.0)+exp(-le_field/radius)+exp(-re_field/radius)+exp(-m_field/radius);

    FragColor = vec4(c*0.3, c*0.9, c*0.9, 1.0);
    return;
}
