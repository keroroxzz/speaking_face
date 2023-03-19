#version 330 core
layout (location = 0) in vec3 aPos;

out vec4 position;

uniform mat4 ModelViewProjMatrix;

void main(void)
{
   position = vec4(aPos, 1.0);
   gl_Position = ModelViewProjMatrix*position;
}
