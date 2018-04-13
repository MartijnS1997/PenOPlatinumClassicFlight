#version 330

layout (location=0) in vec3 position;
layout (location=1) in vec3 inColor;

out vec3 Color;

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

void main()
{
    gl_Position = projectionMatrix * viewMatrix * modelMatrix * vec4(position, 1.0);
    Color = inColor;
}
