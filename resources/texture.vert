#version 330

layout (location=0) in vec3 position;
layout (location=1) in vec3 inColor;
layout (location=2) in vec2 inTexCoord;

out vec3 Color;
out vec2 texCoord

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

void main()
{
    gl_Position = projectionMatrix * viewMatrix * modelMatrix * vec4(position, 1.0);
    Color = inColor;
    texCoord = intexCoord;
}
