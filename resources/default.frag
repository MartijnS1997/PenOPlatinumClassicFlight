#version 330 core

in vec3 Color;

out vec4 exColor;

void main()
{
    exColor = vec4(Color, 1.0f);
}