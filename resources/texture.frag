#version 330 core

in vec3 Color;
in vec2 texCoord;

out vec4 exColor;

uniform sampler2D img;

void main()
{
    // exColor = vec4(Color, 1.0f);
    exColor = texture(img, texCoord);
}