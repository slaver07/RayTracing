#version 460 core

in vec3 glPosition;
out vec4 FragColor;

void main()
{
    FragColor = vec4(abs(glPosition.xy), 0.0, 1.0);
}
