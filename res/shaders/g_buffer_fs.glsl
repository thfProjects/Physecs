#version 330 core

out vec4 gNormal;

in vec3 Normal;

void main()
{
    gNormal = vec4(normalize(Normal), 1.0);
}  