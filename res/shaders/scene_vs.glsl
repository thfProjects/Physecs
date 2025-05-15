#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform mat3 normalWorld;
uniform mat3 normalView;

out vec3 NormalWorldSpace;
out vec3 NormalViewSpace;
out vec3 FragPosWorldSpace;
out vec3 FragPosViewSpace;
out vec2 texCoords;

void main()
{
	vec4 outPos = projection * view * model * vec4(aPos, 1.0f);
	gl_Position = outPos;
	NormalWorldSpace = normalWorld * aNormal;
	NormalViewSpace = normalView * aNormal;
	FragPosWorldSpace = vec3(model * vec4(aPos, 1.0f));
	FragPosViewSpace = vec3(view * model * vec4(aPos, 1.0f));
	texCoords = aTexCoords;
}