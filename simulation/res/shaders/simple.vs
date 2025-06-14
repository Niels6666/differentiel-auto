#version 460 core

layout(location = 0) in vec3 position;

uniform mat4 projectionMatrix;
uniform mat4 viewMatrix;
uniform mat4 modelMatrix;

void main(void){
	gl_Position = projectionMatrix * viewMatrix * modelMatrix * vec4(position, 1.0);
}
