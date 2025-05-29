#version 460 core

out vec4 fragColor;

layout(binding = 0) uniform sampler2D floorTexture;

in vec2 texCoords;

void main(void){
	fragColor = texture(floorTexture, texCoords * 10);
}
