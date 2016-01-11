#version 130
uniform mat4 worldViewProj;
uniform vec4 texelOffsets;

in vec4 vertex;

out float depth;

void main() {
	gl_Position = worldViewProj * vertex;
	gl_Position.xy += texelOffsets.zw * gl_Position.w;

	depth = gl_Position.z;
}
