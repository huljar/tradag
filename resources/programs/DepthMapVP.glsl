#version 130
uniform mat4 worldView;
uniform mat4 projection;
uniform vec4 texelOffsets;

in vec4 vertex;

out float depth;

void main() {
	vec4 cs_Position = worldView * vertex;
	depth = -cs_Position.z;

	gl_Position = projection * cs_Position;
	gl_Position.xy += texelOffsets.zw * gl_Position.w;
}
