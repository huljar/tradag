#version 130
in float depth;

out vec4 fragColor;

void main() {
	uint udepth = uint(depth);
	fragColor = vec4(
		0,
		float((udepth & uint(0xFF00)) >> 8) / 255.0,
		float(udepth & uint(0xFF)) / 255.0,
		1
	);
}
