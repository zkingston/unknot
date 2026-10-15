#version 300 es
precision highp float;
uniform vec3 uColor;
uniform vec3 uDark;
uniform float uShade;   // > 0 on the fill pass: shade toward uDark away from the light
uniform vec3 uLight;    // world-space light direction (unit)
in float vDepthT;
in vec3 vNormal;
out vec4 outColor;
void main() {
  float lit = max(0.0, dot(normalize(vNormal), uLight));
  // soft two-tone shading: lit side stays paper, the far side darkens a little
  float k = uShade * (1.0 - smoothstep(0.0, 0.9, lit));
  outColor = vec4(mix(uColor, uDark, k), 1.0);
}
