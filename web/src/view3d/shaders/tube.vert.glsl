#version 300 es
// Shared by both passes: uExtrude = 0 for the fill (front faces), 1 for the outline (back faces, pushed out).
in vec3 aCenter;
in vec3 aNormal;                  // unit, or scaled by a radius factor (ball and stick)
uniform vec3 uR, uU, uF;          // view basis (right, up, toward viewer)
uniform vec2 uViewport;           // px
uniform vec3 uFrame;              // cx, cy, s: screen = (cx + s*x, cy - s*y)
uniform vec3 uKnotCentre;
uniform vec2 uDepthRange;         // min, max of p·F over the knot
uniform float uRadius;            // world units
uniform float uExtrude;
uniform vec2 uOutlinePx;          // near, far
uniform float uFlatten;           // 0 = 3D, 1 = flat on the view plane through the centre
uniform float uPersp;             // 0 = orthographic, 1 = 30° perspective
uniform float uEyeDist;           // perspective eye distance (world)
uniform float uShadow;            // 1 = drop-shadow pass: flat silhouette, offset, behind everything
uniform vec2 uShadowPx;
out float vDepthT;
out vec3 vNormal;
void main() {
  float d0 = dot(aCenter, uF);
  float t = clamp((uDepthRange.y - d0) / max(1e-6, uDepthRange.y - uDepthRange.x), 0.0, 1.0); // 0 near, 1 far
  float wpx = mix(uOutlinePx.x, uOutlinePx.y, t);
  vec3 p = aCenter + normalize(aNormal) * (uRadius * length(aNormal) + uExtrude * wpx / uFrame.z);
  // flatten: depth about the knot's centre, kept × 0.001 so the over/under order survives
  float dc = dot(p - uKnotCentre, uF);
  p -= uF * dc * (1.0 - 0.001) * max(uFlatten, uShadow);
  float x = dot(p, uR), y = dot(p, uU), d = dot(p, uF);
  vec2 sc = vec2(uFrame.x + uFrame.z * x, uFrame.y - uFrame.z * y) + uShadow * uShadowPx;
  vec3 ortho = vec3(2.0 * sc.x / uViewport.x - 1.0, 1.0 - 2.0 * sc.y / uViewport.y, -(d - dot(uKnotCentre, uF)) / 4.0);
  // perspective about the screen centre: points nearer the eye spread out
  vec2 c = uViewport * 0.5;
  float dz = d - dot(uKnotCentre, uF);
  float k = uEyeDist / max(1e-3, uEyeDist - dz);
  vec2 sp = c + (sc - c) * k;
  vec3 persp = vec3(2.0 * sp.x / uViewport.x - 1.0, 1.0 - 2.0 * sp.y / uViewport.y, ortho.z);
  vec3 pos = mix(ortho, persp, uPersp);
  if (uShadow > 0.5) pos.z = 0.999;
  gl_Position = vec4(pos, 1.0);
  vDepthT = t;
  vNormal = aNormal;
}
