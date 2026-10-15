// Camera = the shared View basis (R, U, F) + the 2D frame. Rotation helpers and basis slerp.
import type { View } from '../diagram/project';

type V = [number, number, number];
type Q = [number, number, number, number]; // w, x, y, z

const unit = (a: V): V => { const l = Math.hypot(a[0], a[1], a[2]) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };
const cross = (a: V, b: V): V => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

/** Rotate v about unit axis by angle (Rodrigues). */
export function rotate(v: V, axis: V, ang: number): V {
  const c = Math.cos(ang), s = Math.sin(ang), d = axis[0] * v[0] + axis[1] * v[1] + axis[2] * v[2], x = cross(axis, v);
  return [v[0] * c + x[0] * s + axis[0] * d * (1 - c), v[1] * c + x[1] * s + axis[1] * d * (1 - c), v[2] * c + x[2] * s + axis[2] * d * (1 - c)];
}

/** Turn the camera: a drag (dx right, dy down, radians) rotates the object under the cursor. */
export function orbit(v: View, dx: number, dy: number): View {
  const R = v.R as V, U = v.U as V, F = v.F as V, ang = Math.hypot(dx, dy);
  if (ang === 0) return v;
  // object rotates about the screen axis perpendicular to the drag; the camera rotates the opposite way
  const axis = unit([U[0] * dx + R[0] * dy, U[1] * dx + R[1] * dy, U[2] * dx + R[2] * dy]);
  const r = (w: V) => unit(rotate(w, axis, -ang));
  const nF = r(F), nU = r(U), nR = unit(cross(nU, nF));
  return { R: nR, U: unit(cross(nF, nR)), F: nF };
}

function toQuat(v: View): Q {
  // rows R, U, F of the world->camera rotation matrix
  const [m00, m01, m02] = v.R as V, [m10, m11, m12] = v.U as V, [m20, m21, m22] = v.F as V;
  const tr = m00 + m11 + m22;
  let q: Q;
  if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; q = [0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s]; }
  else if (m00 > m11 && m00 > m22) { const s = Math.sqrt(1 + m00 - m11 - m22) * 2; q = [(m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s]; }
  else if (m11 > m22) { const s = Math.sqrt(1 + m11 - m00 - m22) * 2; q = [(m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s]; }
  else { const s = Math.sqrt(1 + m22 - m00 - m11) * 2; q = [(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s]; }
  return q;
}
function fromQuat([w, x, y, z]: Q): View {
  return {
    R: [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
    U: [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
    F: [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
  };
}

/** Spherical interpolation between two view bases. */
export function slerpView(a: View, b: View, t: number): View {
  const qa = toQuat(a); let qb = toQuat(b);
  let d = qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3];
  if (d < 0) { qb = qb.map((x) => -x) as Q; d = -d; }
  let k0: number, k1: number;
  if (d > 0.9995) { k0 = 1 - t; k1 = t; }
  else { const th = Math.acos(d), s = Math.sin(th); k0 = Math.sin((1 - t) * th) / s; k1 = Math.sin(t * th) / s; }
  const q = qa.map((x, i) => k0 * x + k1 * qb[i]) as Q, l = Math.hypot(...q);
  return fromQuat(q.map((x) => x / l) as Q);
}

export const easeInOut = (t: number) => (t < 0.5 ? 2 * t * t : 1 - 2 * (1 - t) * (1 - t));
