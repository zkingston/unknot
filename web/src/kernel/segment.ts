// Closest points between segments. C. Ericson, Real-Time Collision Detection, §5.1.9,
// with a five-candidate fallback (interior + four endpoint-to-segment) for nearly parallel pairs.

/** Parameters of the last segDist2 call: closest points are p0 + S(p1-p0), q0 + T(q1-q0). */
export const ST = new Float64Array(2);

const clamp01 = (x: number) => (x < 0 ? 0 : x > 1 ? 1 : x);
const TINY = 1e-24; // squared length below which a segment is a point

/** Squared distance between segments (a0,a1) and (b0,b1); writes params to ST. */
export function segDist2(
  ax: number, ay: number, az: number, bx: number, by: number, bz: number,
  cx: number, cy: number, cz: number, dx: number, dy: number, dz: number,
): number {
  const d1x = bx - ax, d1y = by - ay, d1z = bz - az;
  const d2x = dx - cx, d2y = dy - cy, d2z = dz - cz;
  const rx = ax - cx, ry = ay - cy, rz = az - cz;
  const a = d1x * d1x + d1y * d1y + d1z * d1z;
  const e = d2x * d2x + d2y * d2y + d2z * d2z;
  const f = d2x * rx + d2y * ry + d2z * rz;
  let s: number, t: number;
  if (a <= TINY && e <= TINY) { s = 0; t = 0; }
  else if (a <= TINY) { s = 0; t = clamp01(f / e); }
  else {
    const c = d1x * rx + d1y * ry + d1z * rz;
    if (e <= TINY) { t = 0; s = clamp01(-c / a); }
    else {
      const b = d1x * d2x + d1y * d2y + d1z * d2z;
      const denom = a * e - b * b;
      if (denom <= 1e-8 * a * e) return nearParallel(ax, ay, az, d1x, d1y, d1z, cx, cy, cz, d2x, d2y, d2z, a, e, b, c, f);
      s = clamp01((b * f - c * e) / denom);
      t = (b * s + f) / e;
      if (t < 0) { t = 0; s = clamp01(-c / a); }
      else if (t > 1) { t = 1; s = clamp01((b - c) / a); }
    }
  }
  ST[0] = s; ST[1] = t;
  const px = ax + d1x * s - cx - d2x * t, py = ay + d1y * s - cy - d2y * t, pz = az + d1z * s - cz - d2z * t;
  return px * px + py * py + pz * pz;
}

function nearParallel(
  ax: number, ay: number, az: number, d1x: number, d1y: number, d1z: number,
  cx: number, cy: number, cz: number, d2x: number, d2y: number, d2z: number,
  a: number, e: number, b: number, c: number, f: number,
): number {
  let bs = 0, bt = 0, best = Infinity;
  const cand = (s: number, t: number) => {
    const px = ax + d1x * s - cx - d2x * t, py = ay + d1y * s - cy - d2y * t, pz = az + d1z * s - cz - d2z * t;
    const q = px * px + py * py + pz * pz;
    if (q < best) { best = q; bs = s; bt = t; }
  };
  const denom = a * e - b * b;
  if (denom > 0) {
    const s = (b * f - c * e) / denom, t = (a * f - b * c) / denom;
    if (s >= 0 && s <= 1 && t >= 0 && t <= 1) cand(s, t);
  }
  cand(0, clamp01(f / e));            // a0 to segment b
  cand(1, clamp01((f + b) / e));      // a1 to segment b
  cand(clamp01(-c / a), 0);           // b0 to segment a
  cand(clamp01((b - c) / a), 1);      // b1 to segment a
  ST[0] = bs; ST[1] = bt;
  return best;
}

/** Squared distance between edge i of P and edge j of Q (indices of start vertices; wraps). */
export function edgeDist2(P: Float64Array, i: number, Q: Float64Array, j: number): number {
  const n = P.length / 3, m = Q.length / 3;
  const i1 = i + 1 === n ? 0 : i + 1, j1 = j + 1 === m ? 0 : j + 1;
  return segDist2(P[3 * i], P[3 * i + 1], P[3 * i + 2], P[3 * i1], P[3 * i1 + 1], P[3 * i1 + 2],
    Q[3 * j], Q[3 * j + 1], Q[3 * j + 2], Q[3 * j1], Q[3 * j1 + 1], Q[3 * j1 + 2]);
}
