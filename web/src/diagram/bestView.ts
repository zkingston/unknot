// Best view: score 128 directions on a Fibonacci hemisphere by crossings, crossing angle, crossing isolation
// and projected aspect; rotate in-plane so the principal axis is horizontal.
import type { Scene } from '../kernel/scene';
import type { View } from './project';
import { projectCurves } from './project';
import { findCrossings } from './crossings';

type V = [number, number, number];
const dot = (a: V, b: V) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a: V, b: V): V => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const unit = (a: V): V => { const l = Math.sqrt(dot(a, a)) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };

/** Any orthonormal basis with F toward the viewer. */
export function basisFor(F: V, upHint: V = [0, 1, 0]): View {
  F = unit(F);
  let R = cross(upHint, F);
  if (dot(R, R) < 1e-12) R = cross([1, 0, 0], F);
  R = unit(R);
  return { R, U: cross(F, R), F };
}

export function fibonacciHemisphere(k = 128): V[] {
  const out: V[] = [], ga = Math.PI * (3 - Math.sqrt(5));
  for (let i = 0; i < k; i++) {
    const z = 1 - (i + 0.5) / k, r = Math.sqrt(1 - z * z), a = i * ga;
    out.push([r * Math.cos(a), r * Math.sin(a), z]);
  }
  return out;
}

export interface ViewScore { c: number; theta: number; s: number; r: number; regular: boolean }

/** No tangency, no crossing within 1e-6 of a vertex, no two crossings at one point (validateDiagram's rules). */
function regular(C: Float64Array[], X: ReturnType<typeof findCrossings>, irregular: number) {
  if (irregular) return false;
  let ext = 0;
  for (const Q of C) for (let i = 0; i < Q.length; i++) ext = Math.max(ext, Math.abs(Q[i]));
  const eps = 2e-6 * ext;
  for (const x of X) for (const r of [x.a, x.b]) {
    const Q = C[r.comp], n = Q.length / 2, j = (r.seg + 1) % n, dx = Q[2 * j] - Q[2 * r.seg], dy = Q[2 * j + 1] - Q[2 * r.seg + 1], L = Math.sqrt(dx * dx + dy * dy);
    if (r.t * L < eps || (1 - r.t) * L < eps) return false;
  }
  for (let i = 0; i < X.length; i++) for (let j = i + 1; j < X.length; j++) if (Math.abs(X[i].x - X[j].x) < eps && Math.abs(X[i].y - X[j].y) < eps) return false;
  return true;
}

/** Second moments of a projection's vertices about their mean. */
function covariance(C: Float64Array[]) {
  let n = 0, mx = 0, my = 0;
  for (const Q of C) for (let i = 0; i < Q.length; i += 2) { mx += Q[i]; my += Q[i + 1]; n++; }
  mx /= n; my /= n;
  let sxx = 0, syy = 0, sxy = 0;
  for (const Q of C) for (let i = 0; i < Q.length; i += 2) { const x = Q[i] - mx, y = Q[i + 1] - my; sxx += x * x; syy += y * y; sxy += x * y; }
  return { sxx, syy, sxy };
}

export function scoreView(S: Scene, v: View): ViewScore {
  const C = projectCurves(S, v), irr: { kind: string }[] = [], X = findCrossings(C, irr as never);
  // principal axes
  const { sxx, syy, sxy } = covariance(C);
  const tr = sxx + syy, det = sxx * syy - sxy * sxy, disc = Math.sqrt(Math.max(0, tr * tr / 4 - det));
  const l1 = tr / 2 + disc, l2 = tr / 2 - disc, r = l1 > 0 ? Math.sqrt(Math.max(0, l2) / l1) : 0;
  let theta = Math.PI / 2, s = Infinity;
  for (const x of X) {
    const A = C[x.a.comp], B = C[x.b.comp], na = A.length / 2, nb = B.length / 2;
    const ia = x.a.seg, ja = (ia + 1) % na, ib = x.b.seg, jb = (ib + 1) % nb;
    const ux = A[2 * ja] - A[2 * ia], uy = A[2 * ja + 1] - A[2 * ia + 1], wx = B[2 * jb] - B[2 * ib], wy = B[2 * jb + 1] - B[2 * ib + 1];
    const cs = Math.abs(ux * wx + uy * wy) / (Math.sqrt((ux * ux + uy * uy) * (wx * wx + wy * wy)) || 1);
    theta = Math.min(theta, Math.acos(Math.min(1, cs)));
    // distance from the crossing to unrelated projected edges (not the two segments or their neighbours)
    C.forEach((Q, c) => {
      const m = Q.length / 2;
      for (let i = 0; i < m; i++) {
        const near = (c === x.a.comp && Math.min((i - ia + m) % m, (ia - i + m) % m) <= 1) || (c === x.b.comp && Math.min((i - ib + m) % m, (ib - i + m) % m) <= 1);
        if (near) continue;
        const j = (i + 1) % m, px = Q[2 * i], py = Q[2 * i + 1], dx = Q[2 * j] - px, dy = Q[2 * j + 1] - py, l2 = dx * dx + dy * dy;
        let t = l2 > 0 ? ((x.x - px) * dx + (x.y - py) * dy) / l2 : 0; t = t < 0 ? 0 : t > 1 ? 1 : t;
        const ex = px + t * dx - x.x, ey = py + t * dy - x.y;
        s = Math.min(s, Math.sqrt(ex * ex + ey * ey));
      }
    });
  }
  return { c: X.length, theta: (theta * 180) / Math.PI, s, r, regular: regular(C, X, irr.length) };
}

/** Best orthographic view, keeping the sign that faces the current camera so the image does not mirror. */
export function bestView(S: Scene, current: View): View {
  let best: { v: View; sc: ViewScore } | null = null, fallback: { v: View; sc: ViewScore } | null = null;
  const dirs = fibonacciHemisphere(128);
  // a slight tilt off the lattice planes that hand-made coordinates tend to share
  for (let k = 0; k < dirs.length; k++) { const d = dirs[k]; dirs[k] = unit([d[0] + 0.0123 * d[2], d[1] + 0.0071 * d[2], d[2]]); }
  for (let d of dirs) {
    if (dot(d, current.F as V) < 0) d = [-d[0], -d[1], -d[2]];
    const v = basisFor(d, current.U as V), sc = scoreView(S, v);
    if (!sc.regular) continue;
    if (!fallback || sc.r > fallback.sc.r) fallback = { v, sc };
    if (sc.theta >= 15 && sc.r >= 0.3 && (!best || sc.c < best.sc.c || (sc.c === best.sc.c && sc.s > best.sc.s))) best = { v, sc };
  }
  // an in-plane rotation keeps the projection regular
  return alignPrincipal(S, (best ?? fallback ?? { v: current }).v);
}

/** Rotate the view in-plane so the principal axis of the projection is horizontal. */
function alignPrincipal(S: Scene, v: View): View {
  const { sxx, syy, sxy } = covariance(projectCurves(S, v));
  const a = 0.5 * Math.atan2(2 * sxy, sxx - syy), c = Math.cos(a), s = Math.sin(a);
  const R = v.R as V, U = v.U as V;
  return { R: unit([c * R[0] + s * U[0], c * R[1] + s * U[1], c * R[2] + s * U[2]]), U: unit([-s * R[0] + c * U[0], -s * R[1] + c * U[1], -s * R[2] + c * U[2]]), F: v.F };
}
