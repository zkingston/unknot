// Chaikin corner cutting where every cut is a Delta move: vertex b is replaced by q and r a quarter of the way
// along its edges, removing triangle (q, b, r). Each quarter is capped at 3/4 of the other edge, so a long straight
// run next to a short edge is not swept across its neighbours. A strand that passes through the cut's triangle
// (scaled up to 1.25) shrinks it to 0.8 of the scale where it enters, which varies continuously as strands move (so
// the curve never pops), and every cut is then verified exactly, so the smoothed curve is the same knot as the
// planner's Scene.
import { segTri, CONTACT } from '../kernel/delta';
import { segDist2 } from '../kernel/segment';
import type { Scene } from '../kernel/scene';

/** Segment soup with a uniform 3D grid; segments may be appended while querying. */
class SegGrid {
  seg: number[] = []; // x0,y0,z0,x1,y1,z1 per segment
  tag: number[] = []; // owner key: comp * 2^21 + vertex, or -1 for chords
  cells = new Map<number, number[]>();
  constructor(public h: number) {}
  private key(ix: number, iy: number, iz: number) { return ((ix * 73856093) ^ (iy * 19349663) ^ (iz * 83492791)) | 0; }
  add(x0: number, y0: number, z0: number, x1: number, y1: number, z1: number, tag: number) {
    const id = this.tag.length, h = this.h;
    this.seg.push(x0, y0, z0, x1, y1, z1); this.tag.push(tag);
    const ax = Math.floor(Math.min(x0, x1) / h), bx = Math.floor(Math.max(x0, x1) / h);
    const ay = Math.floor(Math.min(y0, y1) / h), by = Math.floor(Math.max(y0, y1) / h);
    const az = Math.floor(Math.min(z0, z1) / h), bz = Math.floor(Math.max(z0, z1) / h);
    for (let i = ax; i <= bx; i++) for (let j = ay; j <= by; j++) for (let k = az; k <= bz; k++) {
      const key = this.key(i, j, k), l = this.cells.get(key);
      if (l) l.push(id); else this.cells.set(key, [id]);
    }
  }
  /** Segment ids whose cells meet the box (deduplicated via stamp). */
  query(lo: number[], hi: number[], stamp: Int32Array, mark: number, out: number[]) {
    const h = this.h;
    out.length = 0;
    for (let i = Math.floor(lo[0] / h); i <= Math.floor(hi[0] / h); i++)
      for (let j = Math.floor(lo[1] / h); j <= Math.floor(hi[1] / h); j++)
        for (let k = Math.floor(lo[2] / h); k <= Math.floor(hi[2] / h); k++) {
          const l = this.cells.get(this.key(i, j, k));
          if (l) for (const id of l) if (id < stamp.length && stamp[id] !== mark) { stamp[id] = mark; out.push(id); }
        }
    return out;
  }
}

const VBITS = 2097152; // 2^21

/** Squared distance from the origin to segment P→Q. */
function pointSeg2(px: number, py: number, pz: number, qx: number, qy: number, qz: number): number {
  const dx = qx - px, dy = qy - py, dz = qz - pz, l2 = dx * dx + dy * dy + dz * dz;
  let t = l2 > 0 ? -(px * dx + py * dy + pz * dz) / l2 : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  const x = px + t * dx, y = py + t * dy, z = pz + t * dz;
  return x * x + y * y + z * z;
}

/**
 * Smallest f at which segment s0-s1 meets the triangle (b, b + f·u, b + f·v), as an offset from b (Infinity if never):
 * where the segment crosses the triangle's plane, f = α + β for that point b + α·u + β·v (α, β >= 0). A segment
 * lying in the plane (or a degenerate corner) is measured at its nearest point to b, conservatively, and flagged.
 */
let flat = false;
function enters(x0: number, y0: number, z0: number, x1: number, y1: number, z1: number, ux: number, uy: number, uz: number, vx: number, vy: number, vz: number): number {
  const nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
  const d0 = x0 * nx + y0 * ny + z0 * nz, d1 = x1 * nx + y1 * ny + z1 * nz, nn = Math.sqrt(nx * nx + ny * ny + nz * nz);
  const scale = nn * (Math.sqrt(x0 * x0 + y0 * y0 + z0 * z0) + Math.sqrt(x1 * x1 + y1 * y1 + z1 * z1));
  const rho = Math.sqrt(Math.max(ux * ux + uy * uy + uz * uz, vx * vx + vy * vy + vz * vz));
  if (!(nn > 1e-12 * rho * rho) || (Math.abs(d0) <= 1e-12 * scale && Math.abs(d1) <= 1e-12 * scale)) { flat = true; return Math.sqrt(pointSeg2(x0, y0, z0, x1, y1, z1)) / rho; }
  if ((d0 > 0 && d1 > 0) || (d0 < 0 && d1 < 0)) return Infinity;
  const t = d0 === d1 ? 0 : d0 / (d0 - d1), wx = x0 + t * (x1 - x0), wy = y0 + t * (y1 - y0), wz = z0 + t * (z1 - z0);
  const uu = ux * ux + uy * uy + uz * uz, vv = vx * vx + vy * vy + vz * vz, uv = ux * vx + uy * vy + uz * vz;
  const wu = wx * ux + wy * uy + wz * uz, wv = wx * vx + wy * vy + wz * vz, det = uu * vv - uv * uv;
  const a = (wu * vv - wv * uv) / det, b = (wv * uu - wu * uv) / det;
  return a >= 0 && b >= 0 ? a + b : Infinity;
}

/** One Delta-checked corner-cutting iteration, all components together. */
function iterate(S: Scene, orig?: Int32Array[], next?: Int32Array[]): Scene {
  let total = 0, cnt = 0;
  for (const P of S) {
    const n = P.length / 3;
    for (let i = 0; i < n; i++) {
      const j = (i + 1) % n, dx = P[3 * j] - P[3 * i], dy = P[3 * j + 1] - P[3 * i + 1], dz = P[3 * j + 2] - P[3 * i + 2];
      total += Math.sqrt(dx * dx + dy * dy + dz * dz); cnt++;
    }
  }
  const g = new SegGrid(Math.max(1e-6, (2 * total) / Math.max(1, cnt)));
  S.forEach((P, c) => { const n = P.length / 3; for (let i = 0; i < n; i++) { const j = (i + 1) % n;
    g.add(P[3 * i], P[3 * i + 1], P[3 * i + 2], P[3 * j], P[3 * j + 1], P[3 * j + 2], c * VBITS + i); } });
  const stamp = new Int32Array(cnt * 2 + 16), ids: number[] = [], lo = [0, 0, 0], hi = [0, 0, 0];
  let mark = 0;
  const c2 = CONTACT * CONTACT;
  return S.map((P, c) => {
    const n = P.length / 3, out: number[] = [], from: number[] = [];
    if (n < 3) { if (next && orig) next[c] = orig[c].slice(); return P.slice(); }
    for (let b = 0; b < n; b++) {
      const a = (b + n - 1) % n, cc = (b + 1) % n;
      const bx = P[3 * b], by = P[3 * b + 1], bz = P[3 * b + 2];
      const ax = P[3 * a] - bx, ay = P[3 * a + 1] - by, az = P[3 * a + 2] - bz, cx = P[3 * cc] - bx, cy = P[3 * cc + 1] - by, cz = P[3 * cc + 2] - bz;
      const la = Math.sqrt(ax * ax + ay * ay + az * az), lc = Math.sqrt(cx * cx + cy * cy + cz * cz);
      const ka = la > 0 ? Math.min(1, (3 * lc) / la) : 0, kc = lc > 0 ? Math.min(1, (3 * la) / lc) : 0;
      let done = false;
      if (ka > 0 && kc > 0) {
        // quarter-cut triangle (b, b + U, b + V), scaled by f about b
        const ux = 0.25 * ka * ax, uy = 0.25 * ka * ay, uz = 0.25 * ka * az, vx = 0.25 * kc * cx, vy = 0.25 * kc * cy, vz = 0.25 * kc * cz;
        const rho = Math.sqrt(Math.max(ux * ux + uy * uy + uz * uz, vx * vx + vy * vy + vz * vz));
        // every segment within 1.25ρ (beyond that f = 1 anyway), so none enters the minimum abruptly
        const R = 1.25 * rho;
        lo[0] = bx - R; lo[1] = by - R; lo[2] = bz - R; hi[0] = bx + R; hi[1] = by + R; hi[2] = bz + R;
        if (++mark === 0x7fffffff) { stamp.fill(0); mark = 1; }
        g.query(lo, hi, stamp, mark, ids);
        let fin = Infinity;
        flat = false;
        for (const id of ids) {
          const t = g.tag[id];
          if (t === c * VBITS + a || t === c * VBITS + b) continue; // the two edges at b
          const o = 6 * id, sg = g.seg;
          fin = Math.min(fin, enters(sg[o] - bx, sg[o + 1] - by, sg[o + 2] - bz, sg[o + 3] - bx, sg[o + 4] - by, sg[o + 5] - bz, ux, uy, uz, vx, vy, vz));
        }
        // below every entry scale, the cut meets nothing: only a flat (coplanar or degenerate) case needs the exact check,
        // shrinking further if it fails
        for (let f = Math.min(1, 0.8 * fin), k = 0; k < 4 && f > 1e-12 && !done; f /= 2, k++) {
          if (!flat) { const qx = bx + ux * f, qy = by + uy * f, qz = bz + uz * f, rx = bx + vx * f, ry = by + vy * f, rz = bz + vz * f; g.add(qx, qy, qz, rx, ry, rz, -1); out.push(qx, qy, qz, rx, ry, rz); from.push(b, b); done = true; break; }
          const qx = bx + ux * f, qy = by + uy * f, qz = bz + uz * f, rx = bx + vx * f, ry = by + vy * f, rz = bz + vz * f;
          let ok = true;
          for (const id of ids) {
            const t = g.tag[id];
            if (t === c * VBITS + a || t === c * VBITS + b) continue;
            const sg = g.seg, o = 6 * id;
            if (segTri(qx, qy, qz, bx, by, bz, rx, ry, rz, sg[o], sg[o + 1], sg[o + 2], sg[o + 3], sg[o + 4], sg[o + 5])
              || segDist2(qx, qy, qz, rx, ry, rz, sg[o], sg[o + 1], sg[o + 2], sg[o + 3], sg[o + 4], sg[o + 5]) < c2) { ok = false; break; }
          }
          if (ok) { g.add(qx, qy, qz, rx, ry, rz, -1); out.push(qx, qy, qz, rx, ry, rz); from.push(b, b); done = true; }
        }
      }
      if (!done) { out.push(bx, by, bz); from.push(b); }
    }
    if (next && orig) next[c] = Int32Array.from(from, (b) => orig[c][b]);
    return Float64Array.from(out);
  });
}

/**
 * Iterations for a curve of n vertices (each doubles them): few long sticks, as late in a run, need more to lose their
 * kinks; dense curves look smooth after 3. About 1500 points at most from the extra ones.
 */
const itersFor = (S: Scene) => { const n = S.reduce((k, P) => k + P.length / 3, 0); return n <= 47 ? 5 : n <= 94 ? 4 : 3; };

/** Delta-checked corner-cutting smoothing, `iters` iterations (default: by size, 3 to 5). */
export function smoothScene(S: Scene, iters = itersFor(S)): Scene {
  for (let k = 0; k < iters; k++) S = iterate(S);
  return S;
}

/** smoothScene, also giving for every output point the input vertex it comes from. */
function smoothTracked(S: Scene, iters: number): { S: Scene; from: Int32Array[] } {
  let from: Int32Array[] = S.map((P) => Int32Array.from({ length: P.length / 3 }, (_, i) => i));
  for (let k = 0; k < iters; k++) { const next: Int32Array[] = []; S = iterate(S, from, next); from = next; }
  return { S, from };
}

/**
 * (1 − t)·A + t·B for two smoothings of nearly the same curve, matched piece by piece: the stretch from where
 * input vertex v's points begin to where v + 1's begin is resampled by arc length in both and blended. `fa`, `fb`
 * give each output point's input vertex in A's and B's inputs; `toA` maps B's input vertices to A's (−1: only in B).
 */
function blendCurves(A: Scene, fa: Int32Array[], B: Scene, fb: Int32Array[], toA: Int32Array[], t: number): Scene {
  return A.map((PA, c) => {
    const PB = B[c], FA = fa[c], FB = fb[c], M = toA[c], n = Math.max(...FA) + 1;
    const startA = new Int32Array(n).fill(-1), startB = new Int32Array(n).fill(-1);
    FA.forEach((v, i) => { if (startA[v] < 0) startA[v] = i; });
    FB.forEach((w, i) => { const v = M[w]; if (v >= 0 && startB[v] < 0) startB[v] = i; });
    const out: number[] = [];
    // the piece of P from index i0 to i1 (cyclic, i1 included): its points and their arc-length fractions
    const piece = (P: Float64Array, i0: number, i1: number) => {
      const N = P.length / 3, idx: number[] = [i0];
      for (let i = (i0 + 1) % N; idx.length <= N; i = (i + 1) % N) { idx.push(i); if (i === i1) break; }
      const s = [0];
      for (let k = 1; k < idx.length; k++) { const p = 3 * idx[k - 1], q = 3 * idx[k]; s.push(s[k - 1] + Math.hypot(P[q] - P[p], P[q + 1] - P[p + 1], P[q + 2] - P[p + 2])); }
      const L = s[s.length - 1] || 1;
      return { idx, f: s.map((x) => x / L) };
    };
    const at = (P: Float64Array, pc: { idx: number[]; f: number[] }, x: number, k: number, o: number[]) => {
      const { idx, f } = pc, u = f[k + 1] > f[k] ? (x - f[k]) / (f[k + 1] - f[k]) : 0, p = 3 * idx[k], q = 3 * idx[k + 1];
      for (let d = 0; d < 3; d++) o[d] = P[p + d] + u * (P[q + d] - P[p + d]);
    };
    const pa3 = [0, 0, 0], pb3 = [0, 0, 0];
    for (let v = 0; v < n; v++) {
      const w = (v + 1) % n, a0 = startA[v], a1 = startA[w], b0 = startB[v], b1 = startB[w];
      if (a0 < 0 || b0 < 0 || a1 < 0 || b1 < 0) return PA; // not matched: no blend
      const ca = piece(PA, a0, a1), cb = piece(PB, b0, b1);
      // both curves at the union of their own breakpoints, so the blend is exactly A at t = 0 and B at t = 1
      const xs = [...ca.f.slice(0, -1), ...cb.f.slice(0, -1)].sort((p, q) => p - q);
      let ka = 0, kb = 0, last = -1;
      for (const x of xs) {
        if (x === last) continue;
        last = x;
        while (ka + 2 < ca.f.length && ca.f[ka + 1] <= x) ka++;
        while (kb + 2 < cb.f.length && cb.f[kb + 1] <= x) kb++;
        at(PA, ca, x, ka, pa3); at(PB, cb, x, kb, pb3);
        for (let d = 0; d < 3; d++) out.push(pa3[d] + t * (pb3[d] - pa3[d]));
      }
    }
    return Float64Array.from(out);
  });
}

/**
 * The displayed curve: the smoothing of S, or, while a deletion fades (S with the deleted vertices put back as
 * collinear ghosts, weight alpha), the blend from that smoothing to S's own.
 */
export function smoothShown(S: Scene, ghost?: { S: Scene; alpha: number; toCurrent: Int32Array[] } | null): Scene {
  if (!ghost || !(ghost.alpha > 0)) return smoothScene(S);
  // one iteration count for both, so the blend compares like with like
  const it = itersFor(ghost.S), a = smoothTracked(S, it), b = smoothTracked(ghost.S, it);
  return blendCurves(a.S, a.from, b.S, b.from, ghost.toCurrent, ghost.alpha);
}
