// Test-only isotopy checker. Written separately from the kernel so the kernel never grades itself.
// For every Move: each moving edge (F, b_t) against each non-adjacent edge (p,q): the coplanarity triple product
// is linear in t; at its root the segments must be >= MIN_GAP apart. A pair coplanar throughout is checked in its
// plane against the swept triangle. For every Delete: v within 1e-12 of segment (v-1, v+1).

export type P3 = [number, number, number];
const MIN_GAP = 1e-9;

const sub = (a: P3, b: P3): P3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a: P3, b: P3, s = 1): P3 => [a[0] + s * b[0], a[1] + s * b[1], a[2] + s * b[2]];
const dot = (a: P3, b: P3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const crs = (a: P3, b: P3): P3 => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const len = (a: P3) => Math.sqrt(dot(a, a));

/** Point–segment distance. */
export function ptSeg(p: P3, a: P3, b: P3): number {
  const ab = sub(b, a), l2 = dot(ab, ab);
  const t = l2 > 0 ? Math.min(1, Math.max(0, dot(sub(p, a), ab) / l2)) : 0;
  return len(sub(p, add(a, ab, t)));
}

/** Segment–segment distance: minimum of the four endpoint distances and the interior critical point. */
export function segSeg(a0: P3, a1: P3, b0: P3, b1: P3): number {
  let m = Math.min(ptSeg(a0, b0, b1), ptSeg(a1, b0, b1), ptSeg(b0, a0, a1), ptSeg(b1, a0, a1));
  const u = sub(a1, a0), v = sub(b1, b0), w = sub(a0, b0);
  const uu = dot(u, u), vv = dot(v, v), uv = dot(u, v), uw = dot(u, w), vw = dot(v, w);
  const D = uu * vv - uv * uv;
  if (D > 1e-30 * uu * vv) {
    const s = (uv * vw - vv * uw) / D, t = (uu * vw - uv * uw) / D;
    if (s > 0 && s < 1 && t > 0 && t < 1) m = Math.min(m, len(sub(add(a0, u, s), add(b0, v, t))));
  }
  return m;
}

/** Point–triangle distance (closed triangle). */
function ptTri(p: P3, a: P3, b: P3, c: P3): number {
  const n = crs(sub(b, a), sub(c, a)), nn = len(n);
  let m = Math.min(ptSeg(p, a, b), ptSeg(p, b, c), ptSeg(p, c, a));
  if (nn > 0) {
    const h = dot(sub(p, a), n) / nn, q = add(p, n, -h / nn);
    const s1 = dot(crs(sub(b, a), sub(q, a)), n), s2 = dot(crs(sub(c, b), sub(q, b)), n), s3 = dot(crs(sub(a, c), sub(q, c)), n);
    if ((s1 >= 0 && s2 >= 0 && s3 >= 0) || (s1 <= 0 && s2 <= 0 && s3 <= 0)) m = Math.min(m, Math.abs(h));
  }
  return m;
}

/** Segment–triangle distance (0 if they meet). */
export function segTriDist(p: P3, q: P3, a: P3, b: P3, c: P3): number {
  let m = Math.min(segSeg(p, q, a, b), segSeg(p, q, b, c), segSeg(p, q, c, a), ptTri(p, a, b, c), ptTri(q, a, b, c));
  const n = crs(sub(b, a), sub(c, a)), nn = len(n);
  if (nn > 0) {
    const dp = dot(sub(p, a), n), dq = dot(sub(q, a), n);
    if ((dp < 0 && dq > 0) || (dp > 0 && dq < 0)) {
      const x = add(p, sub(q, p), dp / (dp - dq));
      if (ptTri(x, a, b, c) === 0 || ptTri(x, a, b, c) < 1e-15) m = 0;
      else m = Math.min(m, ptTri(x, a, b, c));
    }
  }
  return m;
}

export const pt = (P: Float64Array, i: number): P3 => [P[3 * i], P[3 * i + 1], P[3 * i + 2]];
export const lerp = (a: P3, b: P3, t: number): P3 => [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * (b[2] - a[2])];

/** Smallest gap seen while moving vertex v of comp c straight to `to`; reports each passage. */
export function moveGap(S: Float64Array[], c: number, v: number, to: P3): { gap: number; tFirst: number } {
  const P = S[c], n = P.length / 3, ia = (v + n - 1) % n, ic = (v + 1) % n;
  const b = pt(P, v), D = sub(to, b);
  let gap = Infinity, tFirst = Infinity;
  for (let k = 0; k < S.length; k++) {
    const Q = S[k], m = Q.length / 3;
    for (let i = 0; i < m; i++) {
      const j = (i + 1) % m;
      if (k === c && (i === ia || i === v)) continue;
      const p = pt(Q, i), q = pt(Q, j);
      for (const [F, adj] of [[pt(P, ia), k === c && j === ia], [pt(P, ic), k === c && i === ic]] as [P3, boolean][]) {
        if (adj) continue;
        const N = crs(sub(p, F), sub(q, F));
        const f0 = dot(sub(b, F), N), f1 = dot(D, N);
        const scale = len(N) * (len(sub(b, F)) + len(D));
        if (Math.abs(f0) <= 1e-13 * scale && Math.abs(f1) <= 1e-13 * scale) {
          // coplanar throughout: the moving edge sweeps triangle (F, b, to) in that plane
          const g = segTriDist(p, q, F, b, to);
          if (g < gap) gap = g;
          if (g < MIN_GAP) tFirst = Math.min(tFirst, 0);
          continue;
        }
        if (f1 === 0) continue;
        const r = -f0 / f1;
        if (r < 0 || r > 1) continue;
        const g = segSeg(F, add(b, D, r), p, q);
        if (g < gap) gap = g;
        if (g < MIN_GAP && r < tFirst) tFirst = r;
      }
    }
  }
  return { gap, tFirst };
}

export interface IsotopyReport { passages: number; moves: number; deletes: number; minGap: number; bad: string[] }

/** Replay `recs` from `initial`, checking every record. */
export function checkIsotopy(initial: Float64Array[], recs: Float64Array): IsotopyReport {
  const S = initial.map((P) => P.slice());
  const rep: IsotopyReport = { passages: 0, moves: 0, deletes: 0, minGap: Infinity, bad: [] };
  for (let o = 0; o < recs.length; o += 6) {
    const op = recs[o], c = recs[o + 1], v = recs[o + 2];
    if (op === 0) {
      rep.moves++;
      const to: P3 = [recs[o + 3], recs[o + 4], recs[o + 5]];
      const { gap } = moveGap(S, c, v, to);
      rep.minGap = Math.min(rep.minGap, gap);
      if (gap < MIN_GAP) { rep.passages++; if (rep.bad.length < 10) rep.bad.push(`record ${o / 6}: move c${c} v${v} gap ${gap}`); }
      S[c][3 * v] = to[0]; S[c][3 * v + 1] = to[1]; S[c][3 * v + 2] = to[2];
    } else if (op === 1) {
      rep.deletes++;
      const P = S[c], n = P.length / 3;
      const d = ptSeg(pt(P, v), pt(P, (v + n - 1) % n), pt(P, (v + 1) % n));
      if (!(d <= 1e-12) || n <= 3) { rep.passages++; if (rep.bad.length < 10) rep.bad.push(`record ${o / 6}: delete c${c} v${v} off-line ${d}`); }
      const Q = new Float64Array(P.length - 3);
      Q.set(P.subarray(0, 3 * v)); Q.set(P.subarray(3 * v + 3), 3 * v);
      S[c] = Q;
    } else if (op === 3) {
      const P = S[c], n = P.length / 3, x: P3 = [recs[o + 3], recs[o + 4], recs[o + 5]];
      const d = ptSeg(x, pt(P, (v + n - 1) % n), pt(P, v % n));
      if (!(d <= 1e-12)) { rep.passages++; if (rep.bad.length < 10) rep.bad.push(`record ${o / 6}: insert c${c} v${v} off-line ${d}`); }
      const Q = new Float64Array(P.length + 3);
      Q.set(P.subarray(0, 3 * v)); Q.set(x, 3 * v); Q.set(P.subarray(3 * v), 3 * v + 3);
      S[c] = Q;
    }
  }
  return rep;
}
