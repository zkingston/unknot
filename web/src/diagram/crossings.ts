// Planar segment intersections with a uniform-grid broad phase; regularity checks.
import { type Diagram, type Crossing, type StrandRef, extent2 } from './types';

const key = (s: StrandRef) => `${s.comp}:${s.seg}`;
const before = (p: StrandRef, q: StrandRef) => p.comp - q.comp || p.seg - q.seg || p.t - q.t;

export interface RawCrossing { a: StrandRef; b: StrandRef; x: number; y: number }
export interface Irregular { kind: 'tangency' | 'triple' | 'vertex'; x: number; y: number }

/**
 * Every transverse intersection between segments of the closed curves, exactly once, with half-open [0,1)
 * parameters on both segments; adjacent segments of one curve are skipped. Sorted in canonical order:
 * by the earlier strand (comp, seg + t), then the later one. `irregular` collects tangencies.
 */
export function findCrossings(comps: Float64Array[], irregular?: Irregular[]): RawCrossing[] {
  const segs: { c: number; i: number; x0: number; y0: number; x1: number; y1: number }[] = [];
  let total = 0;
  for (let c = 0; c < comps.length; c++) {
    const P = comps[c], n = P.length / 2;
    for (let i = 0; i < n; i++) {
      const j = i + 1 === n ? 0 : i + 1;
      const s = { c, i, x0: P[2 * i], y0: P[2 * i + 1], x1: P[2 * j], y1: P[2 * j + 1] };
      const dx = s.x1 - s.x0, dy = s.y1 - s.y0;
      total += Math.sqrt(dx * dx + dy * dy);
      segs.push(s);
    }
  }
  if (!segs.length) return [];
  const { x0, y0, ext } = extent2(comps);
  const cell = Math.max(total / segs.length, ext / 512, 1e-12) * 2;
  const grid = new Map<number, number[]>();
  const W = 1 << 20;
  if (segs.length > 256) segs.forEach((s, k) => {
    const cx0 = Math.floor((Math.min(s.x0, s.x1) - x0) / cell), cx1 = Math.floor((Math.max(s.x0, s.x1) - x0) / cell);
    const cy0 = Math.floor((Math.min(s.y0, s.y1) - y0) / cell), cy1 = Math.floor((Math.max(s.y0, s.y1) - y0) / cell);
    for (let gx = cx0; gx <= cx1; gx++) for (let gy = cy0; gy <= cy1; gy++) {
      const gk = gx * W + gy, l = grid.get(gk);
      if (l) l.push(k); else grid.set(gk, [k]);
    }
  });
  const seen = new Set<number>(), out: RawCrossing[] = [], N = segs.length;
  // small inputs: every pair (cheaper than the grid); large: pairs sharing a grid cell
  const lists = N <= 256 ? [segs.map((_, k) => k)] : grid.values();
  for (const l of lists) {
    for (let p = 0; p < l.length; p++) for (let q = p + 1; q < l.length; q++) {
      const ka = Math.min(l[p], l[q]), kb = Math.max(l[p], l[q]);
      if (N > 256) { const pk = ka * N + kb; if (seen.has(pk)) continue; seen.add(pk); }
      const A = segs[ka], B = segs[kb];
      if (A.c === B.c) {
        const n = comps[A.c].length / 2;
        if (B.i === A.i + 1 || (A.i === 0 && B.i === n - 1)) continue; // adjacent
      }
      const rx = A.x1 - A.x0, ry = A.y1 - A.y0, sx = B.x1 - B.x0, sy = B.y1 - B.y0;
      const qx = B.x0 - A.x0, qy = B.y0 - A.y0, den = rx * sy - ry * sx;
      if (Math.abs(den) <= 1e-14 * Math.sqrt((rx * rx + ry * ry) * (sx * sx + sy * sy))) {
        // parallel: collinear overlap is a tangency
        if (irregular && Math.abs(qx * ry - qy * rx) <= 1e-12 * (rx * rx + ry * ry)) {
          const rr = rx * rx + ry * ry, t0 = (qx * rx + qy * ry) / rr, t1 = ((B.x1 - A.x0) * rx + (B.y1 - A.y0) * ry) / rr;
          if (Math.max(t0, t1) >= 0 && Math.min(t0, t1) <= 1) irregular.push({ kind: 'tangency', x: B.x0, y: B.y0 });
        }
        continue;
      }
      const t = (qx * sy - qy * sx) / den, u = (qx * ry - qy * rx) / den;
      if (t >= 0 && t < 1 && u >= 0 && u < 1) out.push({ a: { comp: A.c, seg: A.i, t }, b: { comp: B.c, seg: B.i, t: u }, x: A.x0 + t * rx, y: A.y0 + t * ry });
    }
  }
  for (const X of out) if (before(X.a, X.b) > 0) { const s = X.a; X.a = X.b; X.b = s; }
  return out.sort((X, Y) => before(X.a, Y.a) || before(X.b, Y.b));
}

/** Recompute all intersections; the Diagram must be regular and its crossings must match them exactly. */
export function validateDiagram(d: Diagram): { ok: boolean; error?: string; found: RawCrossing[] } {
  for (const P of d.comps) if (P.length < 6) return { ok: false, error: 'a curve has fewer than 3 points', found: [] };
  const irr: Irregular[] = [];
  const found = findCrossings(d.comps, irr);
  if (irr.length) return { ok: false, error: 'tangency', found };
  const { ext } = extent2(d.comps), eps = 1e-6 * Math.max(ext, 1e-300);
  // crossings near a vertex, and triple points (two crossings at one location)
  for (const X of found) for (const s of [X.a, X.b]) {
    const P = d.comps[s.comp], n = P.length / 2, j = (s.seg + 1) % n;
    const dx = P[2 * j] - P[2 * s.seg], dy = P[2 * j + 1] - P[2 * s.seg + 1], L = Math.sqrt(dx * dx + dy * dy);
    if (s.t * L < eps || (1 - s.t) * L < eps) return { ok: false, error: 'crossing at a vertex', found };
  }
  const sorted = found.map((X) => X).sort((p, q) => p.x - q.x);
  for (let i = 0; i < sorted.length; i++) for (let j = i + 1; j < sorted.length && sorted[j].x - sorted[i].x < eps; j++) {
    if (Math.abs(sorted[j].y - sorted[i].y) < eps) return { ok: false, error: 'triple point', found };
  }
  if (found.length !== d.crossings.length) return { ok: false, error: `crossing count ${d.crossings.length}, found ${found.length}`, found };
  const want = new Map<string, [StrandRef, StrandRef][]>();
  for (const X of d.crossings) {
    const [p, q] = before(X.under, X.over) < 0 ? [X.under, X.over] : [X.over, X.under];
    const k = key(p) + '|' + key(q), l = want.get(k);
    if (l) l.push([p, q]); else want.set(k, [[p, q]]);
  }
  for (const F of found) {
    const l = want.get(key(F.a) + '|' + key(F.b));
    const m = l ? l.findIndex(([p, q]) => Math.abs(p.t - F.a.t) < 1e-6 && Math.abs(q.t - F.b.t) < 1e-6) : -1;
    if (!l || m < 0) return { ok: false, error: 'crossing list does not match the curves', found };
    l.splice(m, 1);
  }
  return { ok: true, found };
}

/** Build a Diagram from curves; `overA(X, k)` says whether strand `a` is over (null: undecided, so flat). */
export function diagramFromCurves(comps: Float64Array[], source: Diagram['source'], overA: (X: RawCrossing, k: number) => boolean | null): Diagram {
  const crossings: Crossing[] = findCrossings(comps).map((X, k) => {
    const o = overA(X, k);
    if (o === null) return { under: X.a, over: X.b, flat: true as const };
    return o ? { under: X.b, over: X.a } : { under: X.a, over: X.b };
  });
  return { comps, crossings, source };
}
