// Steps 6–11: pair arc endpoints across under-crossing gaps, bridge them, close the curves, find crossings.
import type { Diagram, Crossing } from '../../diagram/types';
import { findCrossings, validateDiagram } from '../../diagram/crossings';
import { maxWeightMatching } from './match';
import { prepare, type RGBAImage, type Method } from './threshold';
import { traceArcs, type Arc } from './trace';

export interface Endpoint { arc: number; end: 0 | 1; x: number; y: number; tx: number; ty: number; partner: number }
export interface TraceResult {
  diagram: Diagram; sw: number; arcs: Float64Array[]; gaps: Float64Array[]; endpoints: Endpoint[];
  diagnostics: { arcs: number; unpaired: number; flat: number; components: number };
  /** processed raster size (arc coordinates are processed px, y up) */
  image?: { width: number; height: number; method: string };
}
export interface TraceOptions { threshold?: number; method?: Method; force?: [number, number][]; forbid?: [number, number][] }
export type InkTest = (x0: number, y0: number, x1: number, y1: number) => boolean;

/** Endpoint position and outward unit tangent, from the point `L` back along the arc. */
function endTangent(xy: Float64Array, end: 0 | 1, L: number) {
  const n = xy.length / 2, at = (k: number) => (end === 1 ? n - 1 - k : k);
  const px = xy[2 * at(0)], py = xy[2 * at(0) + 1];
  let s = 0, k = 1, qx = px, qy = py;
  for (; k < n; k++) {
    const ax = xy[2 * at(k - 1)], ay = xy[2 * at(k - 1) + 1], bx = xy[2 * at(k)], by = xy[2 * at(k) + 1];
    s += Math.sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
    qx = bx; qy = by;
    if (s >= L) break;
  }
  const tx = px - qx, ty = py - qy, l = Math.sqrt(tx * tx + ty * ty) || 1;
  return { x: px, y: py, tx: tx / l, ty: ty / l };
}

/** Cubic Hermite from p0 (tangent t0) into p1 (arriving along −t1), tangent length = gap length; interior points. */
function hermite(a: Endpoint, b: Endpoint, n = 24): number[] {
  const d = Math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2), out: number[] = [];
  for (let k = 1; k < n - 1; k++) {
    const s = k / (n - 1), s2 = s * s, s3 = s2 * s;
    const h00 = 2 * s3 - 3 * s2 + 1, h10 = s3 - 2 * s2 + s, h01 = -2 * s3 + 3 * s2, h11 = s3 - s2;
    out.push(h00 * a.x + h10 * a.tx * d + h01 * b.x - h11 * b.tx * d, h00 * a.y + h10 * a.ty * d + h01 * b.y - h11 * b.ty * d);
  }
  return out;
}

/** Uniform resampling of a closed curve; a sample is a gap sample if either neighbouring input point is. */
function resample(xy: number[], gap: boolean[], step: number, phase: number) {
  const n = xy.length / 2, S = new Float64Array(n + 1);
  for (let i = 0; i < n; i++) { const j = (i + 1) % n; S[i + 1] = S[i] + Math.sqrt((xy[2 * j] - xy[2 * i]) ** 2 + (xy[2 * j + 1] - xy[2 * i + 1]) ** 2); }
  const L = S[n], m = Math.max(Math.floor(L / step), 12), out = new Float64Array(2 * m), g: boolean[] = [];
  let i = 0;
  for (let k = 0; k < m; k++) {
    const u = ((k + phase) * L) / m;
    while (i + 1 < n && S[i + 1] <= u) i++;
    const j = (i + 1) % n, f = S[i + 1] > S[i] ? (u - S[i]) / (S[i + 1] - S[i]) : 0;
    out[2 * k] = xy[2 * i] + f * (xy[2 * j] - xy[2 * i]); out[2 * k + 1] = xy[2 * i + 1] + f * (xy[2 * j + 1] - xy[2 * i + 1]);
    g.push(gap[i] || gap[j]);
  }
  return { xy: out, gap: g };
}

export function assembleArcs(arcs: Arc[], sw: number, crossesInk: InkTest, opts: TraceOptions = {}): TraceResult {
  // 6. endpoints and outward tangents (2 sw back)
  const E: Endpoint[] = [];
  arcs.forEach((a, ai) => { if (!a.closed) for (const end of [0, 1] as const) E.push({ arc: ai, end, ...endTangent(a.xy, end, 2 * sw), partner: -1 }); });
  const key = (i: number, j: number) => (i < j ? `${i},${j}` : `${j},${i}`);
  const forbid = new Set((opts.forbid ?? []).map(([i, j]) => key(i, j)));
  const forced = new Set<number>();
  for (const [i, j] of opts.force ?? []) if (i !== j && E[i] && E[j] && !forced.has(i) && !forced.has(j)) { E[i].partner = j; E[j].partner = i; forced.add(i); forced.add(j); }
  // 7. candidate gaps; 8. minimum-cost maximum matching
  const edges: [number, number, number][] = [], BIG = 1e6;
  for (let i = 0; i < E.length; i++) for (let j = i + 1; j < E.length; j++) {
    if (forced.has(i) || forced.has(j) || forbid.has(key(i, j))) continue;
    const a = E[i], b = E[j], vx = b.x - a.x, vy = b.y - a.y, d = Math.sqrt(vx * vx + vy * vy);
    if (d > 14 * sw || d < 1e-6) continue;
    const ci = (a.tx * vx + a.ty * vy) / d, cj = -(b.tx * vx + b.ty * vy) / d;
    if (ci < 0.4 || cj < 0.4) continue;
    if (!crossesInk(a.x + 0.2 * vx, a.y + 0.2 * vy, a.x + 0.8 * vx, a.y + 0.8 * vy)) continue;
    edges.push([i, j, Math.round(BIG - 1000 * (d / sw + 4 * (2 - ci - cj)))]);
  }
  const mate = maxWeightMatching(edges, true);
  mate.forEach((j, i) => { if (j >= 0 && !forced.has(i)) E[i].partner = j; });
  // 9. join arcs through matched gaps into closed curves
  const eidx = new Map(E.map((e, k) => [`${e.arc}:${e.end}`, k]));
  const used = new Set<number>(), comps: { xy: number[]; gap: boolean[] }[] = [], gaps: Float64Array[] = [];
  arcs.forEach((a, ai) => {
    if (used.has(ai)) return;
    if (a.closed) { used.add(ai); comps.push({ xy: Array.from(a.xy), gap: new Array(a.xy.length / 2).fill(false) }); return; }
    const xy: number[] = [], gap: boolean[] = [], bridges: Float64Array[] = [], chain: number[] = [];
    let cur = ai, entry: 0 | 1 = 0, ok = false;
    for (let guard = 0; guard <= arcs.length; guard++) {
      chain.push(cur);
      const A = arcs[cur].xy, n = A.length / 2;
      for (let k = 0; k < n; k++) { const q = entry === 0 ? k : n - 1 - k; xy.push(A[2 * q], A[2 * q + 1]); gap.push(false); }
      const k: number = eidx.get(`${cur}:${1 - entry}`)!, nk: number = E[k].partner;
      if (nk < 0) break;
      const h = hermite(E[k], E[nk]);
      for (let q = 0; q < h.length; q += 2) { xy.push(h[q], h[q + 1]); gap.push(true); }
      bridges.push(Float64Array.from([E[k].x, E[k].y, ...h, E[nk].x, E[nk].y]));
      const na: number = E[nk].arc, ne: 0 | 1 = E[nk].end;
      if (na === ai && ne === 0) { ok = true; break; }
      if (chain.includes(na)) break;
      cur = na; entry = ne;
    }
    for (const c of chain) used.add(c);
    if (ok) { comps.push({ xy, gap }); gaps.push(...bridges); }
  });
  // 10. resample at 0.5 sw; a gap sample crossing ink is a crossing with the gap strand under.
  // Pixel paths are lattice-aligned, so samples get an irrational phase; retry another if a crossing lands on a vertex.
  let curves: Float64Array[] = [], crossings: Crossing[] = [], flat = 0;
  for (const phase of [0.381966, 0.618034, 0.145898, 0.763932]) {
    const rs = comps.map((c) => resample(c.xy, c.gap, 0.5 * sw, phase));
    curves = rs.map((r) => r.xy); flat = 0;
    crossings = findCrossings(curves).map((X) => {
      const ga = rs[X.a.comp].gap[X.a.seg], gb = rs[X.b.comp].gap[X.b.seg];
      if (ga && !gb) return { under: X.a, over: X.b };
      if (gb && !ga) return { under: X.b, over: X.a };
      flat++;
      return { under: X.a, over: X.b, flat: true as const };
    });
    if (validateDiagram({ comps: curves, crossings, source: 'image' }).ok) break;
  }
  // 11. diagram + diagnostics
  return {
    diagram: { comps: curves, crossings, source: 'image' },
    sw, arcs: arcs.map((a) => a.xy), gaps, endpoints: E,
    diagnostics: { arcs: arcs.length, unpaired: E.filter((e) => e.partner < 0).length, flat, components: comps.length },
  };
}

/** Raster diagram -> Diagram (SPEC Image import, steps 1–11). */
export function traceImage(img: RGBAImage, opts: TraceOptions = {}): TraceResult {
  const P = prepare(img, opts), { width: w, height: h } = P;
  const T = traceArcs(P.mask, w, h);
  // the straight line between endpoints must cross (eroded) ink in its middle: sample 25 points
  const ink: InkTest = (x0, y0, x1, y1) => {
    for (let k = 0; k < 25; k++) {
      const t = k / 24, c = Math.round(x0 + t * (x1 - x0)), r = Math.round(-(y0 + t * (y1 - y0)));
      if (r >= 0 && r < h && c >= 0 && c < w && T.ink[r * w + c]) return true;
    }
    return false;
  };
  return { ...assembleArcs(T.arcs, T.sw, ink, opts), image: { width: w, height: h, method: P.method } };
}
