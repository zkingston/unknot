// 2D knot notation: project the smoothed Scene, find crossings, cut the under-strand at each, emit SVG paths.
import type { Scene } from '../kernel/scene';
import { type View, projectCurves, depthAt } from '../diagram/project';
import { findCrossings, type RawCrossing } from '../diagram/crossings';
import type { StrandRef } from '../diagram/types';

const NS = 'http://www.w3.org/2000/svg';
/** Stroke width (px; matches --pen in styles.css) and the under-strand gap either side of a crossing. */
export const PEN = 3.5, GAP = 3 * PEN;

export interface Frame { cx: number; cy: number; s: number }
export interface Drawn { crossings: (RawCrossing & { aOver: boolean; sign: 1 | -1 })[]; curves: Float64Array[]; paths: string[]; cuts: number[][] }
export interface Annot { numbers?: boolean; signs?: boolean; chevrons?: boolean }

/** Screen-space curves (px, y down) for a Scene seen along `view` through `frame`. */
function toScreen(S: Scene, view: View, f: Frame): Float64Array[] {
  return projectCurves(S, view).map((Q) => { for (let i = 0; i < Q.length; i += 2) { Q[i] = f.cx + f.s * Q[i]; Q[i + 1] = f.cy - f.s * Q[i + 1]; } return Q; });
}

/** Path data for every visible run, under-strands cut within ±gap (arc length, px) of each crossing. */
export function knotPaths(S: Scene, view: View, f: Frame, gap: number): { d: string[]; drawn: Drawn } {
  const curves = toScreen(S, view, f), X = findCrossings(curves);
  const cuts: number[][] = curves.map(() => []);
  const arcs = curves.map(arcLength);
  const crossings = X.map((x) => {
    const aOver = depthAt(S, view, x.a.comp, x.a.seg, x.a.t) > depthAt(S, view, x.b.comp, x.b.seg, x.b.t);
    const u = aOver ? x.b : x.a;
    cuts[u.comp].push(arcAt(arcs[u.comp], u));
    // sign on screen (y down): positive iff u×o > 0 here, i.e. u×o < 0 in y-up
    const o = aOver ? x.a : x.b, dir = (r: typeof u) => { const Q = curves[r.comp], n = Q.length / 2, j = (r.seg + 1) % n; return [Q[2 * j] - Q[2 * r.seg], Q[2 * j + 1] - Q[2 * r.seg + 1]]; };
    const du = dir(u), dov = dir(o), sign: 1 | -1 = du[0] * dov[1] - du[1] * dov[0] > 0 ? 1 : -1;
    return { ...x, aOver, sign };
  });
  const d = cutPaths(curves, cuts, gap);
  return { d, drawn: { crossings, curves, paths: d, cuts } };
}

/** Cumulative arc length of a closed 2D curve (n + 1 entries). */
export function arcLength(Q: Float64Array) {
  const n = Q.length / 2, A = new Float64Array(n + 1);
  for (let i = 0; i < n; i++) { const j = (i + 1) % n, dx = Q[2 * j] - Q[2 * i], dy = Q[2 * j + 1] - Q[2 * i + 1]; A[i + 1] = A[i] + Math.sqrt(dx * dx + dy * dy); }
  return A;
}

/** Arc-length position of a strand point, given its curve's arcLength. */
export const arcAt = (A: Float64Array, r: Pick<StrandRef, 'seg' | 't'>) => A[r.seg] + r.t * (A[r.seg + 1] - A[r.seg]);

/** SVG path data for closed curves with the arc-length positions `cuts[c]` removed within ±gap. */
export function cutPaths(curves: Float64Array[], cuts: number[][], gap: number): string[] {
  const d: string[] = [];
  curves.forEach((Q, c) => {
    const n = Q.length / 2, A = arcLength(Q), L = A[n];
    const at = (s: number) => {
      s = ((s % L) + L) % L;
      let lo = 0, hi = n - 1;
      while (lo < hi) { const m = (lo + hi + 1) >> 1; if (A[m] <= s) lo = m; else hi = m - 1; }
      const j = (lo + 1) % n, t = A[lo + 1] > A[lo] ? (s - A[lo]) / (A[lo + 1] - A[lo]) : 0;
      return [Q[2 * lo] + t * (Q[2 * j] - Q[2 * lo]), Q[2 * lo + 1] + t * (Q[2 * j + 1] - Q[2 * lo + 1]), lo] as const;
    };
    const fmt = (x: number) => (Math.round(x * 100) / 100).toString();
    if (!cuts[c].length) {
      let s = `M${fmt(Q[0])} ${fmt(Q[1])}`;
      for (let i = 1; i < n; i++) s += `L${fmt(Q[2 * i])} ${fmt(Q[2 * i + 1])}`;
      d.push(s + 'Z');
      return;
    }
    // visible intervals = complement of the merged cut windows on the circle
    const w = cuts[c].map((s) => [s - gap, s + gap]).sort((p, q) => p[0] - q[0]);
    const merged: number[][] = [];
    for (const iv of w) { const last = merged[merged.length - 1]; if (last && iv[0] <= last[1]) last[1] = Math.max(last[1], iv[1]); else merged.push([...iv]); }
    const first = merged[0], last = merged[merged.length - 1];
    if (merged.length > 1 && last[1] - L >= first[0]) { first[0] = last[0] - L; first[1] = Math.max(first[1], last[1] - L); merged.pop(); }
    for (let k = 0; k < merged.length; k++) {
      const s0 = merged[k][1], s1 = k + 1 < merged.length ? merged[k + 1][0] : merged[0][0] + L;
      if (s1 - s0 <= 0.5) continue;
      const p0 = at(s0);
      let path = `M${fmt(p0[0])} ${fmt(p0[1])}`;
      // vertices strictly inside (s0, s1)
      let i = p0[2] + 1, base = Math.floor(s0 / L) * L;
      for (let guard = 0; guard < n + 1; guard++, i++) {
        if (i >= n) { i -= n; base += L; }
        const s = base + A[i];
        if (s >= s1 || s <= s0) { if (s >= s1) break; continue; }
        path += `L${fmt(Q[2 * i])} ${fmt(Q[2 * i + 1])}`;
      }
      const p1 = at(s1);
      d.push(path + `L${fmt(p1[0])} ${fmt(p1[1])}`);
    }
  });
  return d;
}

/** A crossing label: where its crossing was, which strands, its number and direction from the crossing. */
interface Label { x: number; y: number; ca: number; cb: number; num: number; ang: number; gap: number }
const TRACK = 14, KEEP = Math.PI / 7, PREFER = -Math.PI / 4; // px a crossing may move per frame; least gap kept; up-right

/**
 * Labels that hold still while the curve moves: a crossing within TRACK px of a label of the last frame (same pair of
 * components) keeps its number and stays in the gap between the strands that held it while that gap is at least KEEP
 * wide (its bisector moves with the strands; a narrowing gap pushes the label out instead of moving it); new crossings
 * take the smallest free numbers and the gap facing up-right if it is at least 60°, else the widest (one of the four
 * gaps around a crossing is at least 90°).
 */
function trackLabels(prev: Label[], drawn: Drawn): Label[] {
  const dir = (r: StrandRef) => { const Q = drawn.curves[r.comp], n = Q.length / 2, j = (r.seg + 1) % n; return Math.atan2(Q[2 * j + 1] - Q[2 * r.seg + 1], Q[2 * j] - Q[2 * r.seg]); };
  const TAU = 2 * Math.PI, norm = (x: number) => ((x % TAU) + TAU) % TAU, X = drawn.crossings;
  // match closest pairs first, so neighbouring crossings do not trade labels
  const pairs: [number, number, number][] = [], match = new Int32Array(X.length).fill(-1), taken = new Uint8Array(prev.length);
  X.forEach((C, k) => {
    const ca = Math.min(C.a.comp, C.b.comp), cb = Math.max(C.a.comp, C.b.comp);
    prev.forEach((L, i) => { if (L.ca === ca && L.cb === cb) { const d = Math.hypot(L.x - C.x, L.y - C.y); if (d < TRACK) pairs.push([d, k, i]); } });
  });
  pairs.sort((p, q) => p[0] - q[0]);
  for (const [, k, i] of pairs) if (match[k] < 0 && !taken[i]) { match[k] = i; taken[i] = 1; }
  const out = X.map((C, k) => {
    const ca = Math.min(C.a.comp, C.b.comp), cb = Math.max(C.a.comp, C.b.comp), best = match[k];
    // the four gaps between the strand directions, as [start, width]
    const u = dir(C.a), v = dir(C.b), rays = [u, u + Math.PI, v, v + Math.PI].map(norm).sort((p, q) => p - q);
    const gaps = rays.map((r0, i) => [r0, (i < 3 ? rays[i + 1] : rays[0] + TAU) - r0]);
    const holding = (t: number) => gaps.find(([g0, w]) => norm(t - g0) < w)!;
    let g = holding(best >= 0 ? prev[best].ang : PREFER);
    const kept = best >= 0 && g[1] >= KEEP;
    if (g[1] < (best >= 0 ? KEEP : Math.PI / 3)) g = gaps.reduce((p, q) => (q[1] > p[1] ? q : p));
    // a kept label eases toward its gap's bisector (the arc between lies inside the gap), so a fast turn of the strands
    // swings it rather than jumping it
    const mid = g[0] + g[1] / 2, ang = kept ? prev[best].ang + 0.35 * (norm(mid - prev[best].ang + Math.PI) - Math.PI) : mid;
    return { x: C.x, y: C.y, ca, cb, num: best >= 0 ? prev[best].num : 0, ang, gap: g[1] };
  });
  // new crossings: the smallest numbers not in use, in crossing order
  const used = new Set(out.map((L) => L.num).filter((n) => n > 0));
  let next = 1;
  for (const L of out) if (!L.num) { while (used.has(next)) next++; L.num = next; used.add(next); }
  return out;
}

/** Owns a pool of <path> elements in one <svg>; updates only `d`. */
export class Render2D {
  private g: SVGGElement; private dots: SVGGElement; private ann: SVGGElement; private pool: SVGPathElement[] = []; private dotPool: SVGCircleElement[] = [];
  frame: Frame = { cx: 0, cy: 0, s: 1 };
  constructor(public svg: SVGSVGElement) {
    this.g = document.createElementNS(NS, 'g'); this.dots = document.createElementNS(NS, 'g'); this.ann = document.createElementNS(NS, 'g');
    svg.append(this.g, this.dots, this.ann);
  }
  /** With `sticks`, S is drawn as is with a dot at every vertex (none inside an under-strand gap). */
  draw(S: Scene, view: View, annot: Annot = {}, sticks = false): Drawn {
    const { d, drawn } = knotPaths(S, view, this.frame, GAP);
    while (this.pool.length < d.length) { const p = document.createElementNS(NS, 'path'); p.setAttribute('class', 'k'); this.g.append(p); this.pool.push(p); }
    this.pool.forEach((p, k) => { if (k < d.length) { if (p.getAttribute('d') !== d[k]) p.setAttribute('d', d[k]); p.style.display = ''; } else if (p.style.display !== 'none') p.style.display = 'none'; });
    const pts: number[] = [];
    if (sticks) drawn.curves.forEach((Q, c) => {
      const A = arcLength(Q), L = A[A.length - 1], cut = drawn.cuts[c];
      for (let i = 0; i < Q.length / 2; i++) if (!cut.some((s) => { const e = Math.abs(A[i] - s); return Math.min(e, L - e) < GAP; })) pts.push(Q[2 * i], Q[2 * i + 1]);
    });
    while (this.dotPool.length < pts.length / 2) { const e = document.createElementNS(NS, 'circle'); e.setAttribute('class', 'v'); this.dots.append(e); this.dotPool.push(e); }
    this.dotPool.forEach((e, k) => {
      if (2 * k < pts.length) { e.setAttribute('cx', pts[2 * k].toFixed(1)); e.setAttribute('cy', pts[2 * k + 1].toFixed(1)); e.setAttribute('r', (0.95 * PEN).toFixed(2)); e.style.display = ''; } else if (e.style.display !== 'none') e.style.display = 'none';
    });
    this.svg.classList.toggle('sticks', sticks);
    this.annotate(drawn, annot);
    return drawn;
  }
  /** Forget the crossing labels' numbers and places (a new knot, or annotations switched off). */
  resetLabels() { this.labels = []; }
  private labels: Label[] = [];

  private annotate(drawn: Drawn, a: Annot) {
    this.ann.textContent = '';
    if (!a.numbers && !a.signs && !a.chevrons) { this.labels = []; return; }
    const text = (x: number, y: number, s: string) => { const t = document.createElementNS(NS, 'text'); t.setAttribute('x', x.toFixed(1)); t.setAttribute('y', y.toFixed(1)); t.textContent = s; this.ann.append(t); };
    if (a.numbers || a.signs) {
      const labels = trackLabels(this.labels, drawn);
      labels.forEach((L, k) => {
        const X = drawn.crossings[k], parts: string[] = [];
        if (a.numbers) parts.push(String(L.num));
        if (a.signs) parts.push(X.sign > 0 ? '+' : '−');
        // out far enough that the label (about 7 px a character, 12 px tall) clears both strands of its gap
        const s = parts.join(' '), d = Math.min(40, Math.max(8 + 3.2 * s.length, (6 + 1.8 * s.length) / Math.sin(L.gap / 2)));
        text(X.x + d * Math.cos(L.ang), X.y + d * Math.sin(L.ang), s);
      });
      this.labels = labels;
    }
    if (a.chevrons) drawn.curves.forEach((Q) => {
      const n = Q.length / 2, step = Math.max(1, Math.floor(n / 12));
      for (let i = Math.floor(step / 2); i < n; i += step) {
        const j = (i + 1) % n, dx = Q[2 * j] - Q[2 * i], dy = Q[2 * j + 1] - Q[2 * i + 1], l = Math.sqrt(dx * dx + dy * dy) || 1;
        const ux = dx / l, uy = dy / l, x = (Q[2 * i] + Q[2 * j]) / 2, y = (Q[2 * i + 1] + Q[2 * j + 1]) / 2;
        const p = document.createElementNS(NS, 'path'), L = 9, W = 8; // chevron length along the strand, half-width across
        p.setAttribute('class', 'ann chev');
        p.setAttribute('d', `M${(x - L * ux - W * uy).toFixed(1)} ${(y - L * uy + W * ux).toFixed(1)}L${x.toFixed(1)} ${y.toFixed(1)}L${(x - L * ux + W * uy).toFixed(1)} ${(y - L * uy - W * ux).toFixed(1)}`);
        this.ann.append(p);
      }
    });
  }
}
