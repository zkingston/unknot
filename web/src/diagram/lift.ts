// Diagram -> 3D Scene whose projection is the drawing: resample, keep vertices off crossings, set heights.
import { type Diagram, events, extent2 } from './types';
import { findCrossings } from './crossings';
import { type Scene } from '../kernel/scene';
import { touches } from '../kernel/energy';

/** Height of a crossing (scene units, the drawing spans 2), and the most it may be per unit of arc to the nearer
 *  neighbouring crossing: crowded crossings sit lower, so strands ramp gently and simplify to fewer sticks. */
const LIFT_H = 0.35, RAMP = 0.5;

interface Vtx { s: number; seg: number; z: number; fixed: boolean }

function arcLengths(P: Float64Array) {
  const n = P.length / 2, L = new Float64Array(n), S = new Float64Array(n + 1);
  for (let i = 0; i < n; i++) {
    const j = i + 1 === n ? 0 : i + 1, dx = P[2 * j] - P[2 * i], dy = P[2 * j + 1] - P[2 * i + 1];
    L[i] = Math.sqrt(dx * dx + dy * dy); S[i + 1] = S[i] + L[i];
  }
  return { L, S };
}

/** Lift every component in one shared frame. Throws if a crossing is flat or validation fails. */
export function lift(d: Diagram, h = LIFT_H, validate = true): Scene {
  if (d.crossings.some((X) => X.flat)) throw new Error('flat crossings must be resolved before lifting');
  const { x0, y0, x1, y1, ext } = extent2(d.comps);
  const geo = d.comps.map(arcLengths);
  const ev = d.comps.map((_, c) => events(d, c).map((e) => {
    const X = d.crossings[e.x], r = e.over ? X.over : X.under, g = geo[c];
    return { s: g.S[r.seg] + r.t * g.L[r.seg], seg: r.seg, over: e.over };
  }));
  // target spacing max(ext/2000, min(ext/150, 0.25 × arc)), taken per arc between consecutive events so one
  // tight spot does not oversample the whole curve
  const spacingOf = (arc: number) => Math.max(ext / 2000, Math.min(ext / 150, 0.25 * arc));
  const scene: Scene = [], segMaps: Int32Array[] = [];
  const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2, sc = 2 / ext;
  d.comps.forEach((P, c) => {
    const { L, S } = geo[c], n = L.length, Lc = S[n], E = ev[c];
    const segSpacing = new Float64Array(n).fill(ext / 150);
    for (let k = 0; k < E.length; k++) {
      const a = E[k].s, b = k + 1 < E.length ? E[k + 1].s : E[0].s + Lc, sp = spacingOf(E.length > 1 ? b - a : Lc);
      for (let i = 0; i < n; i++) { const sa = S[i], sb = S[i + 1]; if ((sb >= a && sa <= b) || (sb + Lc >= a && sa + Lc <= b)) segSpacing[i] = Math.min(segSpacing[i], sp); }
    }
    const pieces = Array.from(L, (l, i) => Math.max(1, Math.ceil(l / segSpacing[i])));
    const grid = () => {
      const V: Vtx[] = [];
      for (let i = 0; i < n; i++) for (let j = 0; j < pieces[i]; j++) V.push({ s: S[i] + (j / pieces[i]) * L[i], seg: i, z: 0, fixed: false });
      return V;
    };
    // at least 3 grid vertices strictly inside every arc between consecutive events
    let V = grid();
    for (let it = 0; it < 12 && E.length > 1; it++) {
      let ok = true;
      for (let k = 0; k < E.length; k++) {
        const a = E[k].s, b = k + 1 < E.length ? E[k + 1].s : E[0].s + Lc;
        let cnt = 0;
        for (const v of V) { const s = v.s < a ? v.s + Lc : v.s; if (s > a && s < b) cnt++; }
        if (cnt < 3) {
          ok = false;
          for (let i = 0; i < n; i++) { const sa = S[i], sb = S[i + 1]; if ((sb > a && sa < b) || (sb + Lc > a && sa + Lc < b)) pieces[i] *= 2; }
        }
      }
      if (ok) break;
      V = grid();
    }
    // never a grid vertex on (or within 5% of a piece of) a crossing: per segment, the smallest piece count (not
    // below the current one) that clears all of its crossings
    const ts = new Map<number, number[]>();
    for (const e of E) { const t = (e.s - S[e.seg]) / L[e.seg], l = ts.get(e.seg); if (l) l.push(t); else ts.set(e.seg, [t]); }
    let moved = false;
    for (const [seg, T] of ts) {
      const clear = (p: number) => T.every((t) => { const f = t * p; return Math.abs(f - Math.round(f)) >= 0.05; });
      for (let p = pieces[seg]; p <= 4 * pieces[seg] + 64; p++) if (clear(p)) { moved ||= p !== pieces[seg]; pieces[seg] = p; break; }
    }
    if (moved) V = grid();
    // two vertices at ±δ around each event, at the event's height: the full h, or less where events crowd (at most
    // RAMP × the arc to the nearer neighbouring event), so a strand alternating over and under ramps gently
    const prev = E.map((e, k) => (E.length > 1 ? (k > 0 ? e.s - E[k - 1].s : e.s + Lc - E[E.length - 1].s) : Lc));
    const next = E.map((e, k) => (E.length > 1 ? (k + 1 < E.length ? E[k + 1].s - e.s : E[0].s + Lc - e.s) : Lc));
    const zs = E.map((e, k) => (e.over ? 1 : -1) * Math.min(1, (RAMP * Math.min(prev[k], next[k]) * sc) / h));
    const win = E.map((e, k) => {
      const piece = L[e.seg] / pieces[e.seg], jL = Math.floor((e.s - S[e.seg]) / piece);
      const left = S[e.seg] + jL * piece, right = Math.min(S[e.seg + 1], left + piece);
      return Math.min(0.25 * piece, 0.5 * (e.s - left), 0.5 * (right - e.s), 0.25 * prev[k], 0.25 * next[k]);
    });
    E.forEach((e, k) => {
      for (const sgn of [-1, 1]) {
        let s = e.s + sgn * win[k];
        if (s < 0) s += Lc; else if (s >= Lc) s -= Lc;
        let seg = 0;
        while (seg + 1 < n && S[seg + 1] <= s) seg++;
        V.push({ s, seg, z: zs[k], fixed: true });
      }
    });
    V.sort((a, b) => a.s - b.s);
    // heights
    if (!E.length) for (const v of V) v.z = 0.1 * (1 - 4 * Math.abs(v.s / Lc - 0.5));
    else for (const v of V) {
      if (v.fixed) continue;
      // the event window just before v (cyclically)
      let k = E.length - 1;
      for (let q = 0; q < E.length; q++) if (E[q].s <= v.s) k = q;
      const k2 = (k + 1) % E.length;
      let a = E[k].s + win[k], b = E[k2].s - win[k2], s = v.s;
      if (b <= a) b += Lc;
      if (s < a) s += Lc;
      // over to under eases (smoothstep); over to over dips 10% (flat at both ends), never exactly level: an exactly
      // level run puts many edges in one plane, the degenerate case of the planner's tests
      const u = (s - a) / (b - a), za = zs[k], zb = zs[k2], w = u * (1 - u);
      v.z = za === zb ? za * (1 - 1.6 * w * w) : za + (zb - za) * u * u * (3 - 2 * u);
    }
    const out = new Float64Array(3 * V.length);
    V.forEach((v, i) => {
      const f = L[v.seg] > 0 ? (v.s - S[v.seg]) / L[v.seg] : 0, j = (v.seg + 1) % n;
      const x = P[2 * v.seg] + f * (P[2 * j] - P[2 * v.seg]), y = P[2 * v.seg + 1] + f * (P[2 * j + 1] - P[2 * v.seg + 1]);
      out[3 * i] = (x - cx) * sc; out[3 * i + 1] = (y - cy) * sc; out[3 * i + 2] = v.z * h;
    });
    scene.push(out);
    segMaps.push(Int32Array.from(V, (v) => v.seg));
  });
  if (validate) validateLift(d, scene, segMaps);
  return scene;
}

/** Scene clearance > 0 and its projection's crossings match the diagram's one for one, over/under included. */
function validateLift(d: Diagram, scene: Scene, segMaps: Int32Array[]) {
  if (touches(scene)) throw new Error('lift: polygon touches itself');
  const proj = scene.map((P) => { const Q = new Float64Array((P.length / 3) * 2); for (let i = 0; i < P.length / 3; i++) { Q[2 * i] = P[3 * i]; Q[2 * i + 1] = P[3 * i + 1]; } return Q; });
  const found = findCrossings(proj);
  if (found.length !== d.crossings.length) throw new Error(`lift: projection has ${found.length} crossings, diagram ${d.crossings.length}`);
  const want = new Map<string, number[]>();
  d.crossings.forEach((X, k) => { const key = `${X.under.comp}:${X.under.seg}|${X.over.comp}:${X.over.seg}`, l = want.get(key); if (l) l.push(k); else want.set(key, [k]); });
  const zAt = (c: number, seg: number, t: number) => { const P = scene[c], n = P.length / 3, j = (seg + 1) % n; return P[3 * seg + 2] + t * (P[3 * j + 2] - P[3 * seg + 2]); };
  for (const F of found) {
    const za = zAt(F.a.comp, F.a.seg, F.a.t), zb = zAt(F.b.comp, F.b.seg, F.b.t);
    const [o, u] = za > zb ? [F.a, F.b] : [F.b, F.a];
    const l = want.get(`${u.comp}:${segMaps[u.comp][u.seg]}|${o.comp}:${segMaps[o.comp][o.seg]}`);
    if (!l || !l.length || !(Math.abs(za - zb) > 1e-9)) throw new Error('lift: projected crossing does not match the diagram (position or over/under)');
    l.pop();
  }
}
