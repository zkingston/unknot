// Strand pickup in 3D, after SnapPy's simplify('pickup'). Seen along a direction F, an over-strand (the run of the
// curve between two under-crossings) passes over everything it meets, so it can be lifted along F through empty
// space, slid sideways above the rest of the curve onto a route that crosses fewer strands, and left there.
// The planner's other moves are local; a jammed curve (every triangle pierced) needs this one.
import { onSphere } from '../kernel/prng';
import { type Scene, cloneScene, insertVertex } from '../kernel/scene';
import { basisFor } from '../diagram/bestView';
import { type View, projectCurves, depthAt } from '../diagram/project';
import { findCrossings, type RawCrossing } from '../diagram/crossings';
import { Op, PhaseKind } from './movelog';
import { type Ctx, phase } from './run';

/** Directions sampled for the view, route grid cells per side, how far above the rest a strand is carried (scene units). */
const VIEWS = 8, GRID = 64, CLEAR = 0.05;
/** Strands tried per pickup (longest first), and pickups per series. */
const TRIES = 40, ROUNDS = 40;

interface Ev { seg: number; t: number; over: boolean }
interface Strand { c: number; u0: Ev; u1: Ev; over: number }

/** Over-strands of every component in this view, longest first, from its crossings X. */
function strands(S: Scene, view: View, X: RawCrossing[]): Strand[] {
  const ev: Ev[][] = S.map(() => []);
  for (const x of X) {
    const aOver = depthAt(S, view, x.a.comp, x.a.seg, x.a.t) > depthAt(S, view, x.b.comp, x.b.seg, x.b.t);
    ev[x.a.comp].push({ seg: x.a.seg, t: x.a.t, over: aOver }); ev[x.b.comp].push({ seg: x.b.seg, t: x.b.t, over: !aOver });
  }
  const out: Strand[] = [];
  ev.forEach((E, c) => {
    E.sort((p, q) => p.seg + p.t - (q.seg + q.t));
    const m = E.length, i0 = E.findIndex((e) => !e.over);
    if (i0 < 0) return;
    for (let i = 0; i < m;) {
      let j = 1; while (E[(i0 + i + j) % m].over) j++;
      if (j > 2) out.push({ c, u0: E[(i0 + i) % m], u1: E[(i0 + i + j) % m], over: j - 1 });
      i += j;
    }
  });
  return out.sort((a, b) => b.over - a.over);
}

const cross2 = (ax: number, ay: number, bx: number, by: number, cx: number, cy: number) => (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
/** Do open segments ab and cd cross? */
function segCross(ax: number, ay: number, bx: number, by: number, cx: number, cy: number, dx: number, dy: number) {
  const d1 = cross2(cx, cy, dx, dy, ax, ay), d2 = cross2(cx, cy, dx, dy, bx, by), d3 = cross2(ax, ay, bx, by, cx, cy), d4 = cross2(ax, ay, bx, by, dx, dy);
  return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}

/**
 * A route p -> q in the plane crossing few of the segments `rest` (x0 y0 x1 y1 quadruples): Dijkstra over grid cell
 * centres, cost = crossings (+ a little length), then the farthest straight shortcuts that cross no more.
 * Returns the interior corners and the route's crossing count.
 */
function route(px: number, py: number, qx: number, qy: number, rest: number[]): { pts: number[]; crossings: number } {
  let x0 = Math.min(px, qx), x1 = Math.max(px, qx), y0 = Math.min(py, qy), y1 = Math.max(py, qy);
  for (let i = 0; i < rest.length; i += 2) { x0 = Math.min(x0, rest[i]); x1 = Math.max(x1, rest[i]); y0 = Math.min(y0, rest[i + 1]); y1 = Math.max(y1, rest[i + 1]); }
  const pad = 0.05 * Math.max(x1 - x0, y1 - y0); x0 -= pad; x1 += pad; y0 -= pad; y1 += pad;
  const h = Math.max(x1 - x0, y1 - y0) / GRID, N = GRID;
  const cx = (i: number) => x0 + (i + 0.5) * h, cy = (j: number) => y0 + (j + 0.5) * h;
  // segments by the cells their boxes cover
  const bucket: number[][] = Array.from({ length: N * N }, () => []);
  for (let s = 0; s < rest.length; s += 4) {
    const i0 = Math.max(0, Math.floor((Math.min(rest[s], rest[s + 2]) - x0) / h)), i1 = Math.min(N - 1, Math.floor((Math.max(rest[s], rest[s + 2]) - x0) / h));
    const j0 = Math.max(0, Math.floor((Math.min(rest[s + 1], rest[s + 3]) - y0) / h)), j1 = Math.min(N - 1, Math.floor((Math.max(rest[s + 1], rest[s + 3]) - y0) / h));
    for (let i = i0; i <= i1; i++) for (let j = j0; j <= j1; j++) bucket[j * N + i].push(s);
  }
  const countIn = (ax: number, ay: number, bx: number, by: number, cells: number[]) => {
    const seen = new Set<number>(); let k = 0;
    for (const c of cells) for (const s of bucket[c]) if (!seen.has(s)) { seen.add(s); if (segCross(ax, ay, bx, by, rest[s], rest[s + 1], rest[s + 2], rest[s + 3])) k++; }
    return k;
  };
  const countAll = (ax: number, ay: number, bx: number, by: number) => { let k = 0; for (let s = 0; s < rest.length; s += 4) if (segCross(ax, ay, bx, by, rest[s], rest[s + 1], rest[s + 2], rest[s + 3])) k++; return k; };
  const cell = (x: number, y: number) => Math.min(N - 1, Math.max(0, Math.floor((y - y0) / h))) * N + Math.min(N - 1, Math.max(0, Math.floor((x - x0) / h)));
  const src = cell(px, py), dst = cell(qx, qy);
  // Dijkstra (binary heap; ties by cell index, so the result is reproducible)
  const dist = new Float64Array(N * N).fill(Infinity), prev = new Int32Array(N * N).fill(-1), heap: [number, number][] = [];
  const push = (d: number, v: number) => { heap.push([d, v]); let i = heap.length - 1; while (i > 0) { const p = (i - 1) >> 1; if (heap[p][0] < d || (heap[p][0] === d && heap[p][1] < v)) break; heap[i] = heap[p]; i = p; } heap[i] = [d, v]; };
  const pop = () => { const top = heap[0], last = heap.pop()!; if (heap.length) { let i = 0; for (;;) { const l = 2 * i + 1, r = l + 1; let m = i; const lt = (a: [number, number], b: [number, number]) => a[0] < b[0] || (a[0] === b[0] && a[1] < b[1]); if (l < heap.length && lt(heap[l], m === i ? last : heap[m])) m = l; if (r < heap.length && lt(heap[r], m === i ? last : heap[m])) m = r; if (m === i) break; heap[i] = heap[m]; i = m; } heap[i] = last; } return top; };
  dist[src] = 0; push(0, src);
  while (heap.length) {
    const [d, v] = pop();
    if (d > dist[v]) continue;
    if (v === dst) break;
    const i = v % N, j = (v - i) / N;
    for (const [di, dj] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
      const a = i + di, b = j + dj;
      if (a < 0 || b < 0 || a >= N || b >= N) continue;
      const w = b * N + a, nd = d + countIn(cx(i), cy(j), cx(a), cy(b), [v, w]) + 1e-3;
      if (nd < dist[w]) { dist[w] = nd; prev[w] = v; push(nd, w); }
    }
  }
  const path: number[] = [qx, qy];
  for (let v = prev[dst]; v >= 0 && v !== src; v = prev[v]) path.unshift(cx(v % N), cy(Math.floor(v / N)));
  path.unshift(px, py);
  // farthest shortcuts that cross no more than the path they replace
  const k = path.length / 2, legs = Array.from({ length: k - 1 }, (_, a) => countAll(path[2 * a], path[2 * a + 1], path[2 * a + 2], path[2 * a + 3]));
  const pts: number[] = [];
  let crossings = 0;
  for (let a = 0; a < k - 1;) {
    let b = k - 1;
    for (; b > a + 1; b--) {
      let along = 0; for (let e = a; e < b; e++) along += legs[e];
      if (countAll(path[2 * a], path[2 * a + 1], path[2 * b], path[2 * b + 1]) <= along) break;
    }
    crossings += countAll(path[2 * a], path[2 * a + 1], path[2 * b], path[2 * b + 1]);
    if (b < k - 1) pts.push(path[2 * b], path[2 * b + 1]);
    a = b;
  }
  return { pts, crossings };
}

/**
 * As SnapPy does on one diagram: fix the view (of VIEWS random directions, the one with the fewest crossings), then
 * pick up over-strands (lifting along +F) and under-strands (along −F) in turn while either still shortens one.
 * Returns the number of records logged.
 */
export function* pickup(ctx: Ctx): Generator<boolean, number> {
  const F = new Float64Array(3);
  let view: View | null = null, fewest = Infinity;
  for (let k = 0; k < VIEWS; k++) {
    onSphere(ctx.rng, 1, F);
    const v = basisFor([F[0], F[1], F[2]]), x = findCrossings(projectCurves(ctx.S, v)).length;
    if (x < fewest) { fewest = x; view = v; }
  }
  if (!view) return 0;
  // each pickup sees the view turned by ~1e-3 rad: the last one's risers (parallel to F) must not project to points
  const base = view.F;
  let logged = 0;
  for (let round = 0, idle = 0; round < ROUNDS && idle < 2; round++) {
    onSphere(ctx.rng, 1e-3, F);
    const s2 = round % 2 ? -1 : 1, v = basisFor([s2 * base[0] + F[0], s2 * base[1] + F[1], s2 * base[2] + F[2]]);
    const n = yield* pickOnce(ctx, v);
    logged += n; idle = n ? 0 : idle + 1;
  }
  return logged;
}

/**
 * In this view, pick up the first of the TRIES longest over-strands that has a route crossing fewer strands and a
 * legal plan: split its two end edges just past its bounding under-crossings, lift everything between along F to
 * CLEAR above the rest, then slide it, at that height, onto the route (ends straight above the split points).
 * The plan runs on a copy and is committed (state and records) only if every step passes the planner's move check.
 * Returns the number of records logged.
 */
function* pickOnce(ctx: Ctx, view: View): Generator<boolean, number> {
  const { S, K, p } = ctx, Q = projectCurves(S, view), X = findCrossings(Q);
  // crossing parameters on edge `seg` of component c
  const on = (c: number, seg: number) => X.flatMap((x) => [...(x.a.comp === c && x.a.seg === seg ? [x.a.t] : []), ...(x.b.comp === c && x.b.seg === seg ? [x.b.t] : [])]);
  for (const st of strands(S, view, X).slice(0, TRIES)) {
    const { c, u0, u1 } = st, P = S[c], n = P.length / 3;
    // split points: halfway from the under-crossing to the next event on its edge (or the edge end)
    let after = 1, before = 0;
    for (const x of on(c, u0.seg)) if (x > u0.t && x < after) after = x;
    for (const x of on(c, u1.seg)) if (x < u1.t && x > before) before = x;
    const tp = 0.5 * (u0.t + after), tq = 0.5 * (before + u1.t);
    if (u0.seg === u1.seg && !(tp < tq)) continue;
    // a split point must not land on a vertex (zero-length edges)
    const len = (seg: number) => { const j = 3 * ((seg + 1) % n), i = 3 * seg; return Math.sqrt((P[j] - P[i]) ** 2 + (P[j + 1] - P[i + 1]) ** 2 + (P[j + 2] - P[i + 2]) ** 2); };
    if ((1 - tp) * len(u0.seg) < 1e-6 || tq * len(u1.seg) < 1e-6 || (tp - u0.t) * len(u0.seg) < 1e-6 || (u1.t - tq) * len(u1.seg) < 1e-6) continue;
    const at = (seg: number, t: number) => { const j = (seg + 1) % n; return [0, 1, 2].map((x) => P[3 * seg + x] + t * (P[3 * j + x] - P[3 * seg + x])); };
    const p3 = at(u0.seg, tp), q3 = at(u1.seg, tq);
    const { R, U, F: D } = view, pr = (v: number[]) => [v[0] * R[0] + v[1] * R[1] + v[2] * R[2], v[0] * U[0] + v[1] * U[1] + v[2] * U[2]];
    const [px, py] = pr(p3), [qx, qy] = pr(q3);
    // the rest of the curve in the plane: every edge but the strand's (its end edges only up to p / from q)
    const rest: number[] = [], span = (u1.seg - u0.seg + n) % n;
    Q.forEach((C, l) => {
      const m = C.length / 2;
      for (let e = 0; e < m; e++) {
        const f = (e + 1) % m, ax = C[2 * e], ay = C[2 * e + 1], bx = C[2 * f], by = C[2 * f + 1], inside = (e - u0.seg + n) % n;
        if (l !== c) rest.push(ax, ay, bx, by);
        else if (u0.seg === u1.seg && e === u0.seg) rest.push(ax, ay, px, py, qx, qy, bx, by);
        else if (e === u0.seg) rest.push(ax, ay, px, py);
        else if (e === u1.seg) rest.push(qx, qy, bx, by);
        else if (!(inside > 0 && inside < span)) rest.push(ax, ay, bx, by);
      }
    });
    const r = route(px, py, qx, qy, rest);
    if (r.crossings >= st.over) continue;
    // plan on a copy; commit only if every step is legal
    const T = cloneScene(S), recs: number[][] = [];
    const ins = (v: number, x: number[]) => { insertVertex(T, c, v, x[0], x[1], x[2]); recs.push([Op.Insert, c, v, x[0], x[1], x[2]]); };
    // split the end edges (the later index first, so the earlier stays put)
    let ip: number, iq: number;
    if (u0.seg === u1.seg) { ins(u0.seg + 1, q3); ins(u0.seg + 1, p3); ip = u0.seg + 1; iq = u0.seg + 2; }
    else if (u1.seg > u0.seg) { ins(u1.seg + 1, q3); ins(u0.seg + 1, p3); ip = u0.seg + 1; iq = u1.seg + 2; }
    else { ins(u0.seg + 1, p3); ins(u1.seg + 1, q3); ip = u0.seg + 2; iq = u1.seg + 1; }
    // the strand's vertices strictly between p and q, in order; split its longest edge until there are enough
    const between = () => { const m = T[c].length / 3, out: number[] = []; for (let v = (ip + 1) % m; v !== iq; v = (v + 1) % m) out.push(v); return out; };
    const need = r.pts.length / 2 + 2;
    for (let ids = between(); ids.length < need; ids = between()) {
      const Pc = T[c], m = Pc.length / 3, chain = [ip, ...ids, iq];
      let best = 0, bl = -1;
      for (let k = 0; k + 1 < chain.length; k++) { const a = 3 * chain[k], b = 3 * chain[k + 1], l = (Pc[b] - Pc[a]) ** 2 + (Pc[b + 1] - Pc[a + 1]) ** 2 + (Pc[b + 2] - Pc[a + 2]) ** 2; if (l > bl) { bl = l; best = k; } }
      const a = 3 * chain[best], b = 3 * chain[best + 1], at2 = chain[best + 1] === 0 ? m : chain[best + 1]; // before the edge's end vertex
      ins(at2, [0.5 * (Pc[a] + Pc[b]), 0.5 * (Pc[a + 1] + Pc[b + 1]), 0.5 * (Pc[a + 2] + Pc[b + 2])]);
      if (ip >= at2) ip++;
      if (iq >= at2) iq++;
    }
    const ids = between(), lifted = new Set(ids);
    const dep = (Q2: Float64Array, v: number) => Q2[3 * v] * D[0] + Q2[3 * v + 1] * D[1] + Q2[3 * v + 2] * D[2];
    // carried just above every other edge whose shadow meets the corridor (old strand, new route, padded): a segment
    // is never higher than its higher end, so that clears all it can pass over without stacking up the whole curve
    const xs = [px, qx, ...r.pts.filter((_, a) => a % 2 === 0)], ys = [py, qy, ...r.pts.filter((_, a) => a % 2 === 1)];
    for (const v of ids) { const [x, y] = pr([T[c][3 * v], T[c][3 * v + 1], T[c][3 * v + 2]]); xs.push(x); ys.push(y); }
    const pad = 2 * CLEAR, bx0 = Math.min(...xs) - pad, bx1 = Math.max(...xs) + pad, by0 = Math.min(...ys) - pad, by1 = Math.max(...ys) + pad;
    let top = -Infinity;
    T.forEach((Q2, l) => {
      const m = Q2.length / 3;
      for (let v = 0; v < m; v++) {
        const w = (v + 1) % m;
        if (l === c && (lifted.has(v) || lifted.has(w))) continue; // the strand's own edges
        const [ax, ay] = pr([Q2[3 * v], Q2[3 * v + 1], Q2[3 * v + 2]]), [ex, ey] = pr([Q2[3 * w], Q2[3 * w + 1], Q2[3 * w + 2]]);
        if (Math.max(ax, ex) < bx0 || Math.min(ax, ex) > bx1 || Math.max(ay, ey) < by0 || Math.min(ay, ey) > by1) continue;
        top = Math.max(top, dep(Q2, v), dep(Q2, w));
      }
    });
    for (const v of [ip, iq]) top = Math.max(top, dep(T[c], v));
    // vertex k of the strand rides at H + k·step: non-adjacent strand edges keep apart in height while they slide
    const H = top + CLEAR, step = CLEAR / ids.length, hk = (k: number) => H + k * step, target = new Float64Array(3);
    const moveTo = (v: number, x: number[]) => {
      target[0] = x[0]; target[1] = x[1]; target[2] = x[2];
      if (K.maxLegalMove(T, c, v, target, p.tau, 0) < 1) return false;
      T[c].set(x, 3 * v); recs.push([Op.Move, c, v, x[0], x[1], x[2]]);
      return true;
    };
    // lift along F to height H
    let ok = ids.every((v, k) => { const Pc = T[c], h = hk(k) - dep(Pc, v); return moveTo(v, [Pc[3 * v] + h * D[0], Pc[3 * v + 1] + h * D[1], Pc[3 * v + 2] + h * D[2]]); });
    if (ok) {
      // at H: the ends straight above the split points, the route's corners, the spare vertices spread on its last leg
      const plane = [px, py, ...r.pts], k = ids.length, flat = [...plane];
      const lx = plane[plane.length - 2], ly = plane[plane.length - 1], spare = k - 1 - plane.length / 2;
      for (let s2 = 1; s2 <= spare; s2++) flat.push(lx + (s2 / (spare + 1)) * (qx - lx), ly + (s2 / (spare + 1)) * (qy - ly));
      flat.push(qx, qy);
      const spots = ids.map((_, a) => [0, 1, 2].map((x) => flat[2 * a] * R[x] + flat[2 * a + 1] * U[x] + hk(a) * D[x]));
      ok = [0, k - 1, ...ids.map((_, a) => a).slice(1, -1)].every((a) => moveTo(ids[a], spots[a]));
    }
    yield false;
    if (!ok) continue;
    phase(ctx, PhaseKind.pickup);
    for (let l = 0; l < S.length; l++) S[l] = T[l];
    for (const q of recs) ctx.log.push(q[0], q[1], q[2], q[3], q[4], q[5]);
    return recs.length;
  }
  return 0;
}
