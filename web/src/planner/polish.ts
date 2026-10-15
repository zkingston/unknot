// Playback polish: consecutive single-vertex records are grouped into one simultaneous straight-line motion
// when exact continuous collision detection shows that motion never brings two non-adjacent edges closer than
// a margin. Groups only change how the verified log is animated; the MoveLog itself is untouched.
import { type Scene, cloneScene, prevIdx, nextIdx, identityIds, removeVertex } from '../kernel/scene';
import { segDist2 } from '../kernel/segment';
import { MoveLog, Op, REC, applyRecord } from './movelog';

type V = [number, number, number];

/**
 * Do edges (a, b) and (c, d), each endpoint moving linearly over t in [0, 1], stay at least `margin` apart?
 * Contact needs coplanarity: the triple product is a cubic in t; check the distance at its roots, at its
 * critical points (double roots) and at t = 1.
 */
export function edgesClear(a0: V, a1: V, b0: V, b1: V, c0: V, c1: V, d0: V, d1: V, margin: number): boolean {
  const sub = (p: V, q: V): V => [p[0] - q[0], p[1] - q[1], p[2] - q[2]];
  const crs = (p: V, q: V): V => [p[1] * q[2] - p[2] * q[1], p[2] * q[0] - p[0] * q[2], p[0] * q[1] - p[1] * q[0]];
  const dot = (p: V, q: V) => p[0] * q[0] + p[1] * q[1] + p[2] * q[2];
  const add = (p: V, q: V): V => [p[0] + q[0], p[1] + q[1], p[2] + q[2]];
  // u = b − a, w = c − a, z = d − a, each u0 + t·u1
  const u0 = sub(b0, a0), u1 = sub(sub(b1, a1), u0), w0 = sub(c0, a0), w1 = sub(sub(c1, a1), w0), z0 = sub(d0, a0), z1 = sub(sub(d1, a1), z0);
  const x0 = crs(w0, z0), x1 = add(crs(w0, z1), crs(w1, z0)), x2 = crs(w1, z1);
  const k0 = dot(u0, x0), k1 = dot(u1, x0) + dot(u0, x1), k2 = dot(u1, x1) + dot(u0, x2), k3 = dot(u1, x2);
  const scale = Math.abs(k0) + Math.abs(k1) + Math.abs(k2) + Math.abs(k3);
  const L = Math.sqrt(dot(u0, u0) * dot(w0, w0) * dot(z0, z0)) + 1e-300;
  if (scale <= 1e-12 * L) return false; // coplanar throughout: give up on this group
  const f = (t: number) => ((k3 * t + k2) * t + k1) * t + k0;
  const ts: number[] = [1];
  // critical points of the cubic split [0, 1] into monotone pieces
  // roots of f' = A t² + B t + C, by the cancellation-free form q = −(B + sign(B)√D)/2, roots q/A and C/q
  const crit: number[] = [];
  const A = 3 * k3, B = 2 * k2, C = k1, D = B * B - 4 * A * C;
  if (D >= 0) {
    const q = -0.5 * (B + (B < 0 ? -1 : 1) * Math.sqrt(D));
    if (A !== 0) crit.push(q / A);
    if (q !== 0) crit.push(C / q);
  }
  const knots = [0, ...crit.filter((t) => t > 0 && t < 1).sort((p, q) => p - q), 1];
  for (let i = 0; i + 1 < knots.length; i++) {
    let lo = knots[i], hi = knots[i + 1], flo = f(lo);
    const fhi = f(hi);
    if (i > 0) ts.push(lo);
    if ((flo < 0) === (fhi < 0) && flo !== 0 && fhi !== 0) continue;
    for (let it = 0; it < 60; it++) { const mid = 0.5 * (lo + hi), fm = f(mid); if ((fm < 0) === (flo < 0)) { lo = mid; flo = fm; } else hi = mid; }
    ts.push(0.5 * (lo + hi));
  }
  const m2 = margin * margin;
  for (const t of ts) {
    const at = (p: V, q: V) => [p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]), p[2] + t * (q[2] - p[2])];
    const a = at(a0, a1), b = at(b0, b1), c = at(c0, c1), d = at(d0, d1);
    if (segDist2(a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2]) < m2) return false;
  }
  return true;
}

/** Are the boxes swept by the two moving edges more than `m` apart? (Linear motion stays in the hull.) */
function apart(a0: V, a1: V, b0: V, b1: V, c0: V, c1: V, d0: V, d1: V, m: number) {
  for (let x = 0; x < 3; x++) {
    const lo1 = Math.min(a0[x], a1[x], b0[x], b1[x]), hi1 = Math.max(a0[x], a1[x], b0[x], b1[x]);
    const lo2 = Math.min(c0[x], c1[x], d0[x], d1[x]), hi2 = Math.max(c0[x], c1[x], d0[x], d1[x]);
    if (lo1 > hi2 + m || lo2 > hi1 + m) return true;
  }
  return false;
}

interface Member { from: V; to: V; deleted: boolean }

/**
 * Streams records and reports where groups end. A group never repeats a vertex, keeps every deleted vertex's
 * neighbours out of the group (so its midpoint is the same before and after), and grows only while every edge
 * pair touched by the newest member stays clear under the group's simultaneous motion.
 */
export class Grouper {
  private G0: Scene;           // state at the start of the open group
  private alive: number[][];   // current index -> G0 index, per component
  private members = new Map<number, Member>(); // key comp * 2^21 + G0 index
  private start = 0;           // first record of the open group
  private pending: number[] | null = null; // one record of lookahead (Move waiting for a possible Delete)
  private pendingAt = 0;
  private next = 0;            // index of the next record to arrive
  ends: number[] = [];         // exclusive end of each closed group
  constructor(initial: Scene, private margin = 1e-5) {
    this.G0 = cloneScene(initial);
    this.alive = identityIds(initial);
  }

  /** Feed records; returns the group ends closed by them. */
  push(recs: Float64Array): number[] {
    const before = this.ends.length;
    for (let o = 0; o < recs.length; o += REC) {
      const r = Array.from(recs.subarray(o, o + REC)), at = this.next++;
      if (this.pending) {
        const p = this.pending, pa = this.pendingAt; this.pending = null;
        if (r[0] === Op.Delete && r[1] === p[1] && r[2] === p[2]) { this.unit(pa, [p, r]); continue; }
        this.unit(pa, [p]);
      }
      if (r[0] === Op.Move) { this.pending = r; this.pendingAt = at; }
      else this.unit(at, [r]);
    }
    return this.ends.slice(before);
  }
  /** Group ends if the log ended now, without closing anything (a stopped run may still be extended). */
  peekFinish(): number[] {
    const out: number[] = [];
    if (this.pending) { if (this.pendingAt > this.start) out.push(this.pendingAt); out.push(this.next); }
    else if (this.next > this.start) out.push(this.next);
    return out;
  }
  /** Close everything (end of the log). */
  finish(): number[] {
    const before = this.ends.length;
    if (this.pending) { this.unit(this.pendingAt, [this.pending]); this.pending = null; }
    this.close(this.next);
    return this.ends.slice(before);
  }

  private key(c: number, g: number) { return c * 2097152 + g; }
  private pos(c: number, g: number): V { const P = this.G0[c]; return [P[3 * g], P[3 * g + 1], P[3 * g + 2]]; }
  /** Endpoint motion of G0 vertex g of comp c under the open group. */
  private motion(c: number, g: number): [V, V] { const m = this.members.get(this.key(c, g)); return m ? [m.from, m.to] : [this.pos(c, g), this.pos(c, g)]; }

  /** A unit is a Phase, a lone Move, a Move+Delete pair, a stray Delete or an Insert. */
  private unit(at: number, rs: number[][]) {
    const r = rs[0];
    if (r[0] === Op.Phase) return; // stats only: rides along in whatever group is open
    // a stray Delete (outside a Move+Delete pair) or an Insert is a group of its own
    if (r[0] === Op.Delete || r[0] === Op.Insert) { this.close(at); applyRecord(this.G0, Float64Array.from(r), 0); this.alive = identityIds(this.G0); this.ends.push(at + 1); this.start = at + 1; return; }
    if (!this.tryAdd(r, rs.length === 2)) { this.close(at); if (!this.tryAdd(r, rs.length === 2)) throw new Error('polish: a single move failed its own check'); }
    if (rs.length === 2) this.alive[r[1]].splice(r[2], 1);
  }

  private tryAdd(r: number[], deleting: boolean): boolean {
    const c = r[1], g = this.alive[c][r[2]], n = this.G0[c].length / 3, k = this.key(c, g);
    if (this.members.has(k)) return false;
    const ga = prevIdx(g, n), gc = nextIdx(g, n), ma = this.members.get(this.key(c, ga)), mc = this.members.get(this.key(c, gc));
    if (ma?.deleted || mc?.deleted) return false;          // its edges no longer exist as in G0
    if (deleting && (ma || mc)) return false;              // a deleted vertex's midpoint must not move
    const m: Member = { from: this.pos(c, g), to: [r[3], r[4], r[5]], deleted: deleting };
    if (this.members.size === 0) { this.members.set(k, m); return true; } // one move alone is the planner's own verified move
    this.members.set(k, m);
    // every pair (edge at g, other edge) under the simultaneous motion
    for (const e of [ga, g]) {
      const e1 = nextIdx(e, n), [a0, a1] = this.motion(c, e), [b0, b1] = this.motion(c, e1);
      for (let l = 0; l < this.G0.length; l++) {
        const m2 = this.G0[l].length / 3;
        for (let f = 0; f < m2; f++) {
          const f1 = nextIdx(f, m2);
          if (l === c && (f === e || f1 === e || f === e1 || f1 === e1)) continue;
          if (l === c && f === (e === g ? ga : g)) continue; // the other edge at g (adjacent)
          const [c0, c1] = this.motion(l, f), [d0, d1] = this.motion(l, f1);
          if (apart(a0, a1, b0, b1, c0, c1, d0, d1, this.margin)) continue;
          if (!edgesClear(a0, a1, b0, b1, c0, c1, d0, d1, this.margin)) { this.members.delete(k); return false; }
        }
      }
    }
    return true;
  }

  /** Close the open group at record `at` (exclusive) and advance G0 by its records. */
  private close(at: number) {
    if (at > this.start) {
      // apply the members (G0 indices), then drop the deleted ones from G0 in descending order
      const dels: [number, number][] = [];
      for (const [k, m] of this.members) {
        const c = Math.floor(k / 2097152), g = k % 2097152, P = this.G0[c];
        P[3 * g] = m.to[0]; P[3 * g + 1] = m.to[1]; P[3 * g + 2] = m.to[2];
        if (m.deleted) dels.push([c, g]);
      }
      dels.sort((p, q) => q[1] - p[1]);
      for (const [c, g] of dels) removeVertex(this.G0, c, g);
      this.ends.push(at);
    }
    this.members.clear();
    this.alive = identityIds(this.G0);
    this.start = at;
  }
}

/**
 * A keyframe: state, vertex ids (inserted vertices get fresh ones), record, last Phase, index among the original
 * keyframes (−1 once moved), and whether the step into it from its original predecessor inserts vertices (a jump
 * can only move and delete, so it never crosses such a step).
 */
interface Key { S: Scene; ids: Int32Array[]; r: number; ph: number; o: number; ins: boolean }

/** Clearance the shortcut's motions keep (as groups do), and the edge-pair tests one pass may spend (deterministic). */
const SHORTCUT_MARGIN = 1e-5, SHORTCUT_WORK = 1.5e7;
/** Smoothing: shortcut-then-smooth cycles, sweeps per cycle, clearance its changes keep, retries per step, work per cycle. */
const SMOOTH_CYCLES = 2, SMOOTH_ROUNDS = 4, SMOOTH_MARGIN = 0.003, SMOOTH_TRIES = 8, SMOOTH_WORK = 4e6;

/**
 * Shortcut a finished run for playback. Keyframes are the states at group ends; from each kept keyframe, jump to
 * the furthest later one that a single straight simultaneous motion reaches with every non-adjacent edge pair
 * clear (exact CCD, as for groups). Vertices deleted in between slide to their arc-length fraction on the edge between
 * their surviving neighbours and are dropped there, collinear, so the curve is exactly the later keyframe's.
 * Passes repeat while they still remove keyframes (the search is greedy, so a second look finds more). Then
 * smoothing straightens the vertex paths through the kept keyframes, which often lets more of them go (one more pass).
 * Work is budgeted in edge-pair tests, setup included, so a long log costs about a second, not several.
 * Returns playback records (each jump: its Moves, then its Deletes, then a copy of the last Phase) and group ends;
 * a step that was neither shortcut nor smoothed keeps its original records.
 */
export function shortcut(initial: Scene, recs: Float64Array, ends: number[], passes = 4): { records: Float64Array; ends: number[] } {
  let keys = keyframes(initial, recs, ends);
  const M = new Motions(keys);
  const shorten = (most: number) => { for (let k = 0; k < most; k++) { const s = select(keys, M); if (s.length >= keys.length) break; keys = s; } };
  shorten(passes);
  for (let c = 0; c < SMOOTH_CYCLES; c++) { smooth(keys, M); shorten(1); }
  return emit(keys, recs, M);
}

/**
 * Straighten vertex paths: move each interior keyframe's vertices part way toward the midpoint of where they are
 * one keyframe before and after. A change stands only if both motions it touches stay clear by `margin`; when one
 * does not, the vertices of the failing edge pair (and their neighbours) keep their place and the rest try again.
 */
function smooth(keys: Key[], M: Motions) {
  M.budget = SMOOTH_WORK;
  for (let it = 0; it < SMOOTH_ROUNDS; it++) for (let k = 1; k + 1 < keys.length; k++) {
    if (M.budget <= 0) return;
    if (keys[k].ins || keys[k + 1].ins) continue;   // its steps must stay the original records
    const A = keys[k - 1], K = keys[k], B = keys[k + 1], next = M.targets(K, B), S0 = K.S;
    const mid = S0.map((P, c) => {
      const W = M.index(A, c), I = K.ids[c], Q = A.S[c], N = next[c], out = new Float64Array(P.length);
      for (let v = 0; v < I.length; v++) { const u = 3 * W[I[v]]; for (let x = 0; x < 3; x++) out[3 * v + x] = 0.5 * (Q[u + x] + N[3 * v + x]); }
      return out;
    });
    /** Hold K's vertex v of comp c (and its neighbours) in place. */
    const hold = (frozen: Uint8Array[], c: number, v: number) => { const n = frozen[c].length; frozen[c][v] = frozen[c][(v + 1) % n] = frozen[c][(v + n - 1) % n] = 1; };
    /** Hold the K vertices an A-index vertex maps to: itself, or the survivors either side of where it was deleted. */
    const holdA = (frozen: Uint8Array[], c: number, a: number) => {
      const W = M.index(K, c), I = A.ids[c], n = I.length;
      if (W[I[a]] >= 0) { hold(frozen, c, W[I[a]]); return; }
      let p = a; while (W[I[p]] < 0) p = (p + n - 1) % n;
      let q = a; while (W[I[q]] < 0) q = (q + 1) % n;
      hold(frozen, c, W[I[p]]); hold(frozen, c, W[I[q]]);
    };
    done: for (const lam of [0.5, 0.25]) {
      const frozen = S0.map((P) => new Uint8Array(P.length / 3));
      for (let t = 0; t < SMOOTH_TRIES; t++) {
        K.S = S0.map((P, c) => P.map((p, i) => (frozen[c][(i / 3) | 0] ? p : p + lam * (mid[c][i] - p))));
        if (!M.clear(A, K, SMOOTH_MARGIN)) { const [c, a, l, b] = M.fail, na = A.S[c].length / 3, nb = A.S[l].length / 3; holdA(frozen, c, a); holdA(frozen, c, (a + 1) % na); holdA(frozen, l, b); holdA(frozen, l, (b + 1) % nb); }
        else if (!M.clear(K, B, SMOOTH_MARGIN)) { const [c, a, l, b] = M.fail; hold(frozen, c, a); hold(frozen, c, (a + 1) % frozen[c].length); hold(frozen, l, b); hold(frozen, l, (b + 1) % frozen[l].length); }
        else { K.o = -1; break done; }
        if (M.budget <= 0) break;
      }
      K.S = S0;
    }
  }
}

/** States at the group ends; keyframe 0 is the initial state. */
function keyframes(initial: Scene, recs: Float64Array, ends: number[]): Key[] {
  const S = cloneScene(initial), ids = identityIds(initial).map((a) => Int32Array.from(a)), next = initial.map((P) => P.length / 3);
  const keys: Key[] = [{ S: cloneScene(S), ids: ids.map((a) => a.slice()), r: 0, ph: -1, o: 0, ins: false }];
  let r = 0, ph = -1;
  for (const e of ends) {
    let ins = false;
    for (; r < e; r++) {
      const o = r * REC, c = recs[o + 1], v = recs[o + 2];
      if (recs[o] === Op.Phase) ph = r;
      else if (recs[o] === Op.Delete) { const a = ids[c], b = new Int32Array(a.length - 1); b.set(a.subarray(0, v)); b.set(a.subarray(v + 1), v); ids[c] = b; }
      else if (recs[o] === Op.Insert) { const a = ids[c], b = new Int32Array(a.length + 1); b.set(a.subarray(0, v)); b[v] = next[c]++; b.set(a.subarray(v), v + 1); ids[c] = b; ins = true; }
      applyRecord(S, recs, r);
    }
    keys.push({ S: cloneScene(S), ids: ids.map((a) => a.slice()), r: e, ph, o: keys.length, ins });
  }
  return keys;
}

/** Straight simultaneous motions between keyframes, checked by exact CCD within a work budget. */
class Motions {
  budget = SHORTCUT_WORK;
  /** The edge pair (comp, index, comp, index in A) that failed the last clear(). */
  fail = [0, 0, 0, 0];
  private where: Int32Array[];
  constructor(keys: Key[]) { this.where = keys[0].S.map((_, c) => new Int32Array(1 + Math.max(...keys.map((k) => Math.max(-1, ...k.ids[c]))))); }

  /** where[c][id] = index of original vertex id in B's component c, or −1. */
  index(B: Key, c: number) { const W = this.where[c]; W.fill(-1); B.ids[c].forEach((id, k) => { W[id] = k; }); return W; }

  /** Where each vertex of A goes to reach B. */
  targets(A: Key, B: Key): Scene {
    return A.S.map((_, c) => {
      const I = A.ids[c], Q = B.S[c], n = I.length, T = new Float64Array(3 * n), W = this.index(B, c);
      let s0 = 0; while (W[I[s0]] < 0) s0++;
      let a = s0;
      do {
        const ja = W[I[a]];
        let b = (a + 1) % n, run = 0;
        while (W[I[b]] < 0) { b = (b + 1) % n; run++; }
        const jb = W[I[b]];
        for (let x = 0; x < 3; x++) T[3 * a + x] = Q[3 * ja + x];
        if (run) {
          // at the same arc-length fraction along the survivors' edge as along the path they leave
          const P = A.S[c], s = [0];
          for (let q = 1; q <= run + 1; q++) {
            const u = 3 * ((a + q - 1) % n), v = 3 * ((a + q) % n), dx = P[v] - P[u], dy = P[v + 1] - P[u + 1], dz = P[v + 2] - P[u + 2];
            s.push(s[q - 1] + Math.sqrt(dx * dx + dy * dy + dz * dz));
          }
          const L = s[run + 1] || 1;
          for (let q = 1; q <= run; q++) { const v = (a + q) % n, f = s[q] / L; for (let x = 0; x < 3; x++) T[3 * v + x] = Q[3 * ja + x] + f * (Q[3 * jb + x] - Q[3 * ja + x]); }
        }
        a = b;
      } while (a !== s0);
      return T;
    });
  }

  /** Is the straight motion A -> targets(A, B) clear by `margin`? Sweep-and-prune on x over the swept boxes. */
  clear(A: Key, B: Key, margin: number): boolean {
    const S = A.S, T = this.targets(A, B), E: [number, number][] = [], lo: number[] = [], hi: number[] = [], moving: boolean[] = [];
    S.forEach((P, c) => {
      const n = P.length / 3, Q = T[c], at = (k: number) => P[3 * k] !== Q[3 * k] || P[3 * k + 1] !== Q[3 * k + 1] || P[3 * k + 2] !== Q[3 * k + 2];
      for (let k = 0; k < n; k++) {
        const k1 = (k + 1) % n;
        E.push([c, k]); moving.push(at(k) || at(k1));
        for (let x = 0; x < 3; x++) {
          const v = [P[3 * k + x], P[3 * k1 + x], Q[3 * k + x], Q[3 * k1 + x]];
          lo.push(Math.min(v[0], v[1], v[2], v[3]) - margin); hi.push(Math.max(v[0], v[1], v[2], v[3]) + margin);
        }
      }
    });
    // the setup (boxes, sort) counts against the budget too
    this.budget -= Math.ceil(E.length * Math.log2(E.length + 1));
    const ord = E.map((_, e) => e).sort((p, q) => lo[3 * p] - lo[3 * q] || p - q);
    const at = (P: Float64Array, k: number): V => [P[3 * k], P[3 * k + 1], P[3 * k + 2]];
    for (let p = 0; p < ord.length; p++) {
      const e = ord[p], [c, k] = E[e], n = S[c].length / 3, k1 = (k + 1) % n;
      for (let q = p + 1; q < ord.length && lo[3 * ord[q]] <= hi[3 * e]; q++) {
        const f = ord[q], [l, h] = E[f];
        if (lo[3 * f + 1] > hi[3 * e + 1] || lo[3 * e + 1] > hi[3 * f + 1] || lo[3 * f + 2] > hi[3 * e + 2] || lo[3 * e + 2] > hi[3 * f + 2]) continue;
        const h1 = (h + 1) % (S[l].length / 3);
        if (l === c && (h === k || h === k1 || h1 === k)) continue; // same or adjacent edge
        if (!moving[e] && !moving[f]) continue; // both at rest: clear already
        if (--this.budget < 0) return false;
        if (!edgesClear(at(S[c], k), at(T[c], k), at(S[c], k1), at(T[c], k1), at(S[l], h), at(T[l], h), at(S[l], h1), at(T[l], h1), margin)) { this.fail = [c, k, l, h]; return false; }
      }
    }
    return true;
  }
}

/** One greedy pass: from each kept keyframe, the furthest later one in reach (doubling, then bisection). */
function select(keys: Key[], M: Motions): Key[] {
  M.budget = SHORTCUT_WORK;
  const m = keys.length - 1, out = [keys[0]];
  for (let i = 0; i < m;) {
    let lim = i + 1; while (lim < m && !keys[lim + 1].ins && !keys[lim].ins) lim++;   // furthest jump without an insert
    let good = i + 1, bad = lim + 1;
    if (M.budget > 0 && lim > i + 1) {
      for (let s = 2; i + s <= lim; s *= 2) { if (M.clear(keys[i], keys[i + s], SHORTCUT_MARGIN)) good = i + s; else { bad = i + s; break; } }
      while (bad - good > 1) { const mid = (good + bad) >> 1; if (M.clear(keys[i], keys[mid], SHORTCUT_MARGIN)) good = mid; else bad = mid; }
    }
    out.push(keys[good]);
    i = good;
  }
  return out;
}

/** Playback records and group ends for consecutive keyframes: original records between untouched neighbours, else a jump. */
function emit(keys: Key[], recs: Float64Array, M: Motions): { records: Float64Array; ends: number[] } {
  const out = new MoveLog(), ends: number[] = [];
  for (let t = 0; t + 1 < keys.length; t++) {
    const A = keys[t], B = keys[t + 1];
    if (A.o >= 0 && B.o === A.o + 1) for (let q = A.r; q < B.r; q++) out.pushRecord(recs, q);
    else {
      const T = M.targets(A, B);
      A.S.forEach((P, c) => { for (let k = 0; k < P.length / 3; k++) if (P[3 * k] !== T[c][3 * k] || P[3 * k + 1] !== T[c][3 * k + 1] || P[3 * k + 2] !== T[c][3 * k + 2]) out.push(Op.Move, c, k, T[c][3 * k], T[c][3 * k + 1], T[c][3 * k + 2]); });
      A.ids.forEach((I, c) => { const W = M.index(B, c); for (let k = I.length - 1; k >= 0; k--) if (W[I[k]] < 0) out.push(Op.Delete, c, k, 0, 0, 0); });
      if (B.ph >= 0) out.pushRecord(recs, B.ph);
    }
    ends.push(out.len);
  }
  return { records: out.records.slice(), ends };
}

/** End of the group holding the log's last Delete (the last stick lost); 0 if nothing was deleted. */
export function lastProgress(recs: Float64Array, ends: number[]): number {
  let last = -1;
  for (let q = 0; q < recs.length / REC; q++) if (recs[q * REC] === Op.Delete) last = q;
  return last < 0 ? 0 : (ends.find((e) => e > last) ?? recs.length / REC);
}
