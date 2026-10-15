// Trajectory = initial Scene + MoveLog + a snapshot every 256 records. Playback animates groups: runs of records
// the planner worker verified (exact CCD) as one simultaneous straight-line motion. A group lasts in proportion to
// its largest displacement (constant top speed SPEED, plus a short floor), the whole log scaled into [6 s, 40 s].
// Without groups (live), one record is one group.
import { type Scene, cloneScene, identityIds } from '../kernel/scene';
import { Op, REC, applyRecord } from './movelog';

const SNAP = 256;
const SPEED = 1;       // scene units per second at 1x (the lifted knot spans about 2)
const FLOOR = 0.012;   // units: every moving group lasts at least FLOOR / SPEED
const T_MIN = 6000, T_MAX = 40000;
const FADE = 400;      // ms at 1x: a deletion's effect on the smoothed display fades in over this long

/** Vertices a group deleted, as points on the edge between their surviving neighbours (index `after` and next). */
interface GhostRun { c: number; after: number; f: number[] }
/** The Scene with the most recent deletion's vertices put back (collinear), how much they still count, and each of
 *  its vertices' index in the current Scene (−1 for the put-back ones). */
export interface Ghost { S: Scene; alpha: number; toCurrent: Int32Array[] }

export class Trajectory {
  recs = new Float64Array(REC * 4096);
  len = 0;
  private snaps: Scene[];
  private head: Scene;
  /** (records, plannerMs) markers from chunks, for live playback. */
  marks: { r: number; ms: number }[] = [{ r: 0, ms: 0 }];
  /** Phase records: index and stats. */
  phases: { r: number; kind: number; energy: number; clearance: number; n: number; ms: number }[] = [];
  done = false;
  private disp = new Float64Array(4096);      // displacement of each Move record
  private closedEnds: number[] = [];           // group ends from chunks (final)
  private tailEnds: number[] = [];             // provisional ends for a run that stopped and may be extended
  private gEnd: Int32Array | null = null;      // exclusive end of each group, finalized
  private gTime: Float64Array | null = null;   // cumulative end time (ms at 1x) of each group
  private gT = T_MIN;
  private ghostRuns = new Map<number, GhostRun[]>(); // per group with deletions
  constructor(public initial: Scene) {
    this.snaps = [cloneScene(initial)];
    this.head = cloneScene(initial);
  }

  append(chunk: Float64Array, plannerMs?: number) {
    const m = chunk.length / REC;
    if ((this.len + m) * REC > this.recs.length) {
      let cap = this.recs.length; while ((this.len + m) * REC > cap) cap *= 2;
      const nr = new Float64Array(cap); nr.set(this.recs.subarray(0, this.len * REC)); this.recs = nr;
      const nd = new Float64Array(cap / REC); nd.set(this.disp.subarray(0, this.len)); this.disp = nd;
    }
    this.recs.set(chunk, this.len * REC);
    for (let k = 0; k < m; k++) {
      const r = this.len, o = r * REC, R = this.recs;
      if (R[o] === Op.Move) {
        const P = this.head[R[o + 1]], v = 3 * R[o + 2];
        const dx = R[o + 3] - P[v], dy = R[o + 4] - P[v + 1], dz = R[o + 5] - P[v + 2];
        this.disp[r] = Math.sqrt(dx * dx + dy * dy + dz * dz);
      } else if (R[o] === Op.Phase) this.phases.push({ r, kind: R[o + 1], energy: R[o + 2], clearance: R[o + 3], n: R[o + 4], ms: R[o + 5] });
      applyRecord(this.head, R, r);
      this.len++;
      if (this.len % SNAP === 0) this.snaps.push(cloneScene(this.head));
    }
    if (plannerMs !== undefined) this.marks.push({ r: this.len, ms: plannerMs });
  }

  /** Group ends from the planner: `final` ends are permanent, others replace the provisional tail. */
  addGroups(ends: number[], final: boolean) {
    if (final) { this.closedEnds.push(...ends); this.tailEnds = []; } else this.tailEnds = ends;
    this.gEnd = null;
  }

  /** Build the group table (records after the last reported end play one by one). */
  private groups() {
    if (this.gEnd) return;
    const ends = [...this.closedEnds, ...this.tailEnds].filter((e, i, a) => e > (i ? a[i - 1] : 0) && e <= this.len);
    for (let r = (ends[ends.length - 1] ?? 0) + 1; r <= this.len; r++) ends.push(r);
    const g = Int32Array.from(ends), w = new Float64Array(g.length);
    for (let k = 0, r = 0; k < g.length; k++) {
      let m = 0, any = false;
      for (; r < g[k]; r++) if (this.recs[r * REC] === Op.Move) { any = true; if (this.disp[r] > m) m = this.disp[r]; }
      if (any) w[k] = FLOOR + m;
    }
    const W = w.reduce((a, b) => a + b, 0) || 1, T = Math.min(T_MAX, Math.max(T_MIN, (1000 * W) / SPEED));
    this.gT = T;
    const t = new Float64Array(g.length);
    for (let k = 0, acc = 0; k < g.length; k++) { acc += w[k]; t[k] = (T * acc) / W; }
    this.gEnd = g; this.gTime = t; this.ghostRuns.clear();
  }

  /** Deleted-vertex runs of group k (empty if it deletes nothing), from its state just before the deletions. */
  private runsOf(k: number): GhostRun[] {
    let runs = this.ghostRuns.get(k);
    if (runs) return runs;
    runs = [];
    const g = this.gEnd!, r0 = k ? g[k - 1] : 0, S = this.stateAt(r0).S; // a fresh copy
    const alive = identityIds(S), dead = S.map(() => new Set<number>());
    for (let q = r0; q < g[k]; q++) {
      const o = q * REC, op = this.recs[o], c = this.recs[o + 1];
      if (op === Op.Move) { const v = alive[c][this.recs[o + 2]]; S[c].set(this.recs.subarray(o + 3, o + 6), 3 * v); }
      else if (op === Op.Delete) { dead[c].add(alive[c][this.recs[o + 2]]); alive[c].splice(this.recs[o + 2], 1); }
    }
    S.forEach((P, c) => {
      if (!dead[c].size) return;
      const n = P.length / 3, keep = alive[c], d = (i: number, j: number) => Math.hypot(P[3 * j] - P[3 * i], P[3 * j + 1] - P[3 * i + 1], P[3 * j + 2] - P[3 * i + 2]);
      keep.forEach((A, rank) => {
        const B = keep[(rank + 1) % keep.length], path = [A];
        for (let v = (A + 1) % n; v !== B; v = (v + 1) % n) path.push(v);
        if (path.length === 1) return;
        path.push(B);
        const s = [0];
        for (let i = 1; i < path.length; i++) s.push(s[i - 1] + d(path[i - 1], path[i]));
        const L = s[s.length - 1] || 1;
        runs!.push({ c, after: rank, f: s.slice(1, -1).map((x) => x / L) });
      });
    });
    this.ghostRuns.set(k, runs);
    return runs;
  }

  /** The latest deletion within FADE before τ, put back into S (current indexing) as collinear ghosts. */
  private ghostAt(tau: number, S: Scene, k: number): Ghost | null {
    const t = this.gTime!;
    for (let j = k - 1; j >= 0 && t[j] > tau - FADE; j--) {
      if (this.inserts(j)) return null; // indices of earlier deletions no longer match the current Scene
      const runs = this.runsOf(j);
      if (!runs.length) continue;
      const out = S.map((P) => Array.from(P)), idx = identityIds(S);
      // insert from the highest index down so earlier insert positions stay valid
      for (const run of [...runs].sort((x, y) => x.c - y.c || y.after - x.after)) {
        const P = S[run.c], n = P.length / 3, A = run.after, B = (A + 1) % n, pts: number[] = [];
        for (const f of run.f) for (let x = 0; x < 3; x++) pts.push(P[3 * A + x] + f * (P[3 * B + x] - P[3 * A + x]));
        out[run.c].splice(3 * (A + 1), 0, ...pts);
        idx[run.c].splice(A + 1, 0, ...run.f.map(() => -1));
      }
      return { S: out.map((a) => Float64Array.from(a)), alpha: Math.max(0, Math.min(1, 1 - (tau - t[j]) / FADE)), toCurrent: idx.map((a) => Int32Array.from(a)) };
    }
    return null;
  }
  /** Does group k insert vertices? */
  private inserts(k: number) {
    const g = this.gEnd!;
    for (let q = k ? g[k - 1] : 0; q < g[k]; q++) if (this.recs[q * REC] === Op.Insert) return true;
    return false;
  }
  /** Group containing record r (records in [start, end)). */
  private groupOf(r: number) {
    this.groups();
    const g = this.gEnd!;
    let lo = 0, hi = g.length - 1;
    while (lo < hi) { const m = (lo + hi) >> 1; if (g[m] > r) hi = m; else lo = m + 1; }
    return lo;
  }

  get records() { return this.recs.subarray(0, this.len * REC); }
  get final(): Scene { return this.head; }

  /** Total playback time at 1x, ms. */
  get T() { this.groups(); return this.gT; }
  /** End time (ms at 1x) of record r: the end of its group. */
  endTime(r: number) {
    if (r < 0 || !this.len) return 0;
    const k = this.groupOf(Math.min(r, this.len - 1)); // builds the table before gTime is read
    return this.gTime![k];
  }
  /** First record of the group in progress at τ (records before it are complete), or len. */
  indexAt(tau: number) {
    this.groups();
    const t = this.gTime!, g = this.gEnd!;
    let lo = 0, hi = t.length;
    while (lo < hi) { const m = (lo + hi) >> 1; if (t[m] <= tau) lo = m + 1; else hi = m; }
    return lo === 0 ? 0 : lo >= t.length ? this.len : g[lo - 1];
  }

  /** Scene after the first r records: a fresh copy, or `into` advanced in place (then returned). */
  stateAt(r: number, into?: { S: Scene; r: number }): { S: Scene; r: number } {
    r = Math.max(0, Math.min(r, this.len));
    let cur = into;
    if (!cur || cur.r > r || r - cur.r > SNAP) {
      const k = Math.floor(r / SNAP);
      cur = { S: cloneScene(this.snaps[k]), r: k * SNAP };
    }
    for (; cur.r < r; cur.r++) applyRecord(cur.S, this.recs, cur.r);
    return cur;
  }

  /** Displayed Scene at τ: complete groups applied; the group in progress moves all its vertices together. */
  sceneAt(tau: number, cursor?: { S: Scene; r: number }): { S: Scene; r: number; cursor: { S: Scene; r: number }; ghost: Ghost | null } {
    const r = this.indexAt(tau), cur = this.stateAt(r, cursor), S = cloneScene(cur.S);
    const kNow = r < this.len ? this.groupOf(r) : this.gEnd!.length;
    if (r < this.len) {
      const k = kNow, t0 = k ? this.gTime![k - 1] : 0, t1 = this.gTime![k];
      const f = t1 > t0 ? Math.min(1, Math.max(0, (tau - t0) / (t1 - t0))) : 0;
      // map each record's index (in the scene as it stands then) back to the group's start state
      const alive = identityIds(S), start = cur.S; // S is a copy of cur.S, so cur.S keeps the group's start
      for (let q = r; q < this.gEnd![k]; q++) {
        const o = q * REC, op = this.recs[o], c = this.recs[o + 1];
        if (op === Op.Move) {
          const g = alive[c][this.recs[o + 2]], P = S[c], P0 = start[c];
          for (let x = 0; x < 3; x++) P[3 * g + x] = P0[3 * g + x] + f * (this.recs[o + 3 + x] - P0[3 * g + x]);
        } else if (op === Op.Delete) alive[c].splice(this.recs[o + 2], 1);
      }
    }
    return { S, r, cursor: cur, ghost: this.ghostAt(tau, S, kNow) };
  }

  /** Record count at the live head minus `lagMs` of planner time. */
  liveIndex(lagMs = 250) {
    const last = this.marks[this.marks.length - 1], target = last.ms - lagMs;
    if (target <= 0) return 0;
    for (let k = this.marks.length - 1; k > 0; k--) {
      const a = this.marks[k - 1], b = this.marks[k];
      if (a.ms <= target) return Math.round(a.r + ((b.r - a.r) * (target - a.ms)) / Math.max(1e-9, b.ms - a.ms));
    }
    return 0;
  }
}
