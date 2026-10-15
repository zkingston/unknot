// MoveLog: Float64Array of 6-wide records (48 bytes each). Indices refer to the Scene as it stands at that point.
// [Move,   comp, v, x, y, z]   vertex v of component comp slides straight to (x,y,z); the sweep is collision-free
// [Delete, comp, v, 0, 0, 0]   remove v; v lies within 1e-12 of segment (v-1, v+1)
// [Insert, comp, v, x, y, z]   new vertex v at (x,y,z), within 1e-12 of segment (v-1, v): the curve is unchanged
// [Phase,  kind, energy, clearance, n, plannerMs]   stats only
import { type Scene, removeVertex, insertVertex } from '../kernel/scene';

export const Op = { Move: 0, Delete: 1, Phase: 2, Insert: 3 } as const;
export const PhaseKind = { elm: 0, opt: 1, lip: 2, tree: 3, finish: 4, pickup: 5 } as const;
export const REC = 6;

export class MoveLog {
  buf = new Float64Array(REC * 4096);
  len = 0; // records
  push(op: number, a: number, b: number, c: number, d: number, e: number) {
    if ((this.len + 1) * REC > this.buf.length) { const nb = new Float64Array(this.buf.length * 2); nb.set(this.buf); this.buf = nb; }
    const o = this.len++ * REC, B = this.buf;
    B[o] = op; B[o + 1] = a; B[o + 2] = b; B[o + 3] = c; B[o + 4] = d; B[o + 5] = e;
  }
  /** Copy record r of `recs`. */
  pushRecord(recs: Float64Array, r: number) { const o = r * REC; this.push(recs[o], recs[o + 1], recs[o + 2], recs[o + 3], recs[o + 4], recs[o + 5]); }
  get records() { return this.buf.subarray(0, this.len * REC); }
  /** Copy of records [from, len). */
  since(from: number) { return this.buf.slice(from * REC, this.len * REC); }
}

/** Apply record r of `recs` to S in place. */
export function applyRecord(S: Scene, recs: Float64Array, r: number) {
  const o = r * REC, op = recs[o];
  if (op === Op.Move) { const P = S[recs[o + 1]], v = 3 * recs[o + 2]; P[v] = recs[o + 3]; P[v + 1] = recs[o + 4]; P[v + 2] = recs[o + 5]; }
  else if (op === Op.Delete) removeVertex(S, recs[o + 1], recs[o + 2]);
  else if (op === Op.Insert) insertVertex(S, recs[o + 1], recs[o + 2], recs[o + 3], recs[o + 4], recs[o + 5]);
}

export function replay(S: Scene, recs: Float64Array, from = 0, to = recs.length / REC) {
  for (let r = from; r < to; r++) applyRecord(S, recs, r);
  return S;
}

/** Records with the timing field of Phase records zeroed: the bytes that must be reproducible. */
export function canonical(recs: Float64Array) {
  const c = recs.slice();
  for (let o = 0; o < c.length; o += REC) if (c[o] === Op.Phase) c[o + 5] = 0;
  return c;
}

