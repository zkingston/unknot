import { type Scene, prevIdx, nextIdx, removeVertex, straightestFirst } from '../kernel/scene';
import { Op } from './movelog';
import type { Ctx } from './run';

/** Move v onto the midpoint of its neighbours, then delete it; logs Move + Delete. */
export function deleteVertex(ctx: Ctx, S: Scene, c: number, v: number, log = ctx.log.push.bind(ctx.log)) {
  const P = S[c], n = P.length / 3, a = 3 * prevIdx(v, n), b = 3 * nextIdx(v, n);
  const x = 0.5 * (P[a] + P[b]), y = 0.5 * (P[a + 1] + P[b + 1]), z = 0.5 * (P[a + 2] + P[b + 2]);
  P[3 * v] = x; P[3 * v + 1] = y; P[3 * v + 2] = z;
  log(Op.Move, c, v, x, y, z);
  log(Op.Delete, c, v, 0, 0, 0);
  removeVertex(S, c, v);
}

/** First removable vertex, straightest first, or null. */
export function firstRemovable(ctx: Ctx, S: Scene): { c: number; v: number } | null {
  const k = ctx.p.maxEdge;
  // cap per component: k × its mean edge length
  const cap2 = S.map((P) => { if (!(k > 0)) return Infinity; const n = P.length / 3; let L = 0; for (let i = 0; i < n; i++) { const j = 3 * nextIdx(i, n), dx = P[j] - P[3 * i], dy = P[j + 1] - P[3 * i + 1], dz = P[j + 2] - P[3 * i + 2]; L += Math.sqrt(dx * dx + dy * dy + dz * dz); } const m = (k * L) / n; return m * m; });
  for (const { c, v } of straightestFirst(S)) {
    if (k > 0) { const P = S[c], n = P.length / 3, a = 3 * prevIdx(v, n), b = 3 * nextIdx(v, n), dx = P[b] - P[a], dy = P[b + 1] - P[a + 1], dz = P[b + 2] - P[a + 2]; if (dx * dx + dy * dy + dz * dz > cap2[c]) continue; }
    if (ctx.K.removeLegal(S, c, v, ctx.p.tau)) return { c, v };
  }
  return null;
}

/** elm: delete every removable vertex, straightest first, re-sorting after each deletion. */
export function* elm(ctx: Ctx): Generator<boolean, number> {
  let removed = 0;
  for (let f = firstRemovable(ctx, ctx.S); f; f = firstRemovable(ctx, ctx.S)) {
    deleteVertex(ctx, ctx.S, f.c, f.v);
    removed++;
    yield false;
  }
  return removed;
}
