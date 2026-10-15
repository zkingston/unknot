import { index, onSphere } from '../kernel/prng';
import { totalVerts } from '../kernel/scene';
import { Op } from './movelog';
import type { Ctx } from './run';

export const RADIUS = Math.sqrt(3);
const target = new Float64Array(3);

/** Global vertex index -> (component, vertex): a component weighted by vertex count, then a vertex in it. */
export function pickVertex(ctx: Ctx, S = ctx.S) {
  let g = index(ctx.rng, totalVerts(S)), c = 0;
  while (g >= S[c].length / 3) { g -= S[c].length / 3; c++; }
  return { c, v: g };
}

/** One lip try on vertex (c,v): move toward a random point on the √3 sphere; keep if energy drops. */
export function lipTry(ctx: Ctx, c: number, v: number): boolean {
  const { S, K, p } = ctx;
  onSphere(ctx.rng, RADIUS, target);
  const t = K.maxLegalMove(S, c, v, target, p.tau, p.softMargin);
  if (t <= 0) return false;
  const P = S[c], o = 3 * v;
  const x = P[o] + t * (target[0] - P[o]), y = P[o + 1] + t * (target[1] - P[o + 1]), z = P[o + 2] + t * (target[2] - P[o + 2]);
  target[0] = x; target[1] = y; target[2] = z;
  if (!(K.energyDelta(S, c, v, target) < 0)) return false;
  P[o] = x; P[o + 1] = y; P[o + 2] = z;
  ctx.log.push(Op.Move, c, v, x, y, z);
  return true;
}

/** lip: lipSteps random tries. */
export function* lip(ctx: Ctx): Generator<boolean, void> {
  for (let k = 0; k < ctx.p.lipSteps; k++) {
    const { c, v } = pickVertex(ctx);
    lipTry(ctx, c, v);
    yield false;
  }
}
