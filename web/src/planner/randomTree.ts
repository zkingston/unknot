import { onSphere, uniform } from '../kernel/prng';
import { type Scene, cloneScene } from '../kernel/scene';
import { Op, PhaseKind, REC } from './movelog';
import { deleteVertex, firstRemovable } from './elm';
import { pickVertex, RADIUS } from './lip';
import { type Ctx, phase } from './run';

interface Node { S: Scene; parent: number; w: number; E: number; rec: number[] }

function sample(ctx: Ctx, nodes: Node[]): number {
  let total = 0;
  for (const n of nodes) total += n.w;
  let u = uniform(ctx.rng) * total, last = 0;
  for (let i = 0; i < nodes.length; i++) {
    if (nodes[i].w <= 0) continue;
    last = i;
    if (u < nodes[i].w) return i;
    u -= nodes[i].w;
  }
  return last;
}

const target = new Float64Array(3);

/**
 * One random-tree call from the current state (the repo's RandomTree, seeded), used by the heuristic's hybrid
 * option: one elm deletion if possible, else grow a tree and follow its best path. Repo defect #5 fixed: the elm
 * deletion is tried on the new lip node itself and attached under it, instead of running elm on the parent.
 */
export function* treeStep(ctx: Ctx): Generator<boolean, void> {
  const { K, p } = ctx;
  phase(ctx, PhaseKind.tree);
  const f = firstRemovable(ctx, ctx.S);
  if (f) { deleteVertex(ctx, ctx.S, f.c, f.v); return; }
  const nodes: Node[] = [{ S: cloneScene(ctx.S), parent: -1, w: 1, E: K.energy(ctx.S), rec: [] }];
  let goal = -1;
  for (let it = 0; it < p.treeNodes && goal < 0; it++) {
    const idx = sample(ctx, nodes), par = nodes[idx];
    par.w /= 2;
    const S = cloneScene(par.S), { c, v } = pickVertex(ctx, S);
    onSphere(ctx.rng, RADIUS, target);
    const t = K.maxLegalMove(S, c, v, target, p.tau, p.softMargin), rec: number[] = [];
    let E = par.E;
    if (t > 0) {
      const P = S[c], o = 3 * v;
      target[0] = P[o] + t * (target[0] - P[o]); target[1] = P[o + 1] + t * (target[1] - P[o + 1]); target[2] = P[o + 2] + t * (target[2] - P[o + 2]);
      E += K.energyDelta(S, c, v, target);
      P[o] = target[0]; P[o + 1] = target[1]; P[o + 2] = target[2];
      rec.push(Op.Move, c, v, target[0], target[1], target[2]);
    }
    nodes.push({ S, parent: idx, w: t, E, rec });
    const g = firstRemovable(ctx, S);
    if (g) {
      const S2 = cloneScene(S), rec2: number[] = [];
      deleteVertex(ctx, S2, g.c, g.v, (...r: number[]) => { rec2.push(...r); });
      nodes.push({ S: S2, parent: nodes.length - 1, w: 1, E: 0, rec: rec2 });
      goal = nodes.length - 1;
    }
    yield false;
  }
  if (goal < 0) {
    let best = 0;
    for (let i = 1; i < nodes.length; i++) if (nodes[i].E < nodes[best].E) best = i;
    goal = best !== 0 ? best : sample(ctx, nodes);
  }
  const path: Node[] = [];
  for (let i = goal; i > 0; i = nodes[i].parent) path.push(nodes[i]);
  for (let k = path.length - 1; k >= 0; k--) {
    const r = path[k].rec;
    for (let o = 0; o < r.length; o += REC) ctx.log.push(r[o], r[o + 1], r[o + 2], r[o + 3], r[o + 4], r[o + 5]);
  }
  ctx.S = nodes[goal].S;
}
