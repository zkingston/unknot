import { index } from '../kernel/prng';
import { PhaseKind } from './movelog';
import { elm } from './elm';
import { opt } from './opt';
import { lip, lipTry } from './lip';
import { type Ctx, isDone, phase } from './run';
import { treeStep } from './randomTree';
import { pickup } from './pickup';

/** Jams that need a pickup show up on curves of 40 sticks or more; below that, elm and the tree step finish. */
export const PICKUP_STICKS = 40;
import { totalVerts } from '../kernel/scene';

/**
 * Heuristic planner (repo plan.hh): elm, opt, lip per outer iteration until every component has n <= 5.
 * With `hybrid` > 0, an iteration that ends `hybrid` iterations after the last drop in sticks is followed by one
 * random-tree step, which explores moves the energy rule rejects; with `pickup` > 0 (and a curve of more than
 * PICKUP_STICKS sticks), one ending `pickup` iterations after it is followed by a strand pickup series instead.
 */
export function* heuristic(ctx: Ctx): Generator<boolean, void> {
  let best = Infinity, since = 0;
  for (let first = true; !isDone(ctx.S); first = false) {
    if (!first) yield true;
    phase(ctx, PhaseKind.elm); yield* elm(ctx);
    phase(ctx, PhaseKind.opt); yield* opt(ctx);
    phase(ctx, PhaseKind.lip); yield* lip(ctx);
    const n = totalVerts(ctx.S);
    if (n < best) { best = n; since = 0; } else since++;
    if (ctx.p.pickup > 0 && since >= ctx.p.pickup && n > PICKUP_STICKS && (yield* pickup(ctx)) > 0) { since = 0; continue; }
    if (ctx.p.hybrid > 0 && since >= ctx.p.hybrid && !isDone(ctx.S)) { yield* treeStep(ctx); since = 0; }
  }
  yield* finish(ctx);
}

/** Finishing tail: up to tailRounds of one elm pass and one lip try per component, to reach triangles. */
export function* finish(ctx: Ctx): Generator<boolean, void> {
  phase(ctx, PhaseKind.finish);
  for (let r = 0; r < ctx.p.tailRounds && !ctx.S.every((P) => P.length === 9); r++) {
    yield* elm(ctx);
    for (let c = 0; c < ctx.S.length; c++) if (ctx.S[c].length > 9) lipTry(ctx, c, index(ctx.rng, ctx.S[c].length / 3));
    yield false;
  }
}
