import { sfc32, splitmix32, type Rng } from '../kernel/prng';
import { type Kernel, tsKernel } from '../kernel/kernel';
import { type Scene, cloneScene, totalVerts } from '../kernel/scene';
import { MoveLog, Op } from './movelog';
import { heuristic } from './heuristic';

export interface PlannerParams {
  tau: number;        // clearance kept between non-adjacent edges
  optSteps: number;   // opt sweeps per heuristic iteration [repo 10]
  optStep: number;    // opt step length [repo 1.0]
  lipSteps: number;   // lip tries per heuristic iteration [repo 100]
  treeNodes: number;  // random-tree nodes per call [repo 100]
  tailRounds: number; // finishing rounds after n <= 5
  forcePower: number; // opt repulsion magnitude ~ 1/d^p [repo 1]; 3 keeps near-contact strands from sticking
  softMargin: number; // opt/lip moves keep moved edges at least this far from others, or no closer than now [repo 0]
  hybrid: number;     // heuristic: after this many iterations without fewer sticks, one random-tree step [repo 0 = off]
  straighten: number; // opt direction = unit(repulsion) + this × unit(toward the neighbours' midpoint) [repo 0]
  maxEdge: number;    // > 0: no deletion makes an edge longer than this × the mean edge (doubling on every marginStall
                      // stall, for the rest of the attempt): greedy deletion otherwise leaves long sticks that jam [0 = off]
  pickup: number;     // > 0: after this many iterations without progress (over 40 sticks), pick up strands (pickup.ts) [0 = off]
  restart: number;    // start over from the input with a new derived seed after this many iterations without progress [0 = never]
  stall: number;      // stop (unsolved) after this many iterations without progress [0 = never]; for proven knots
  marginStall: number; // > 0: adaptive margin. softMargin applies once the curve is down to MARGIN_STICKS sticks (a
                       // crowded curve keeps MARGIN_FLOOR, where a wide margin costs the hardest diagrams most); it
                       // halves after every marginStall iterations without progress and stays halved for the rest
                       // of the attempt, never below MARGIN_FLOOR
                       // [0 = fixed softMargin]
}
/** Adaptive margin: full at this many sticks or fewer, and never below the floor (the former fixed default). */
const MARGIN_STICKS = 40, MARGIN_FLOOR = 0.005;
export const DEFAULT_PARAMS: PlannerParams = { tau: 1e-5, optSteps: 10, optStep: 1.0, lipSteps: 100, treeNodes: 100, tailRounds: 50, forcePower: 3, softMargin: 0.05, hybrid: 5, straighten: 0.3, maxEdge: 4, pickup: 3, restart: 50, stall: 0, marginStall: 10 };

export interface Ctx { S: Scene; rng: Rng; K: Kernel; p: PlannerParams; log: MoveLog; ms: () => number }

/** Every knot with 5 or fewer sticks is the unknot (the trefoil needs 6). */
export const isDone = (S: Scene) => S.every((P) => P.length <= 15);

export function phase(ctx: Ctx, kind: number) {
  const { S, K, log } = ctx;
  log.push(Op.Phase, kind, K.energy(S), K.clearance(S), totalVerts(S), ctx.ms());
}

/**
 * A resumable run: the generator yields `true` between outer iterations (the only place a budget may stop it)
 * and `false` at finer points (so a driver can post chunks). Slicing never changes the sequence.
 */
export function createRun(scene: Scene, params: Partial<PlannerParams>, seed: number, ms: () => number, K: Kernel = tsKernel) {
  const p = { ...DEFAULT_PARAMS, ...params }, baseMargin = p.softMargin, baseEdge = p.maxEdge;
  // adaptive margin for a curve of n sticks after `level` halvings (only + − × ÷ and exact halvings: bit-identical)
  const margin = (n: number, level: number) => {
    let m = n > MARGIN_STICKS ? MARGIN_FLOOR : baseMargin;
    for (let k = 0; k < level && m > MARGIN_FLOOR; k++) m *= 0.5;
    return m < MARGIN_FLOOR ? MARGIN_FLOOR : m;
  };
  if (p.marginStall > 0) p.softMargin = margin(totalVerts(scene), 0);
  const fresh = (s: number): Ctx => ({ S: cloneScene(scene), rng: sfc32(s), K, p, log: new MoveLog(), ms });
  const run = { ctx: fresh(seed), attempt: 0, stalled: false, gen: null as unknown as Generator<boolean, void> };
  // restarts: an attempt that goes `restart` outer iterations without losing a stick is abandoned, and the next
  // starts from the input with seed splitmix32(seed + attempt); counted in iterations, so runs stay reproducible
  run.gen = (function* () {
    for (;;) {
      const inner = heuristic(run.ctx);
      let best = Infinity, since = 0, abandoned = false, level = 0; // level: margin halvings so far this attempt
      for (;;) {
        const r = inner.next();
        if (r.done) return;
        if (r.value && (p.restart > 0 || p.stall > 0 || p.marginStall > 0)) {
          const n = totalVerts(run.ctx.S);
          if (n < best) { best = n; since = 0; } else ++since;
          if (p.marginStall > 0) {
            if (since > 0 && since % p.marginStall === 0) { level++; if (baseEdge > 0) p.maxEdge *= 2; } // sticky for the attempt
            p.softMargin = margin(n, level);
          }
          if (p.stall > 0 && since >= p.stall) { run.stalled = true; return; }
          if (p.restart > 0 && since >= p.restart) { abandoned = true; break; }
        }
        yield r.value;
      }
      if (!abandoned) return;
      run.attempt++;
      p.maxEdge = baseEdge;
      if (p.marginStall > 0) p.softMargin = margin(totalVerts(scene), 0);
      run.ctx = fresh(splitmix32((seed + run.attempt) >>> 0)());
      yield true;
    }
  })();
  return run;
}

/** Run to completion or budget on this thread (tests, tools). */
export function runSync(scene: Scene, params: Partial<PlannerParams>, seed: number, budgetMs = 5000, K?: Kernel) {
  const t0 = performance.now(), ms = () => performance.now() - t0;
  const run = createRun(scene, params, seed, ms, K);
  for (;;) {
    const r = run.gen.next();
    if (r.done) return { log: run.ctx.log, S: run.ctx.S, solved: !run.stalled, stalled: run.stalled, ms: ms(), attempts: run.attempt + 1 };
    if (r.value && ms() >= budgetMs) return { log: run.ctx.log, S: run.ctx.S, solved: false, stalled: false, ms: ms(), attempts: run.attempt + 1 };
  }
}
