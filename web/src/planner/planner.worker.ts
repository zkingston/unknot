import { createRun, type PlannerParams } from './run';
import { totalVerts } from '../kernel/scene';
import { Grouper, shortcut, lastProgress } from './polish';
import { REC } from './movelog';

const ctx = self as unknown as { postMessage(m: unknown, t?: Transferable[]): void; onmessage: ((e: MessageEvent) => void) | null };

export type ToPlanner =
  | { type: 'start'; scene: Float64Array[]; params: Partial<PlannerParams>; seed: number; budgetMs: number }
  | { type: 'pause' } | { type: 'resume' } | { type: 'extend'; ms: number } | { type: 'cancel' } | { type: 'ping' };
export interface ChunkStats { n: number; energy: number; clearance: number; plannerMs: number }
export type FromPlanner =
  | { type: 'chunk'; records: Float64Array; stats: ChunkStats; groups: number[] }
  | { type: 'done'; solved: boolean; n: number; plannerMs: number; reason?: string; groups: number[]; simplify?: boolean } // simplify: a 'simplified' follows
  | { type: 'restart'; attempt: number }   // discard everything received so far: the run starts over
  // after a finished run: shortcut playback (polish.ts) of its first `cut` records (a stalled run ends at the group of
  // its last deletion: the iterations that only failed to lose a stick are not shown)
  | { type: 'simplified'; records: Float64Array; groups: number[]; cut: number }
  | { type: 'pong' } | { type: 'error'; message: string };

const SLICE_MS = 40;            // work per slice; a chunk is posted after every slice (< 50 ms apart)
const LOG_LIMIT = 1_000_000;    // records (48 MB)

const GROUP_LIMIT = 60_000;    // records grouped for smooth playback; later ones play one by one
let run: ReturnType<typeof createRun> | null = null;
let grouper: Grouper | null = null;
let kept: Float64Array[] = []; // this attempt's records while grouped, for the shortcut pass
let attempt = 0, initial: Float64Array[] = [];
let paused = false, stopped = false, budget = 0, active = 0, sliceStart = 0, total = 0;
const tick = new MessageChannel();
tick.port1.onmessage = () => slice();
const schedule = () => tick.port2.postMessage(0);
const plannerMs = () => active + (sliceStart ? performance.now() - sliceStart : 0);

function post(m: FromPlanner, t?: Transferable[]) { ctx.postMessage(m, t); }

function flush() {
  if (!run) return;
  if (run.attempt !== attempt) {
    // the planner abandoned an attempt and started over from the input
    attempt = run.attempt; total = 0; grouper = new Grouper(initial); kept = [];
    post({ type: 'restart', attempt });
  }
  const { log, S, K } = run.ctx;
  const records = log.since(0);
  total += log.len;
  log.len = 0;
  let groups: number[] = [];
  if (grouper) { kept.push(records.slice()); groups = grouper.push(records); if (total > GROUP_LIMIT) { groups.push(...grouper.finish()); grouper = null; kept = []; } }
  post({ type: 'chunk', records, stats: { n: totalVerts(S), energy: K.energy(S), clearance: K.clearance(S), plannerMs: plannerMs() }, groups }, [records.buffer]);
}

function slice() {
  if (!run || paused || stopped) return;
  sliceStart = performance.now();
  let finished = false, reason = '';
  try {
    while (performance.now() - sliceStart < SLICE_MS) {
      const r = run.gen.next();
      if (r.done) { finished = true; break; }
      if (r.value && plannerMs() >= budget) { stopped = true; break; }
      if (total + run.ctx.log.len >= LOG_LIMIT) { stopped = true; reason = 'log limit reached'; break; }
    }
  } catch (e) {
    post({ type: 'error', message: String(e) });
    run = null;
    return;
  }
  active += performance.now() - sliceStart;
  sliceStart = 0;
  flush();
  const n = totalVerts(run.ctx.S);
  // a stopped run may be extended: groups stay open until the run really ends
  const closing = finished || !!reason, groups = closing && grouper ? grouper.finish() : [];
  if (finished) {
    // finished = solved, or a proven knot that stopped simplifying (params.stall)
    const stalled = run.stalled;
    post({ type: 'done', solved: !stalled, n, plannerMs: active, groups, simplify: !!grouper, ...(stalled ? { reason: 'stalled' } : {}) });
    run = null;
    if (grouper) {
      const all = new Float64Array(kept.reduce((s, k) => s + k.length, 0));
      let o = 0; for (const k of kept) { all.set(k, o); o += k.length; }
      const cut = stalled ? lastProgress(all, grouper.ends) : all.length / REC;
      const sc = shortcut(initial, all.subarray(0, cut * REC), grouper.ends.filter((e) => e <= cut));
      kept = [];
      post({ type: 'simplified', records: sc.records, groups: sc.ends, cut }, [sc.records.buffer]);
    }
  }
  else if (stopped) { post({ type: 'done', solved: false, n, plannerMs: active, groups: grouper ? grouper.peekFinish() : [], ...(reason ? { reason } : {}) }); if (reason) run = null; }
  else schedule();
}

ctx.onmessage = (e) => {
  const m = e.data as ToPlanner;
  switch (m.type) {
    case 'ping': post({ type: 'pong' }); break;
    case 'start':
      active = 0; total = 0; paused = false; stopped = false; budget = m.budgetMs;
      run = createRun(m.scene, m.params, m.seed, plannerMs);
      initial = m.scene.map((P) => P.slice()); attempt = 0;
      grouper = new Grouper(initial); kept = [];
      schedule();
      break;
    case 'pause': paused = true; break;
    case 'resume': if (paused) { paused = false; schedule(); } break;
    case 'extend': budget += m.ms; if (run && stopped) { stopped = false; schedule(); } break;
    case 'cancel': run = null; break;
  }
};
