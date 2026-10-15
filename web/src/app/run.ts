// Planner runs and playback: the timeline player, planner messages, status line, stats, run controls.
import type { Scene } from '../kernel/scene';
import { totalVerts } from '../kernel/scene';
import { Trajectory } from '../planner/trajectory';
import { Player } from '../ui/timeline';
import { format, type Laurent } from '../invariants/laurent';
import type { Analysis } from '../invariants';
import { toPlanner, onPlanner } from '../state/workers';
import { $, esc, app, freshRun, runnable, type RunState } from './state';
import { stage, draw, editor, runBestView } from './view';
import { params } from './settings';

let phaseSeen = 0, endedFor: Trajectory | null = null, pendingPlay: Trajectory | null = null, pendingCut = 0, liveAtDone = false;

// A run plays under glass: frozen on its input for FROST_MS (a fast solve then shows nothing but its replay), then
// frosted ("planning…", "simplifying…") with the live knot moving behind it, until its replay starts. Pausing or
// touching the timeline clears it.
const FROST_MS = 150;
let glass = false, frostTimer = 0;
/** Frost the display with this text (after FROST_MS, unless it already is); `then` runs once it is frosted. */
function frost(text: string, then?: () => void) {
  $('frost-text').textContent = text;
  clearTimeout(frostTimer);
  const on = () => { $('stage').classList.add('frosted'); then?.(); };
  if ($('stage').classList.contains('frosted')) on(); else frostTimer = window.setTimeout(on, FROST_MS);
}
function unfrost() { clearTimeout(frostTimer); glass = false; $('stage').classList.remove('frosted'); }
/** The user takes over the timeline: clear the glass, start no replay for them. */
export function takeOver() { if (glass) { unfrost(); liveAtDone = false; } }

export const player = new Player({
  scrub: $('scrub'), head: $('head'), area: $('playarea'), line: $('playline'), counter: $('counter'), speed: $('b-speed'), live: $('b-live') as HTMLButtonElement,
  sticks: $('sp-sticks') as unknown as SVGSVGElement, energy: $('sp-energy') as unknown as SVGSVGElement, cross: $('sp-cross') as unknown as SVGSVGElement,
}, (S, r, ghost) => draw(S, r, ghost), () => { updateRunButton(); if (!player.playing) stage.follow(null); if (player.ended()) endOfPlayback(); });

/** At the end of a run, show the final state from its best view. */
async function endOfPlayback() {
  if (pendingPlay) usePlayback();
  const t = app.traj;
  if (!t || endedFor === t || !t.done) return;
  endedFor = t;
  await runBestView(t.final);
}

const inView = (S: Scene) => stage.crossingsAlong(S, stage.view);

export function cancelRun() {
  if (app.run.state === 'planning') toPlanner({ type: 'cancel' });
  player.pause();
  unfrost();
}

export function startRun() {
  const k = app.knot;
  if (!k || !runnable()) return;
  cancelRun();
  const t = new Trajectory(k.scene);
  app.traj = t; app.raw = null; pendingPlay = null; phaseSeen = 0; liveAtDone = false;
  app.run = freshRun('planning', totalVerts(k.scene));
  player.setTrajectory(t);
  player.crossings.push({ r: 0, c: inView(k.scene) });
  player.live = false; player.render();
  glass = true;
  frost('planning…', () => { if (app.run.state === 'planning' && !player.playing) { player.live = true; player.play(); } });
  // a knot the invariants prove knotted can never reach 5 sticks: no restarts (they only erase the trajectory), and
  // it stops once 50 iterations pass without losing a stick
  const knotted = k.analysis.kind === 'ok' && k.analysis.knotted;
  toPlanner({ type: 'start', scene: k.scene.map((P) => P.slice()), params: { ...params, ...(knotted ? { restart: 0, stall: 50 } : {}) }, seed: app.seed, budgetMs: app.budgetMs });
  updateStatus(); updateRunButton();
}

onPlanner((m) => {
  const t = app.traj;
  if (m.type === 'chunk' && t && app.run.state === 'planning') {
    t.append(m.records, m.stats.plannerMs);
    t.addGroups(m.groups, true);
    // crossing counts for the sparkline: one sample per chunk (at its last Phase record) keeps the main thread light
    if (t.phases.length > phaseSeen) { const ph = t.phases[t.phases.length - 1]; player.crossings.push({ r: ph.r, c: inView(t.stateAt(ph.r).S) }); phaseSeen = t.phases.length; }
    app.run.plannerMs = m.stats.plannerMs; app.run.n = m.stats.n;
    player.drawSparks();
    updateStats();
  } else if (m.type === 'done' && t && app.run.state === 'planning') {
    t.done = true;
    t.addGroups(m.groups, m.solved || !!m.reason);
    // under the glass with a shortcut to come: hold (the final state, or the input if it never moved) behind the
    // frost ("simplifying…") until it arrives, then replay it from the start
    const behind = player.live; // the live knot was moving behind the glass (else still frozen on the input)
    liveAtDone = glass && !!m.simplify;
    app.run = { state: 'done', solved: m.solved, plannerMs: m.plannerMs, n: m.n, reason: m.reason ?? '', attempts: app.run.attempts, error: '', simplifying: liveAtDone };
    player.crossings.push({ r: t.len, c: inView(t.final) });
    player.finished();
    player.drawSparks();
    if (liveAtDone) { player.pause(); player.seek(behind ? player.duration() : 0); frost('simplifying…'); }
    else { if (glass && !behind) player.seek(0); unfrost(); if (!player.playing) player.play(); }
    document.body.dataset.run = m.solved ? 'solved' : 'stopped';
    updateStatus(); updateRunButton();
  } else if (m.type === 'simplified' && t && app.run.state === 'done' && !app.raw) {
    const s = new Trajectory(app.knot!.scene);
    s.append(m.records); s.addGroups(m.groups, true); s.done = true;
    pendingPlay = s; pendingCut = m.cut;
    // waited for (and untouched since): replay the shortcut from the start. Otherwise swap now if the playback has barely
    // started or is over, or the run stalled (its tail is not shown); else at its end
    app.run.simplifying = false;
    if (liveAtDone && !player.playing) usePlayback(true);
    else { unfrost(); if (player.tau < 250 || player.ended() || app.run.reason === 'stalled') usePlayback(); }
  } else if (m.type === 'restart' && t && app.run.state === 'planning') {
    // the planner started over from the input with a new seed: drop the abandoned attempt
    const fresh = new Trajectory(app.knot!.scene);
    app.traj = fresh; phaseSeen = 0;
    const was = player.playing;
    player.setTrajectory(fresh); player.crossings.push({ r: 0, c: inView(fresh.initial) });
    if (was) { player.live = true; player.play(); } else player.live = false; // still frozen: stays so
    app.run.attempts = m.attempt + 1;
    updateStatus();
  } else if (m.type === 'error') {
    unfrost();
    app.run.state = 'done'; app.run.error = m.message;
    updateStatus(); updateRunButton();
  }
});

/** Play the shortcut trajectory instead of the planner's log (kept as app.raw); `replay`: from its start. */
function usePlayback(replay = false) {
  const s = pendingPlay, t = app.traj;
  pendingPlay = null;
  if (!s || !t) return;
  // at the end, or already past what the shortcut shows (a stalled run's tail): show its end
  const atEnd = player.ended() || (t.done && t.indexAt(player.tau) >= pendingCut), playing = player.playing;
  app.raw = t; app.traj = s;
  if (endedFor === t) endedFor = s;
  player.setTrajectory(s);
  player.crossings.push({ r: 0, c: inView(s.initial) });
  const ph = s.phases, step = Math.max(1, Math.ceil(ph.length / 150));
  for (let k = 0; k < ph.length; k += step) player.crossings.push({ r: ph[k].r, c: inView(s.stateAt(ph[k].r).S) });
  player.crossings.push({ r: s.len, c: inView(s.final) });
  if (replay) { liveAtDone = false; endedFor = null; unfrost(); player.play(); updateStatus(); }
  else if (atEnd) player.seek(player.duration()); else if (playing) player.play(); else player.render();
  player.drawSparks();
  updateStats(); updateRunButton();
}

function extendRun() {
  const t = app.traj;
  if (!t || app.run.state !== 'done' || app.run.solved || app.run.reason) return;
  t.done = false;
  app.run.state = 'planning';
  player.live = true;
  toPlanner({ type: 'extend', ms: 5000 });
  player.play();
  updateStatus(); updateRunButton();
}

/** No run: no trajectory, idle state for a curve of n sticks. */
export function clearRun(n: number) {
  app.traj = null; app.raw = null; pendingPlay = null; player.setTrajectory(null); phaseSeen = 0; endedFor = null; liveAtDone = false;
  app.run = freshRun('idle', n);
}
/** Discard the run and show the knot as loaded, ready to run again (same or new seed). */
export function resetRun() {
  const k = app.knot;
  if (!k) return;
  cancelRun();
  clearRun(totalVerts(k.scene));
  delete document.body.dataset.run;
  stage.snapFrame();
  draw(k.scene, 0);
  player.drawSparks();
  updateStatus(); updateRunButton();
}

export function runButton() {
  if (!app.knot || !runnable()) return;
  if (glass) {
    // "pause" under the glass: stop the planner and show where it is; no replay is started for you now
    takeOver();
    if (app.run.state === 'planning') { toPlanner({ type: 'pause' }); player.live = true; }
    player.pause(); player.render(); updateRunButton();
    return;
  }
  if (player.playing) { player.pause(); if (app.run.state === 'planning') toPlanner({ type: 'pause' }); return; }
  if (!app.traj) { startRun(); return; }
  if (app.run.state === 'planning') toPlanner({ type: 'resume' });
  player.play();
}

$('b-run').addEventListener('click', runButton);
// scrubbing under the glass takes over (capture: before the player's own handlers)
$('playarea').addEventListener('pointerdown', takeOver, true);
$('scrub').addEventListener('keydown', (e) => { if (['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(e.key)) takeOver(); }, true);
$('b-extend').addEventListener('click', extendRun);
$('b-reset').addEventListener('click', resetRun);

// ---------------------------------------------------------------- status, stats, run button

export function updateStatus() {
  const k = app.knot;
  if (editor.isOpen) return;
  if (!k) { setStatus(''); return; }
  setStatus(statusHTML(k.analysis, app.run));
  updateStats();
}
export function setStatus(html: string) { $('status').innerHTML = html; }

export function statusHTML(a: Analysis, r: RunState): string {
  if (a.kind === 'flat') return `${a.flat} crossing${a.flat > 1 ? 's' : ''} need over/under; click to set.`;
  if (a.kind === 'nonsplit') return `A link of ${a.comps} components; untangling links is not supported.`;
  if (r.state === 'planning') return 'Planning…';
  if (r.state === 'done' && r.error) return `Planner error: ${esc(r.error)}`;
  const more = r.simplifying ? ' Simplifying the replay…' : '';
  if (r.state === 'done' && a.knotted) return `Knotted: simplified to ${r.n} sticks in ${(r.plannerMs / 1000).toFixed(2)} s.${more}`;
  if (r.state === 'done') return r.solved ? `Solved in ${(r.plannerMs / 1000).toFixed(2)} s.${more}` : `Not solved after ${(r.plannerMs / 1000).toFixed(1)} s.`;
  return a.knotted ? 'Knotted.' : '';
}

const it = (s: string) => s.replace(/t/g, '<i>t</i>');
/** Determinant |Δ(−1)|. */
const det = (p: Laurent) => { let v = 0n; p.c.forEach((a, i) => { v += (i + p.lo) % 2 ? -a : a; }); return v < 0n ? -v : v; };

export function updateStats() {
  const t = app.traj, k = app.knot;
  if (!k) return;
  const rows: [string, string][] = [];
  const a = k.analysis;
  rows.push(['crossings', String(k.crossings)]);
  if (a.kind === 'ok') a.comps.forEach((c, i) => {
    const pre = a.comps.length > 1 ? `${i + 1}: ` : '';
    rows.push([`${pre}writhe`, String(c.writhe)]);
    if (c.alexander.kind === 'exact') rows.push([`${pre}Alexander Δ(<i>t</i>)`, it(format(c.alexander.poly))], [`${pre}det`, String(det(c.alexander.poly))]);
    else rows.push([`${pre}Alexander Δ(<i>t</i>)`, c.alexander.isOne ? '1 (mod <i>p</i>)' : '≠ 1']);
    rows.push([`${pre}Jones <i>V</i>(<i>t</i>)`, c.jones === 'skipped' ? 'too large' : it(format(c.jones))]);
  });
  else if (a.kind === 'nonsplit') rows.push(['components', String(a.comps)]);
  rows.push(['sticks', String(totalVerts(stage.shown ?? k.scene))], ['in view', `${app.lastCrossings} crossings`]);
  if (t) {
    // energy and clearance from the last Phase record at or before the displayed record
    let ph = null as (typeof t.phases)[number] | null;
    for (const p of t.phases) { if (p.r <= app.shownRecord) ph = p; else break; }
    rows.push(['record', `${app.shownRecord} / ${t.len}`]);
    if (ph && ph.n > 3 * k.scene.length) rows.push(['energy', ph.energy.toPrecision(4)], ['clearance', ph.clearance.toExponential(2)]);
    rows.push(['planner', `${(app.run.plannerMs / 1000).toFixed(2)} s${app.run.attempts > 1 ? `, attempt ${app.run.attempts}` : ''}`]);
  }
  const src = k.cite?.map((c) => (c.u ? `<a href="${esc(c.u)}" target="_blank" rel="noopener">${esc(c.t)}</a>` : esc(c.t))).join('; ') ?? (k.credit ? esc(k.credit) : '');
  if (src) rows.push(['source', `<span class="credit">${src}</span>`]);
  $('stats').innerHTML = rows.map(([x, y]) => `<dt>${x}</dt><dd>${y}</dd>`).join('');
}

export function updateRunButton() {
  const b = $('b-run') as HTMLButtonElement, a = app.knot?.analysis;
  b.disabled = !runnable() || editor.isOpen;
  let text = a?.kind === 'ok' && a.knotted ? 'simplify' : 'untangle';
  if (player.playing || glass) text = 'pause';
  else if (app.traj && app.traj.done && player.tau >= player.duration() - 1e-6) text = 'replay';
  else if (app.traj && player.tau > 0) text = 'play';
  b.firstElementChild!.textContent = text;
  ($('b-extend') as HTMLButtonElement).hidden = !(app.run.state === 'done' && !app.run.solved && !app.run.reason);
  ($('b-reset') as HTMLButtonElement).hidden = !app.traj;
}
