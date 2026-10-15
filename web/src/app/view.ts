// The figure: stage (2D + 3D), Reidemeister marks, follow view, view toggles, draw editor, theme.
import type { Scene } from '../kernel/scene';
import type { View } from '../diagram/project';
import type { Ghost } from '../planner/trajectory';
import type { Analysis } from '../invariants';
import { Stage } from '../ui/stage';
import { Editor } from '../ui/editor';
import { request } from '../state/workers';
import { $, app, freshRun, press, NAME } from './state';
import { player, cancelRun, updateRunButton, updateStatus, updateStats, setStatus, statusHTML } from './run';
import { loadKnot, showError } from './load';
import { updateHash } from './settings';

export const stage = new Stage($('stage'), $('view2d') as unknown as SVGSVGElement, $<HTMLCanvasElement>('view3d'), $('notice'), {
  annot: () => app.annot, onBestView: () => runBestView(),
});
// in 3D the 2D drawing is redone on demand, so its crossing count is refreshed at most every 250 ms
let countTimer = 0;
const countCrossings = () => { app.lastCrossings = stage.drawn?.crossings.length ?? 0; };
stage.onChange = () => {
  if (stage.mode === '2d') countCrossings();
  else if (!countTimer) countTimer = window.setTimeout(() => { countTimer = 0; countCrossings(); updateStats(); }, 250);
  updateStats();
};
if (!stage.has3d) {
  for (const b of ['b-3d', 'b-ortho', 'b-persp']) { ($(b) as HTMLButtonElement).disabled = true; $(b).title = '3d needs WebGL2'; }
  stage.disable3d('3d needs WebGL2');
}

export function draw(S: Scene, r = app.shownRecord, ghost: Ghost | null = null) {
  if (!app.knot) return;
  app.shownRecord = r;
  stage.show(S, ghost);
  followView();
  reidemeister();
}

// Reidemeister marks during playback: one crossing appears or vanishes at one spot (R1), two on one strand pair (R2)
let prevX: { x: number; y: number; ca: number; cb: number }[] = [];
const marks = document.createElementNS('http://www.w3.org/2000/svg', 'g');
stage.r2d.svg.append(marks);   // after the stage's own layers, so marks sit on top
function reidemeister() {
  if (stage.mode !== '2d') { prevX = []; return; }
  const cur = (stage.drawn?.crossings ?? []).map((X) => ({ x: X.x, y: X.y, ca: Math.min(X.a.comp, X.b.comp), cb: Math.max(X.a.comp, X.b.comp) }));
  const before = prevX; prevX = cur;
  if (!app.reid || !player.playing || !before.length) return;
  const unmatched = (A: typeof cur, B: typeof cur) => A.filter((p) => !B.some((q) => Math.hypot(p.x - q.x, p.y - q.y) < 6));
  const changed = [...unmatched(cur, before), ...unmatched(before, cur)];
  let label = '', at: { x: number; y: number } | null = null;
  if (Math.abs(cur.length - before.length) === 1 && changed.length === 1) { label = 'R1'; at = changed[0]; }
  else if (Math.abs(cur.length - before.length) === 2 && changed.length === 2 && changed[0].ca === changed[1].ca && changed[0].cb === changed[1].cb && Math.hypot(changed[0].x - changed[1].x, changed[0].y - changed[1].y) < 60) {
    label = 'R2'; at = { x: (changed[0].x + changed[1].x) / 2, y: (changed[0].y + changed[1].y) / 2 };
  }
  if (!at) return;
  const t = document.createElementNS('http://www.w3.org/2000/svg', 'text');
  t.setAttribute('class', 'rmark'); t.setAttribute('x', String(at.x + 10)); t.setAttribute('y', String(at.y - 10)); t.textContent = label;
  marks.append(t);
  t.animate([{ opacity: 1 }, { opacity: 0 }], { duration: 600, easing: 'ease-in' }).onfinish = () => t.remove();
}

// follow view: every second of playback, re-score; the camera eases toward any view with fewer crossings
let followAt = 0, following = false;
function followView() {
  if (!app.follow || !player.playing || following || !stage.shown) return;
  const now = performance.now();
  if (now - followAt < 1000) return;
  followAt = now; following = true;
  const S = stage.shown;
  request<View>({ kind: 'bestView', scene: S, view: stage.view }).then((v) => {
    if (app.follow && stage.crossingsAlong(S, v) < stage.crossingsAlong(S, stage.view)) stage.follow(v);
  }).finally(() => { following = false; });
}
// turning the camera by hand stops following
stage.onManual = () => { if (app.follow) { app.follow = false; press('b-follow', false); } };

/** Ease the camera to the best view of S (default: what is shown). */
export async function runBestView(S = stage.shown ?? app.knot?.scene) {
  if (!S) return;
  await stage.animateView(await request<View>({ kind: 'bestView', scene: S, view: stage.view }));
}

export async function flip(to?: '2d' | '3d') {
  const target = to ?? (stage.mode === '2d' ? '3d' : '2d');
  if (target === '3d' && !stage.has3d) return;
  press('b-2d', target === '2d'); press('b-3d', target === '3d');
  await stage.flip(target);
  // a flip requested mid-flip is ignored: show where the stage actually ends up
  press('b-2d', stage.mode === '2d'); press('b-3d', stage.mode === '3d');
  document.body.dataset.view = stage.mode;
  updateHash();
}
/** Curve or ball and stick, in either view. */
export function setSticks(on: boolean) {
  stage.sticks = on;
  press('b-curve', !on); press('b-sticks', on);
  document.body.dataset.style = on ? 'sticks' : 'curve';
  stage.redraw();
  updateHash();
}

$('b-best').addEventListener('click', () => runBestView());
$('b-2d').addEventListener('click', () => flip('2d'));
$('b-3d').addEventListener('click', () => flip('3d'));
$('b-curve').addEventListener('click', () => setSticks(false));
$('b-sticks').addEventListener('click', () => setSticks(true));
for (const [id, on] of [['b-ortho', false], ['b-persp', true]] as const) $(id).addEventListener('click', () => {
  press('b-ortho', !on); press('b-persp', on);
  stage.setPersp(on);
});
$('b-follow').addEventListener('click', () => { app.follow = !app.follow; press('b-follow', app.follow); if (!app.follow) stage.follow(null); });
$('a-reid').addEventListener('click', () => { app.reid = !app.reid; press('a-reid', app.reid); });
for (const k of ['numbers', 'signs', 'chevrons'] as const) $(`a-${k}`).addEventListener('click', () => {
  app.annot[k] = !app.annot[k];
  press(`a-${k}`, app.annot[k]);
  stage.redraw();
});

// ---------------------------------------------------------------- draw editor

export const editor = new Editor($('stage'), {
  status: setStatus,
  analyze: async (d) => statusHTML(await request<Analysis>({ kind: 'invariants', diagram: d }), freshRun('idle')),
  done: (d) => { closeEditor(); loadKnot({ diagram: d }, NAME.draw, { kind: 'draw' }).catch(showError); },
  cancel: () => closeEditor(),
});
export function openEditor() {
  if (editor.isOpen) return;
  cancelRun();
  if (stage.mode === '3d') flip('2d');
  $('stage').classList.add('drawing');
  press('b-draw', true);
  editor.open();
  updateRunButton();
}
export function closeEditor() {
  editor.close();
  $('stage').classList.remove('drawing');
  press('b-draw', false);
  updateStatus();
}
$('b-draw').addEventListener('click', () => (editor.isOpen ? closeEditor() : openEditor()));

// ---------------------------------------------------------------- theme, remembered per browser

const isDark = () => (document.documentElement.dataset.theme ? document.documentElement.dataset.theme === 'dark' : matchMedia('(prefers-color-scheme: dark)').matches);
function labelTheme() { const d = isDark(); $('b-theme').setAttribute('aria-label', d ? 'switch to light mode' : 'switch to dark mode'); $('b-theme').title = d ? 'light mode' : 'dark mode'; }
try { const th = localStorage.getItem('unknot.theme'); if (th === 'dark' || th === 'light') { document.documentElement.dataset.theme = th; stage.v3d?.readColors(); } } catch { /* storage blocked */ }
labelTheme();
$('b-theme').addEventListener('click', () => {
  const th = isDark() ? 'light' : 'dark';
  document.documentElement.dataset.theme = th;
  try { localStorage.setItem('unknot.theme', th); } catch { /* private mode */ }
  labelTheme(); stage.v3d?.readColors(); stage.redraw();
});
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', labelTheme);
