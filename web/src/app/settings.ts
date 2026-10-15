// Planner parameters, seed and budget (the panel's planner section), and the URL hash that carries them.
import { DEFAULT_PARAMS, type PlannerParams } from '../planner/run';
import { type Payload, encodePayload, formatHash, HASH_LIMIT } from '../state/url';
import { $, app } from './state';
import { stage } from './view';

type Key = keyof PlannerParams;
/** Settings fields in display (and link) order: hover hint, 0 allowed, non-integers allowed. */
const META: Partial<Record<Key, { hint?: string; zero?: true; real?: true }>> = {
  tau: { real: true },
  optSteps: {},
  optStep: { real: true },
  lipSteps: {},
  treeNodes: {},
  tailRounds: {},
  forcePower: { hint: 'repulsion ~ 1/dᵖ (repo: 1)' },   // an integer, so 1/dᵖ stays exact
  softMargin: { real: true, zero: true, hint: 'moves keep strands this far apart (or no closer than now) once the curve is down to 40 sticks; 0.005 before (repo: 0)' },
  marginStall: { zero: true, hint: 'the margin halves after this many iterations without progress, down to 0.005 (0 = fixed margin at all times)' },
  straighten: { real: true, zero: true, hint: 'opt also pulls vertices toward their neighbours’ midpoint (repo: 0)' },
  maxEdge: { real: true, zero: true, hint: 'no deletion makes an edge longer than this × the mean edge; doubles when stalled (0 = off; repo: 0)' },
  pickup: { zero: true, hint: 'a jammed curve (over 40 sticks) lifts strands that pass over everything onto routes crossing fewer strands, after this many iterations without progress (0 = off; repo: 0)' },
  hybrid: { zero: true, hint: 'heuristic: a random-tree step after this many iterations without progress (repo: 0 = off)' },
  restart: { zero: true, hint: 'start over with a new seed after this many iterations without progress (0 = never)' },
};
const KEYS = Object.keys(META) as Key[];
export const params: PlannerParams = { ...DEFAULT_PARAMS };

function changedParams(): Record<string, number> {
  const out: Record<string, number> = {};
  for (const k of KEYS) if (params[k] !== DEFAULT_PARAMS[k]) out[k] = params[k];
  if (app.budgetMs !== 10_000) out.budget = app.budgetMs / 1000;
  return out;
}
export function applyParams(p: Record<string, number>) {
  for (const k of KEYS) if (typeof p[k] === 'number') params[k] = p[k];
  if (typeof p.budget === 'number') { app.budgetMs = p.budget * 1000; budgetEl.value = String(p.budget); }
  renderSettings();
}

const settings = $('settings');
function renderSettings() {
  settings.innerHTML = KEYS.map((k) => `<div class="row"><label for="p-${k}" title="${META[k]!.hint ?? ''}">${k}</label><input type="number" id="p-${k}" step="any" value="${params[k]}"></div>`).join('')
    + `<div class="row"><button class="btn" id="b-defaults"><span>defaults</span></button></div>`;
}
renderSettings();
settings.addEventListener('change', (e) => {
  const el = e.target as HTMLInputElement, k = el.id.slice(2) as Key;
  if (!KEYS.includes(k)) return;
  const v = Number(el.value), m = META[k]!;
  const ok = Number.isFinite(v) && (m.zero ? v >= 0 : v > 0) && (m.real || Number.isInteger(v));
  if (ok) params[k] = v; else el.value = String(params[k]);
  updateHash();
});
settings.addEventListener('click', (e) => {
  const b = (e.target as HTMLElement).closest('button');
  if (b?.id === 'b-defaults') { Object.assign(params, DEFAULT_PARAMS); renderSettings(); updateHash(); }
});
$('b-settings').addEventListener('click', () => { settings.hidden = !settings.hidden; $('b-settings').setAttribute('aria-expanded', String(!settings.hidden)); });

const seedEl = $<HTMLInputElement>('i-seed'), budgetEl = $<HTMLInputElement>('i-budget');
export function setSeed(s: number) { app.seed = s; seedEl.value = String(s); }
seedEl.addEventListener('change', () => { setSeed(Math.max(0, Math.min(4294967295, Math.floor(Number(seedEl.value) || 0)))); updateHash(); });
$('b-reseed').addEventListener('click', () => { setSeed((Math.random() * 4294967296) >>> 0); updateHash(); });
budgetEl.addEventListener('change', () => { const s = Math.max(1, Math.min(600, Number(budgetEl.value) || 10)); budgetEl.value = String(s); app.budgetMs = s * 1000; updateHash(); });

/** Put the knot, seed, view and changed parameters in the URL; true when the knot is linkable. */
export async function updateHash(): Promise<boolean> {
  const k = app.knot;
  let enc: string | undefined;
  if (k?.payload) {
    const p = { ...k.payload, ...(Object.keys(changedParams()).length ? { params: changedParams() } : {}) } as Payload;
    enc = await encodePayload(p);
  }
  const h = formatHash({ k: enc, s: app.seed, view: stage.mode, sticks: stage.sticks });
  const linkable = !!enc && h.length <= HASH_LIMIT;
  history.replaceState(null, '', linkable ? h : formatHash({ s: app.seed, view: stage.mode, sticks: stage.sticks }));
  ($('e-link') as HTMLButtonElement).disabled = !linkable;
  $('export-note').textContent = k && !linkable ? (k.payload ? 'Link too long; save the [ knot ] file instead.' : 'This knot cannot be linked; save the [ knot ] file instead.') : '';
  return linkable;
}
