// Loading: presets, codes, drawings, files and links all end in loadKnot.
import type { Scene } from '../kernel/scene';
import { totalVerts } from '../kernel/scene';
import { type View, TOP, projectScene } from '../diagram/project';
import type { Analysis } from '../invariants';
import { type Payload, type QuantDiagram, commitDiagram, commitScene, decodePayload, parseHash, dequantizeScene, dequantizeDiagram } from '../state/url';
import { presetById } from '../presets';
import type { Loaded } from '../import/json';
import type { CodeResult } from '../import/import.worker';
import { request } from '../state/workers';
import { $, app, NAME, type CodeFormat, type Source } from './state';
import { stage, draw, editor, closeEditor, flip, setSticks } from './view';
import { player, cancelRun, clearRun, updateStatus, updateRunButton } from './run';
import { applyParams, setSeed, updateHash } from './settings';

let loadToken = 0;
/** Lifted Scenes of the presets loaded this session (lifting is deterministic; nothing mutates a knot's Scene). */
const lifted = new Map<string, Scene>();
/** Thrown (and swallowed) when a newer load supersedes this one. */
class Superseded extends Error {}

export async function loadKnot(l: Loaded, name: string, source: Source, opts: { params?: Record<string, number> } = {}) {
  const token = ++loadToken, live = () => { if (token !== loadToken) throw new Superseded(); };
  cancelRun();
  if (editor.isOpen) closeEditor();
  // quantise on commit so the in-memory knot is exactly what a link reloads
  let payload: Payload | null = null;
  if (source.kind === 'preset') payload = { src: 'preset', data: source.id };
  else if (source.kind === 'text') payload = { src: source.format, data: source.text };
  else if (l.diagram && !l.scene) {
    const c = commitDiagram(l.diagram);
    if (c) { l = { ...l, diagram: c.diagram }; payload = { src: source.kind === 'image' || source.kind === 'svg' ? source.kind : 'draw', data: c.q }; }
  } else if (l.scene && !l.diagram) {
    const c = commitScene(l.scene);
    if (c) { l = { ...l, scene: c.scene }; payload = { src: 'json', data: c.q }; }
  }
  let diagram = l.diagram, scene: Scene, view: View = TOP;
  if (diagram && l.scene) scene = l.scene;   // precomputed (presets, saved files)
  else if (diagram) {
    const hit = source.kind === 'preset' ? lifted.get(source.id) : undefined;
    if (hit) scene = hit;
    else {
      const r = await request<{ scene: Scene }>({ kind: 'lift', diagram });
      live();
      scene = r.scene;
      if (source.kind === 'preset') lifted.set(source.id, scene);
    }
  } else {
    scene = l.scene!;
    view = await request<View>({ kind: 'bestView', scene, view: TOP });
    live();
    diagram = projectScene(scene, view);
  }
  const analysis = l.analysis ?? await request<Analysis>({ kind: 'invariants', diagram });
  live();
  app.knot = { name: l.name && !name ? l.name : name, credit: l.credit, cite: l.cite, diagram, scene, analysis, view, crossings: diagram.crossings.length, source, payload };
  clearRun(totalVerts(scene));
  if (opts.params) applyParams(opts.params);
  stage.view = view;
  stage.snapFrame();
  draw(scene, 0);
  player.drawSparks();
  updateStatus(); updateRunButton();
  document.body.dataset.knot = name;
  updateHash();
}

export function showError(e: unknown) {
  if (e instanceof Superseded) return;
  $('status').textContent = e instanceof Error ? e.message : String(e);
}

export function loadPreset(id: string, params?: Record<string, number>) {
  const p = presetById(id);
  loadToken++; // a click supersedes any load in flight, even before its own layout returns
  if (!p) return Promise.reject(new Error(`unknown preset ${id}`)).catch(showError);
  return Promise.resolve().then(p.load).then((l) => loadKnot(l, p.name, { kind: 'preset', id }, params ? { params } : {})).catch(showError);
}

/** Parse a PD, Gauss or braid code in the worker and load it; throws on a bad code. */
export async function loadText(format: CodeFormat, text: string, opts: { params?: Record<string, number> } = {}) {
  const token = ++loadToken;
  const r = await request<CodeResult>({ kind: format, text });
  if (token !== loadToken) return; // superseded by a newer load
  await loadKnot({ diagram: r.diagram }, NAME[format], { kind: 'text', format, text }, opts);
}

export async function loadFromHash(): Promise<boolean> {
  const h = parseHash(location.hash);
  if (!h) return false;
  setSeed(h.s);
  if (!h.k) return false;
  const p = await decodePayload(h.k);
  const params = p.params ?? {};
  if (p.src === 'preset') { if (!presetById(p.data)) throw new Error(`unknown preset ${p.data}`); await loadPreset(p.data, params); }
  else if (p.src === 'pd' || p.src === 'gauss' || p.src === 'braid') await loadText(p.src, p.data, { params });
  else if (p.src === 'json') await loadKnot({ scene: dequantizeScene(p.data) }, NAME.json, { kind: 'json' }, { params });
  else await loadKnot({ diagram: dequantizeDiagram(p.data as QuantDiagram, p.src) }, NAME[p.src], { kind: p.src }, { params });
  if (h.sticks) setSticks(true);
  if (h.view === '3d') await flip('3d');
  return true;
}
