// .knot.json and legacy [[x,y,z], ...] import.
import type { Diagram, Crossing } from '../diagram/types';
import type { Scene } from '../kernel/scene';
import type { Analysis } from '../invariants';
import { touches } from '../kernel/energy';

export interface KnotFile { version: 1; name?: string; credit?: string; diagram?: DiagramJSON; scene?: number[][] | number[][][] }
export interface DiagramJSON { comps: number[][]; crossings: Crossing[]; source?: Diagram['source']; meta?: Diagram['meta'] }
/** An APA in-text citation (`Author, year`) and where it resolves (DOI link when there is one). */
export interface Cite { t: string; u?: string }
export interface Loaded { name?: string; credit?: string; cite?: Cite[]; diagram?: Diagram; scene?: Scene; analysis?: Analysis; legacy?: boolean }

function diagramFromJSON(j: DiagramJSON): Diagram {
  return {
    comps: j.comps.map((c) => Float64Array.from(c)),
    crossings: j.crossings.map((X) => ({ under: { ...X.under }, over: { ...X.over }, ...(X.flat ? { flat: true as const } : {}) })),
    source: j.source ?? 'json', ...(j.meta ? { meta: j.meta } : {}),
  };
}
export const diagramToJSON = (d: Diagram): DiagramJSON => ({ comps: d.comps.map((c) => Array.from(c)), crossings: d.crossings, source: d.source, ...(d.meta ? { meta: d.meta } : {}) });

const isPoint = (p: unknown) => Array.isArray(p) && p.length === 3 && p.every((x) => typeof x === 'number' && Number.isFinite(x));

export function parseKnotJSON(text: string | unknown): Loaded {
  const j = typeof text === 'string' ? JSON.parse(text) : text;
  if (Array.isArray(j)) {
    if (j.length && Array.isArray(j[0]) && Array.isArray(j[0][0])) throw new Error('this is a trajectory; open it as a trajectory');
    if (j.length < 3 || !j.every(isPoint)) throw new Error('expected [[x,y,z], ...] with at least 3 points');
    const scene: Scene = [Float64Array.from(j.flat())];
    if (touches(scene)) throw new Error('input polygon intersects itself');
    return { scene, legacy: true };
  }
  if (!j || typeof j !== 'object' || j.version !== 1) throw new Error('unrecognised JSON');
  const f = j as KnotFile, out: Loaded = { name: f.name, credit: f.credit };
  if (f.diagram) out.diagram = diagramFromJSON(f.diagram);
  if (f.scene) {
    const S = f.scene;
    out.scene = Array.isArray(S[0]?.[0]) ? (S as number[][][]).map((c) => Float64Array.from(c.flat())) : (S as number[][]).map((c) => Float64Array.from(c));
    if (touches(out.scene)) throw new Error('input polygon intersects itself');
  }
  if (!out.diagram && !out.scene) throw new Error('.knot.json has neither diagram nor scene');
  return out;
}
