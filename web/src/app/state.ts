// App state shared by the modules in src/app, and small DOM helpers.
import type { Scene } from '../kernel/scene';
import type { Diagram } from '../diagram/types';
import type { View } from '../diagram/project';
import type { Trajectory } from '../planner/trajectory';
import type { Payload } from '../state/url';
import type { Analysis } from '../invariants';
import type { Cite } from '../import/json';

export const $ = <T extends HTMLElement = HTMLElement>(id: string) => document.getElementById(id) as T;
export const esc = (s: string) => s.replace(/[&<>"']/g, (c) => `&#${c.charCodeAt(0)};`);
export const press = (id: string, on: boolean) => $(id).setAttribute('aria-pressed', String(on));

export type CodeFormat = 'pd' | 'gauss' | 'braid';
export interface Knot {
  name: string; credit?: string; cite?: Cite[]; diagram: Diagram; scene: Scene; analysis: Analysis; view: View;
  crossings: number; source: Source;
  payload: Payload | null;   // what a share link carries; null = not linkable
}
export type Source = { kind: 'preset'; id: string } | { kind: 'text'; format: CodeFormat; text: string } | { kind: 'draw' } | { kind: 'image' } | { kind: 'svg' } | { kind: 'json' };
/** Default knot names by source. */
export const NAME = { draw: 'Drawn knot', image: 'Imported image', svg: 'Imported SVG', json: 'Linked knot', pd: 'PD code', gauss: 'Gauss code', braid: 'Braid word' } as const;

export type RunState = { state: 'idle' | 'planning' | 'done'; solved: boolean; plannerMs: number; n: number; reason: string; attempts: number; error: string; simplifying?: boolean };
export const freshRun = (state: RunState['state'], n = 0): RunState => ({ state, solved: false, plannerMs: 0, n, reason: '', attempts: 1, error: '' });

export const app = {
  knot: null as Knot | null,
  traj: null as Trajectory | null,  // what plays: the planner's log, then its shortcut once the run is solved
  raw: null as Trajectory | null,   // the planner's own log after the swap
  run: freshRun('idle'),
  seed: 7,
  budgetMs: 10_000,
  annot: { numbers: false, signs: false, chevrons: false },
  follow: false,                    // camera eases toward the best view during playback
  reid: false,
  shownRecord: 0,
  lastCrossings: 0,                 // in the 2D drawing of the shown state
};

export function runnable() {
  const a = app.knot?.analysis;
  return !!a && a.kind === 'ok';
}
