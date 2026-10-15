import goeritz from './goeritz.json';
import ochiai from './ochiai.json';
import thistlethwaite from './thistlethwaite.json';
import trefoil from './trefoil.knot.json';
import figureEight from './figure-eight.knot.json';
import cinquefoil from './cinquefoil.knot.json';
import threeTwist from './three-twist.knot.json';
import hard from './hard-unknots.json';
import invariants from './invariants.json';
import { parseKnotJSON, type Loaded, type Cite } from '../import/json';
import { analysisFromJSON } from '../invariants';
import { normalizeScene } from '../kernel/normalize';
import { diagramFromCurves } from '../diagram/crossings';
import type { Diagram, StrandRef } from '../diagram/types';

export interface Preset { id: string; name: string; label: string; group: string; cite?: Cite[]; load: () => Loaded }
const credit = (c: Cite[]) => c.map((x) => x.t).join('; ');

/** Baked orthogonal layout (tools/hard-unknots.py): grid polylines per component, and (under, over) global segment
 *  index pairs, one per crossing. */
interface Layout { v: number[][]; x: number[] }
const R = 0.4; // corner radius in grid units; < 0.5 keeps arcs clear of neighbouring grid lines and crossings
function layoutDiagram(L: Layout): Diagram {
  const comps: Float64Array[] = [], orig: number[][] = [];
  let base = 0;
  for (const V of L.v) {
    const n = V.length / 2, out: number[] = [], o: number[] = [];
    for (let i = 0; i < n; i++) {
      const a = (i + n - 1) % n, c = (i + 1) % n, bx = V[2 * i], by = V[2 * i + 1];
      const ux = V[2 * a] - bx, uy = V[2 * a + 1] - by, wx = V[2 * c] - bx, wy = V[2 * c + 1] - by;
      const lu = Math.hypot(ux, uy), lw = Math.hypot(wx, wy);
      if (Math.abs(ux * wy - uy * wx) < 1e-9) { out.push(bx, by); o.push(base + i); continue; }
      // quarter arc as a quadratic Bézier with the corner as control point; only its last point starts a grid segment
      for (let k = 0; k <= 4; k++) {
        const t = k / 4, p = (1 - t) * (1 - t) * R, q = t * t * R;
        out.push(bx + (p * ux) / lu + (q * wx) / lw, by + (p * uy) / lu + (q * wy) / lw);
        o.push(k === 4 ? base + i : -1);
      }
    }
    comps.push(Float64Array.from(out)); orig.push(o); base += n;
  }
  const M = 1 << 16, under = new Set<number>();
  for (let i = 0; i < L.x.length; i += 2) under.add(L.x[i] * M + L.x[i + 1]);
  const g = (r: StrandRef) => orig[r.comp][r.seg];
  return diagramFromCurves(comps, 'preset', (X) => (under.has(g(X.b) * M + g(X.a)) ? true : under.has(g(X.a) * M + g(X.b)) ? false : null));
}
const baked = (L: Layout, name: string, cite: Cite[]) => (): Loaded => ({ diagram: { ...layoutDiagram(L), meta: { name, credit: credit(cite) } }, name, credit: credit(cite), cite });

const legacy = (pts: number[][], cite: Cite[]) => () => { const l = parseKnotJSON(pts); return { ...l, credit: credit(cite), cite, scene: normalizeScene(l.scene!) }; };
const GOERITZ: Cite[] = [{ t: 'Goeritz, 1934', u: 'https://doi.org/10.1007/BF02940674' }];
const PZ_CURTIS: Cite[] = [{ t: 'Petronio & Zanellati, 2016', u: 'https://doi.org/10.1142/S0218216516500590' }, { t: 'Curtis, 2001', u: 'http://f2.org/maths/kt/unknoteq.html' }];
const OCHIAI: Cite[] = [{ t: 'Ochiai, 1990', u: 'https://www.numdam.org/item/AST_1990__192__7_0/' }];
const KNOT_ATLAS: Cite[] = [{ t: 'The Knot Atlas, n.d.', u: 'http://katlas.org/' }];
const known = (j: unknown) => (): Loaded => ({ ...parseKnotJSON(j), credit: credit(KNOT_ATLAS), cite: KNOT_ATLAS });

// 3D polygons from the C++ repo (zkingston/unknot), with the crossing counts of their diagrams
const repo = [
  { id: 'goeritz', name: 'Goeritz', crossings: 11, cite: GOERITZ, load: legacy(goeritz, GOERITZ) },
  { id: 'thistlethwaite', name: 'Thistlethwaite', crossings: 15, cite: PZ_CURTIS, load: legacy(thistlethwaite, PZ_CURTIS) },
  { id: 'ochiai', name: 'Ochiai I', crossings: 13, cite: OCHIAI, load: legacy(ochiai, OCHIAI) },
];

/** Invariants computed ahead of time (tools/make-presets.ts), by preset id: loading skips the invariants worker. */
const INVARIANTS = invariants as Record<string, unknown>;
const withInvariants = (p: Preset): Preset => (INVARIANTS[p.id] ? { ...p, load: () => ({ ...p.load(), analysis: analysisFromJSON(INVARIANTS[p.id]) }) } : p);

export const PRESETS: Preset[] = ([
  ...[...hard.named.map((k) => ({ id: k.id, name: k.name, crossings: k.crossings, cite: k.cite, load: baked(k.layout, k.name, k.cite) })), ...repo]
    .sort((a, b) => a.crossings - b.crossings)
    .map(({ crossings, ...p }) => ({ ...p, label: `${p.name}, ${crossings} crossings`, group: 'hard unknots' })),
  { id: 'trefoil', name: 'Trefoil', label: 'Trefoil', group: 'known knots', cite: KNOT_ATLAS, load: known(trefoil) },
  { id: 'figure-eight', name: 'Figure-eight', label: 'Figure-eight', group: 'known knots', cite: KNOT_ATLAS, load: known(figureEight) },
  { id: 'cinquefoil', name: 'Cinquefoil', label: 'Cinquefoil', group: 'known knots', cite: KNOT_ATLAS, load: known(cinquefoil) },
  { id: 'three-twist', name: 'Three-twist', label: 'Three-twist', group: 'known knots', cite: KNOT_ATLAS, load: known(threeTwist) },
] as Preset[]).map(withInvariants);

export const VERY_HARD = hard.veryHard.items.length;

/** `deepmind:<i>` is diagram i of DeepMind's very hard unknots (sorted by crossing number). */
export function presetById(id: string): Preset | undefined {
  const m = /^deepmind:(\d+)$/.exec(id);
  if (m) {
    const i = Number(m[1]), it = hard.veryHard.items[i];
    if (it === undefined) return undefined;
    const name = `DeepMind very hard #${i}`;
    return { id, name, label: `#${i}, ${it.n} crossings`, group: 'DeepMind very hard unknots', cite: hard.veryHard.cite, load: baked(it.layout, name, hard.veryHard.cite) };
  }
  return PRESETS.find((p) => p.id === id);
}
