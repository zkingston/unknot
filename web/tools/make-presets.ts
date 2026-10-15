// Known-knot presets from Knot Atlas PD codes (3₁, 4₁, 5₁, 5₂), with the lifted Scene precomputed, and the
// invariants of every preset that ships a diagram (src/presets/invariants.json).
// Regenerate: GEN_PRESETS=1 npx vitest run test/unit/presets.gen.test.ts
import { parsePD } from '../src/import/pdcode';
import { pdToDiagram } from '../src/import/embed';
import { diagramToJSON } from '../src/import/json';
import { lift } from '../src/diagram/lift';
import { simplify } from '../src/diagram/simplify';
import { analyze, analysisToJSON } from '../src/invariants';
import { PRESETS } from '../src/presets';

export const KNOTTED = {
  trefoil: { name: 'Trefoil', pd: 'X[1,5,2,4], X[3,1,4,6], X[5,3,6,2]', credit: 'Knot Atlas' },
  'figure-eight': { name: 'Figure-eight', pd: 'X[4,2,5,1], X[8,6,1,5], X[6,3,7,4], X[2,7,3,8]', credit: 'Knot Atlas' },
  cinquefoil: { name: 'Cinquefoil', pd: 'X[1,6,2,7], X[3,8,4,9], X[5,10,6,1], X[7,2,8,3], X[9,4,10,5]', credit: 'Knot Atlas' },
  'three-twist': { name: 'Three-twist', pd: 'X[1,4,2,5], X[3,8,4,9], X[5,10,6,1], X[9,6,10,7], X[7,2,8,3]', credit: 'Knot Atlas' },
};

/** file name -> .knot.json text */
export function makePresets(): Record<string, string> {
  const out: Record<string, string> = {};
  for (const [id, k] of Object.entries(KNOTTED)) {
    const diagram = pdToDiagram(parsePD(k.pd), { source: 'pd', sourceText: k.pd, name: k.name });
    const scene = simplify(lift(diagram));
    out[`${id}.knot.json`] = JSON.stringify({ version: 1, name: k.name, credit: k.credit, diagram: diagramToJSON(diagram), scene: scene.map((P) => Array.from(P)) });
  }
  return out;
}

/** invariants.json text: the analysis of each preset's diagram, as loading computes it. */
export function makeInvariants(): string {
  const out: Record<string, unknown> = {};
  for (const p of PRESETS) { const d = p.load().diagram; if (d) out[p.id] = JSON.parse(analysisToJSON(analyze(d))); }
  return JSON.stringify(out) + '\n';
}
