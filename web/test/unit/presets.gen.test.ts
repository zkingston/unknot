import { expect, test } from 'vitest';
import { existsSync, writeFileSync } from 'node:fs';
import { makePresets, makeInvariants, KNOTTED } from '../../tools/make-presets';
import { parseKnotJSON } from '../../src/import/json';
import { analyze } from '../../src/invariants';
import { validateDiagram } from '../../src/diagram/crossings';
import { clearance } from '../../src/kernel/energy';
import { format } from '../../src/invariants/laurent';
import { json } from './util';

test.skipIf(!process.env.GEN_PRESETS)('write src/presets/<known knot>.knot.json and invariants.json', () => {
  for (const [f, text] of Object.entries(makePresets())) writeFileSync(new URL(`../../src/presets/${f}`, import.meta.url), text);
  writeFileSync(new URL('../../src/presets/invariants.json', import.meta.url), makeInvariants());
});

test('baked preset invariants are what loading would compute (else: GEN_PRESETS=1 to regenerate)', () => {
  expect(JSON.parse(makeInvariants())).toEqual(json('../../src/presets/invariants.json'));
});

const want: Record<string, string> = { trefoil: 't − 1 + t⁻¹', 'figure-eight': '−t + 3 − t⁻¹', cinquefoil: 't² − t + 1 − t⁻¹ + t⁻²', 'three-twist': '2t − 3 + 2t⁻¹' };
for (const id of Object.keys(KNOTTED)) test(`preset ${id}: regular diagram, embedded scene, knotted invariants, matches the generator`, () => {
  const path = `../../src/presets/${id}.knot.json`;
  expect(existsSync(new URL(path, import.meta.url))).toBe(true);
  const l = parseKnotJSON(json(path));
  expect(validateDiagram(l.diagram!).ok).toBe(true);
  expect(clearance(l.scene!)).toBeGreaterThan(0);
  const a = analyze(l.diagram!);
  expect(a.kind === 'ok' && a.knotted).toBe(true);
  expect(a.kind === 'ok' && a.comps[0].alexander.kind === 'exact' && format(a.comps[0].alexander.poly)).toBe(want[id]);
  // committed file = what the generator produces now (deterministic layout)
  expect(JSON.parse(makePresets()[`${id}.knot.json`])).toEqual(json(path));
});
