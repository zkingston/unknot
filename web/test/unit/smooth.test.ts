import { expect, test } from 'vitest';
import { smoothScene } from '../../src/diagram/smooth';
import { projectScene } from '../../src/diagram/project';
import { analyze } from '../../src/invariants';
import { format } from '../../src/invariants/laurent';
import { clearance } from '../../src/kernel/energy';
import { rand } from './util';
import type { Scene } from '../../src/kernel/scene';

/** Jones polynomial of the top projection, or null when it is flat or too large. */
function jonesOf(S: Scene): string | null {
  const a = analyze(projectScene(S));
  if (a.kind !== 'ok') return null;
  const j = a.comps[0].jones;
  return j === 'skipped' ? null : format(j);
}

test('smoothing random polygons near self-contact keeps the knot type and cuts most corners', () => {
  const r = rand(77);
  let tried = 0, knotted = 0, cut = 0, possible = 0;
  for (let k = 0; k < 300 && tried < 120; k++) {
    const n = 8 + Math.floor(r() * 12), P = new Float64Array(3 * n);
    // a squashed random polygon: strands pass close to each other in z
    for (let i = 0; i < n; i++) { P[3 * i] = 2 * r() - 1; P[3 * i + 1] = 2 * r() - 1; P[3 * i + 2] = 0.02 * (2 * r() - 1); }
    const S = [P];
    if (!(clearance(S) > 1e-4)) continue;
    const j0 = jonesOf(S);
    if (j0 === null) continue;
    const sm = smoothScene(S), j1 = jonesOf(sm);
    if (j1 === null) continue;
    tried++;
    if (j0 !== '1') knotted++;
    expect(j1, `polygon ${k}`).toBe(j0);
    expect(clearance(sm)).toBeGreaterThan(0);
    cut += sm[0].length / 3 - n; possible += 7 * n;
  }
  expect(tried).toBeGreaterThan(100);
  expect(knotted).toBeGreaterThan(10);
  expect(cut / possible, 'most corners are still cut').toBeGreaterThan(0.6);
});
