import { expect, test } from 'vitest';
import hard from '../../src/presets/hard-unknots.json';
import { presetById, VERY_HARD } from '../../src/presets';
import { validateDiagram } from '../../src/diagram/crossings';
import { lift } from '../../src/diagram/lift';
import { simplify } from '../../src/diagram/simplify';
import { analyze } from '../../src/invariants';
import { runSync } from '../../src/planner/run';
import { checkIsotopy } from './isotopy';

const prep = (id: string) => { const d = presetById(id)!.load().diagram!; return { d, S: simplify(lift(d)) }; };

test('every named hard unknot lays out regularly, lifts, has invariants that find no knot, and the planner untangles it', () => {
  for (const k of hard.named) {
    const t0 = performance.now(), { d, S } = prep(k.id), t1 = performance.now();
    expect(validateDiagram(d).ok, k.name).toBe(true);
    expect(d.crossings.length, k.name).toBe(k.crossings);
    const a = analyze(d);
    expect(a.kind === 'ok' && !a.knotted, k.name).toBe(true);
    // seed 7 first; the hardest (Petronio–Zanellati 120) needs another seed within the budget
    let r = runSync(S, {}, 7, 60_000);
    for (const seed of [1, 2]) if (!r.solved) r = runSync(S, {}, seed, 60_000);
    const iso = checkIsotopy(S, r.log.records);
    console.log(`${k.name.padEnd(26)} ${String(k.crossings).padStart(3)} crossings, ${S[0].length / 3} sticks, layout+lift ${(t1 - t0).toFixed(0)} ms, planner ${r.ms.toFixed(0)} ms ${r.solved ? 'solved' : 'UNSOLVED'} (attempt ${r.attempts}), passages ${iso.passages}`);
    expect(r.solved, k.name).toBe(true);
    expect(iso.passages, k.name).toBe(0);
  }
}, 600_000);

test('all DeepMind very hard unknots lay out and lift; a sample solves', () => {
  let solved = 0, tried = 0, worst = 0;
  for (let i = 0; i < VERY_HARD; i++) {
    const { d, S } = prep(`deepmind:${i}`);
    expect(validateDiagram(d).ok, `#${i}`).toBe(true);
    expect(d.crossings.length, `#${i}`).toBe(hard.veryHard.items[i].n);
    if (i % 16 === 0) { tried++; const r = runSync(S, {}, 7, 10_000); if (r.solved) solved++; worst = Math.max(worst, r.ms); }
  }
  console.log(`DeepMind sample: ${solved}/${tried} solved, worst ${worst.toFixed(0)} ms`);
  expect(solved).toBe(tried);
}, 900_000);
