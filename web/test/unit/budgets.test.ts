// Performance budgets that run in Node (SPEC "Performance budgets"); browser-side ones are in e2e.
import { expect, test } from 'vitest';
import { parseKnotJSON } from '../../src/import/json';
import { lift } from '../../src/diagram/lift';
import { simplify } from '../../src/diagram/simplify';
import { braidToDiagram, parseBraid } from '../../src/import/braid';
import { pdCode } from '../../src/invariants/pd';
import { alexander } from '../../src/invariants/alexander';
import { jones } from '../../src/invariants/jones';
import { bestView } from '../../src/diagram/bestView';
import { TOP } from '../../src/diagram/project';
import { json, presetScene } from './util';

const time = (f: () => unknown, reps = 3) => { f(); let best = Infinity; for (let i = 0; i < reps; i++) { const t0 = performance.now(); f(); best = Math.min(best, performance.now() - t0); } return best; };
const pdOf = (w: string) => pdCode(braidToDiagram(parseBraid(w)));

test('lift + simplify of ~500 sampled vertices <= 300 ms', () => {
  for (const f of ['wikipedia-complex-unknot.knot.json', 'wikipedia-simple-unknots.knot.json']) {
    const d = parseKnotJSON(json(`../fixtures/${f}`)).diagram!;
    const n = lift(d).reduce((s, P) => s + P.length / 3, 0), ms = time(() => simplify(lift(d)));
    console.log(`${f}: ${n} lifted verts, lift + simplify ${ms.toFixed(0)} ms`);
    expect(ms).toBeLessThanOrEqual(300);
  }
});

test('Alexander, n <= 40: <= 200 ms', () => {
  const { pd, signs } = pdOf(`3: ${'s1 s2^-1 '.repeat(20)}`); // 40 crossings, a knot
  expect(pd.length).toBe(40);
  const ms = time(() => alexander(pd, signs), 1);
  console.log(`Alexander n = 40: ${ms.toFixed(0)} ms`);
  expect(ms).toBeLessThanOrEqual(200);
});

test('Jones n = 16 <= 100 ms, n = 20 <= 1 s', () => {
  for (const [k, budget] of [[8, 100], [10, 1000]] as const) {
    const { pd, signs } = pdOf(`3: ${'s1 s2^-1 '.repeat(k)}`);
    const ms = time(() => jones(pd, signs), 1);
    console.log(`Jones n = ${pd.length}: ${ms.toFixed(0)} ms`);
    expect(ms).toBeLessThanOrEqual(budget);
  }
});

test('best view, 128 directions, n = 100: <= 30 ms', () => {
  const S = presetScene('thistlethwaite'), ms = time(() => bestView(S, TOP));
  console.log(`best view n = 96: ${ms.toFixed(1)} ms`);
  expect(ms).toBeLessThanOrEqual(30);
});
