import { describe, expect, test } from 'vitest';
import { runSync } from '../../src/planner/run';
import { canonical } from '../../src/planner/movelog';
import { clearance } from '../../src/kernel/energy';
import { checkIsotopy } from './isotopy';
import { fixtureScene, presetScene, sha } from './util';
import type { Scene } from '../../src/kernel/scene';

const presets: [string, () => Scene][] = [
  ['goeritz', () => presetScene('goeritz')],
  ['ochiai', () => presetScene('ochiai')],
  ['thistlethwaite', () => presetScene('thistlethwaite')],
  ['complex-c0', () => fixtureScene('complex-c0.json')],
  ['simple-c0', () => fixtureScene('simple-c0.json')],
  ['simple-c1', () => fixtureScene('simple-c1.json')],
];

describe.each(presets)('%s', (name, load) => {
  const S = load();
  test('input is embedded', () => expect(clearance(S)).toBeGreaterThan(0));
  // presets.test.ts already runs the repo presets with the defaults
  const kinds = ['goeritz', 'ochiai', 'thistlethwaite'].includes(name) ? ['hybrid and restarts off'] as const : ['default', 'hybrid and restarts off'] as const;
  for (const kind of kinds) {
    const params = kind === 'default' ? {} : { hybrid: 0, restart: 0 };
    test(`${kind}: seeds 1-5 reach n <= 5 with no strand passage`, () => {
      for (let seed = 1; seed <= 5; seed++) {
        const r = runSync(S, params, seed, 20_000);
        expect(r.solved, `${name} ${kind} seed ${seed}`).toBe(true);
        const iso = checkIsotopy(S, r.log.records);
        expect(iso.passages, iso.bad.join('\n')).toBe(0);
      }
    });
  }
  test('determinism: seed 7 twice gives the same MoveLog; a budget-cut run is a byte prefix', () => {
    {
      const a = runSync(S, {}, 7, 20_000), b = runSync(S, {}, 7, 20_000);
      expect(sha(canonical(a.log.records))).toBe(sha(canonical(b.log.records)));
      // restarts discard abandoned attempts, so the prefix property holds within an attempt
      const r1 = runSync(S, {}, 7, 1), r2 = runSync(S, {}, 7, 2);
      const c1 = canonical(r1.log.records), c2 = canonical(r2.log.records), full = canonical(a.log.records);
      expect(r1.attempts).toBeLessThanOrEqual(r2.attempts);
      expect(r2.attempts).toBeLessThanOrEqual(a.attempts);
      if (r1.attempts === r2.attempts) { expect(c1.length).toBeLessThanOrEqual(c2.length); expect(Array.from(c2.subarray(0, c1.length))).toEqual(Array.from(c1)); }
      if (r2.attempts === a.attempts) expect(Array.from(full.subarray(0, c2.length))).toEqual(Array.from(c2));
      // and without restarts, the strict version
      const n1 = canonical(runSync(S, { restart: 0 }, 7, 1).log.records), nf = canonical(runSync(S, { restart: 0 }, 7, 20_000).log.records);
      expect(Array.from(nf.subarray(0, n1.length))).toEqual(Array.from(n1));
    }
  });
});

const median = (x: number[]) => [...x].sort((a, b) => a - b)[Math.floor(x.length / 2)];
const p95 = (x: number[]) => [...x].sort((a, b) => a - b)[Math.ceil(0.95 * x.length) - 1];

test('budgets: heuristic on repo presets, seeds 1-20: median <= 1 s, p95 <= 3 s; Wikipedia complex median <= 300 ms', () => {
  for (const [name, load] of presets.slice(0, 4)) {
    const S = load(), ms: number[] = [];
    for (let seed = 1; seed <= 20; seed++) { const r = runSync(S, {}, seed, 20_000); expect(r.solved).toBe(true); ms.push(r.ms); }
    console.log(`${name}: median ${median(ms).toFixed(1)} ms, p95 ${p95(ms).toFixed(1)} ms`);
    if (name === 'complex-c0') expect(median(ms)).toBeLessThanOrEqual(300);
    else { expect(median(ms)).toBeLessThanOrEqual(1000); expect(p95(ms)).toBeLessThanOrEqual(3000); }
  }
}, 600_000);

test('proven knots stop once simplification stalls: the same run cut short, at the stick number', () => {
  for (const [id, sticks] of [['trefoil', 6], ['figure-eight', 7]] as const) {
    const S = presetScene(id), r = runSync(S, { restart: 0, stall: 50 }, 7, 30_000), full = runSync(S, { restart: 0 }, 7, 2_000);
    expect(r.stalled, id).toBe(true);
    expect(r.solved, id).toBe(false);
    expect(r.S[0].length / 3, id).toBe(sticks);
    const a = canonical(r.log.records), b = canonical(full.log.records).subarray(0, a.length);
    expect(a.length).toBeLessThanOrEqual(canonical(full.log.records).length);
    expect(sha(a), `${id}: stall changes nothing before the stop`).toBe(sha(b));
  }
}, 60_000);
