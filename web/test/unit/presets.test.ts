import { describe, expect, test } from 'vitest';
import { readFileSync, writeFileSync } from 'node:fs';
import { runSync } from '../../src/planner/run';
import { canonical, REC } from '../../src/planner/movelog';
import { analyze } from '../../src/invariants';
import { checkIsotopy } from './isotopy';
import { sha, presetScene, presetDiagram, CORE } from './util';

const KNOTTED = new Set(['trefoil', 'figure-eight']);
/** Knotted runs end on the budget, so only a fixed prefix is comparable across machines. */
const PREFIX = 1000;

describe.each(CORE)('preset %s', (id) => {
  const S = presetScene(id);
  test(KNOTTED.has(id) ? 'invariants say knotted; the planner stops at >= 6 sticks without strand passages' : 'reaches n <= 5 for seeds 1-5 without strand passages', () => {
    const d = presetDiagram(id);
    if (d) { const a = analyze(d); expect(a.kind === 'ok' && a.knotted).toBe(KNOTTED.has(id)); }
    for (let seed = 1; seed <= 5; seed++) {
      const r = runSync(S, KNOTTED.has(id) ? { restart: 0 } : {}, seed, KNOTTED.has(id) ? 300 : 20_000);
      expect(r.solved).toBe(!KNOTTED.has(id));
      if (KNOTTED.has(id)) expect(r.S.reduce((s, P) => s + P.length / 3, 0)).toBeGreaterThanOrEqual(6);
      const iso = checkIsotopy(S, r.log.records);
      expect(iso.passages, iso.bad.join('\n')).toBe(0);
    }
  });
});

const GOLD = new URL('../fixtures/determinism.json', import.meta.url);
test('seed-7 goldens (Scene and MoveLog SHA-256) for the cross-browser check', () => {
  const out: Record<string, { scene: string; log: string; records: number }> = {};
  for (const id of CORE) {
    // as the app does: no restarts for a knot the invariants prove knotted
    const S = presetScene(id), r = runSync(S, KNOTTED.has(id) ? { restart: 0 } : {}, 7, KNOTTED.has(id) ? 2000 : 20_000);
    let c = canonical(r.log.records);
    if (KNOTTED.has(id)) { expect(c.length / REC).toBeGreaterThanOrEqual(PREFIX); c = c.subarray(0, PREFIX * REC); }
    out[id] = { scene: sha(S), log: sha(c), records: c.length / REC };
  }
  if (process.env.GEN_GOLDENS) writeFileSync(GOLD, JSON.stringify(out, null, 1) + '\n');
  expect(out).toEqual(JSON.parse(readFileSync(GOLD, 'utf8')));
});
