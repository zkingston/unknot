import { describe, expect, test } from 'vitest';
import { parsePD, pdSigns, pdComponents } from '../../src/import/pdcode';
import { parseGauss } from '../../src/import/gauss';
import { pdToDiagram } from '../../src/import/embed';
import { validateDiagram } from '../../src/diagram/crossings';
import { pdCode, type PD } from '../../src/invariants/pd';
import { analyze } from '../../src/invariants';
import { alexander } from '../../src/invariants/alexander';
import { jones } from '../../src/invariants/jones';
import { type Laurent, L, eq, format, mirror, isOne } from '../../src/invariants/laurent';
import { parseKnotJSON } from '../../src/import/json';
import { json, rand } from './util';
import { braidToDiagram } from '../../src/import/braid';
import { PRESETS } from '../../src/presets';
import type { Diagram } from '../../src/diagram/types';

const KA = {
  '3_1': 'X[1,5,2,4], X[3,1,4,6], X[5,3,6,2]',
  '4_1': 'X[4,2,5,1], X[8,6,1,5], X[6,3,7,4], X[2,7,3,8]',
  '5_1': 'X[1,6,2,7], X[3,8,4,9], X[5,10,6,1], X[7,2,8,3], X[9,4,10,5]',
  '5_2': 'X[1,4,2,5], X[3,8,4,9], X[5,10,6,1], X[9,6,10,7], X[7,2,8,3]',
  '6_1': 'X[1,4,2,5], X[7,10,8,11], X[3,9,4,8], X[9,3,10,2], X[5,12,6,1], X[11,6,12,7]',
  '6_2': 'X[1,4,2,5], X[5,10,6,11], X[3,9,4,8], X[9,3,10,2], X[7,12,8,1], X[11,6,12,7]',
  '6_3': 'X[4,2,5,1], X[8,4,9,3], X[12,9,1,10], X[10,5,11,6], X[6,11,7,12], X[2,8,3,7]',
};
// symmetric Alexander polynomials, lowest power first
const ALEX: Record<string, Laurent> = {
  '3_1': L(-1, [1, -1, 1]), '4_1': L(-1, [-1, 3, -1]), '5_1': L(-2, [1, -1, 1, -1, 1]), '5_2': L(-1, [2, -3, 2]),
  '6_1': L(-1, [-2, 5, -2]), '6_2': L(-2, [-1, 3, -3, 3, -1]), '6_3': L(-2, [1, -3, 5, -3, 1]),
};
const exactAlex = (d: Diagram) => { const a = analyze(d); if (a.kind !== 'ok') throw new Error(a.kind); const x = a.comps[0].alexander; if (x.kind !== 'exact') throw new Error('mod'); return { alex: x.poly, jones: a.comps[0].jones as Laurent, writhe: a.comps[0].writhe }; };

/** Same code up to a cyclic relabelling of the (single) component. */
function samePD(a: PD, b: PD) {
  const N = 2 * a.length, norm = (p: PD) => { const lo = Math.min(...p.flat()); return p.map((X) => X.map((x) => x - lo)); };
  const A = norm(a).map((X) => X.join(',')).sort().join(' '), B0 = norm(b);
  for (let k = 0; k < N; k++) if (B0.map((X) => X.map((x) => (x + k) % N).join(',')).sort().join(' ') === A) return true;
  return false;
}

describe('PD parsing', () => {
  test('Knot Atlas and KnotInfo formats, PD[...] wrapper', () => {
    const a = parsePD(KA['3_1']), b = parsePD('[[1,5,2,4],[3,1,4,6],[5,3,6,2]]'), c = parsePD('PD[X[1, 5, 2, 4],\n X[3, 1, 4, 6], X[5, 3, 6, 2]]');
    expect(b).toEqual(a); expect(c).toEqual(a);
    expect(pdSigns(a)).toEqual([1, 1, 1]);
    expect(pdComponents(a).length).toBe(1);
  });
  test('rejects malformed and non-planar codes with clear messages', () => {
    expect(() => parsePD('')).toThrow(/no crossings/);
    expect(() => parsePD('X[1,5,2,4], X[3,1,4,6]')).toThrow(/appears/);
    expect(() => parsePD('X[1,2,3,4] foo')).toThrow(/unrecognised/);
    expect(() => parsePD('X[1,5,2,4], X[3,1,4,6], X[5,3,2,6]')).toThrow();      // swapped slots: not a planar/consistent code
    expect(() => parsePD('X[1,4,2,5], X[3,1,4,6], X[5,3,6,2]')).toThrow(/not planar/);  // one crossing mirrored
  });
});

describe('PD layout', () => {
  for (const [name, code] of Object.entries(KA)) test(`${name}: regular diagram, own PD equals the input, Alexander matches`, () => {
    const pd = parsePD(code), d = pdToDiagram(pd, { sourceText: code });
    expect(validateDiagram(d).ok).toBe(true);
    expect(d.crossings.length).toBe(pd.length);
    expect(samePD(pdCode(d).pd, pd)).toBe(true);
    const inv = exactAlex(d);
    expect(format(inv.alex), name).toBe(format(ALEX[name]));
    // the drawn diagram has the same invariants as the code itself
    const s = pdSigns(pd), a = alexander(pd, s);
    expect(a.kind === 'exact' && eq(a.poly, inv.alex)).toBe(true);
    expect(eq(jones(pd, s)!, inv.jones)).toBe(true);
  });
  test('invariant table: trefoil (writhe +3: t + t³ − t⁴) and figure-eight', () => {
    const t = exactAlex(pdToDiagram(parsePD(KA['3_1'])));
    expect(t.writhe).toBe(3);
    expect(eq(t.jones, L(1, [1, 0, 1, -1]))).toBe(true);
    expect(format(t.alex)).toBe('t − 1 + t⁻¹');
    const f = exactAlex(pdToDiagram(parsePD(KA['4_1'])));
    expect([f.writhe, format(f.alex), format(f.jones)]).toEqual([0, '−t + 3 − t⁻¹', 't² − t + 1 − t⁻¹ + t⁻²']);
  });
  test('round trip: Wikipedia complex diagram -> PD -> layout -> Diagram -> PD', () => {
    const d0 = parseKnotJSON(json('../fixtures/wikipedia-complex-unknot.knot.json')).diagram!;
    const pd0 = pdCode(d0).pd, d1 = pdToDiagram(pd0), pd1 = pdCode(d1).pd;
    expect(samePD(pd1, pd0)).toBe(true);
    const d2 = pdToDiagram(pd1);
    expect(samePD(pdCode(d2).pd, pd0)).toBe(true);
  });
  test('two-component link and a split link lay out', () => {
    const hopf = parsePD('X[4,1,3,2], X[2,3,1,4]');                    // Knot Atlas L2a1
    expect(pdComponents(hopf).length).toBe(2);
    const d = pdToDiagram(hopf);
    expect(d.comps.length).toBe(2);
    expect(validateDiagram(d).ok).toBe(true);
    const fig8 = parsePD(KA['4_1']).map((X) => X.map((x) => x + 6)) as PD;
    const split = [...parsePD(KA['3_1']), ...fig8] as PD;
    const ds = pdToDiagram(split);
    expect(ds.comps.length).toBe(2);
    expect(validateDiagram(ds).ok).toBe(true);
    const a = analyze(ds);
    expect(a.kind === 'ok' && a.comps.map((c) => format((c.alexander as { poly: Laurent }).poly))).toEqual(['t − 1 + t⁻¹', '−t + 3 − t⁻¹']);
  });
});

/** T(2, n), n odd: X[a, a+n, a+1, a+n+1] (mod 2n) for a = 1, 3, ..., 2n−1. */
const torus2 = (n: number): PD => Array.from({ length: n }, (_, k) => { const a = 2 * k + 1, w = (x: number) => ((x - 1) % (2 * n)) + 1; return [a, w(a + n), w(a + 1), w(a + n + 1)]; });

describe('layout budgets', () => {
  test('n = 29 within 300 ms; n = 151 within 3 s', () => {
    for (const [n, ms] of [[29, 300], [151, 3000]] as const) {
      const pd = torus2(n), t0 = performance.now(), d = pdToDiagram(pd), dt = performance.now() - t0;
      console.log(`layout T(2,${n}): ${dt.toFixed(0)} ms`);
      expect(validateDiagram(d).ok).toBe(true);
      expect(d.crossings.length).toBe(n);
      expect(dt).toBeLessThan(ms);
    }
  }, 60_000);
});

describe('Gauss codes', () => {
  test('signed trefoil and figure-eight reproduce the table exactly', () => {
    const t = parseGauss('1, -2, 3, -1, 2, -3; +,+,+');
    expect(t.chiralityUndetermined).toBe(false);
    const ti = exactAlex(pdToDiagram(t.pd, { source: 'gauss' }));
    expect([ti.writhe, format(ti.alex)]).toEqual([3, 't − 1 + t⁻¹']);
    expect(eq(ti.jones, L(1, [1, 0, 1, -1]))).toBe(true);
    const m = exactAlex(pdToDiagram(parseGauss('1 −2 3 −1 2 −3 ; − − −').pd));
    expect(m.writhe).toBe(-3);
    expect(eq(m.jones, L(-4, [-1, 1, 0, 1]))).toBe(true);
  });
  test('unsigned trefoil and figure-eight find a planar rotation choice; invariants up to t ↔ t⁻¹', () => {
    for (const [code, alex, jon] of [
      ['1, -2, 3, -1, 2, -3', 't − 1 + t⁻¹', L(1, [1, 0, 1, -1])],
      ['-1, 2, -3, 1, -4, 3, -2, 4', '−t + 3 − t⁻¹', L(-2, [1, -1, 1, -1, 1])],
    ] as const) {
      const g = parseGauss(code);
      expect(g.chiralityUndetermined).toBe(true);
      const inv = exactAlex(pdToDiagram(g.pd, { source: 'gauss' }));
      expect(format(inv.alex)).toBe(alex);
      expect(eq(inv.jones, jon) || eq(inv.jones, mirror(jon))).toBe(true);
    }
  });
  test('errors', () => {
    expect(() => parseGauss('1, -2, 3')).toThrow(/once over/);
    expect(() => parseGauss('1, -2, 3, -1, 2, -3; +,+')).toThrow(/3 crossing signs/);
    expect(() => parseGauss('1, 2, -1, -2')).toThrow(/planar/);
  });
});

test('Alexander beyond 41 crossings: the three-point mod-p test agrees with the exact polynomial', () => {
  const r = rand(41);
  let knots = 0, trivial = 0;
  for (let k = 0; knots < 40 && k < 2000; k++) {
    const strands = 3 + (k % 2), len = 41 + Math.floor(r() * 24), word = Array.from({ length: len }, () => (1 + Math.floor(r() * (strands - 1))) * (r() < 0.5 ? 1 : -1));
    // one component: the braid permutation is a single cycle
    const perm = Array.from({ length: strands }, (_, i) => i);
    for (const g of word) { const a = Math.abs(g) - 1; [perm[a], perm[a + 1]] = [perm[a + 1], perm[a]]; }
    let len1 = 1; for (let i = perm[0]; i !== 0; i = perm[i]) len1++;
    if (len1 !== strands) continue;
    const { pd, signs } = pdCode(braidToDiagram({ word, strands }), [0], true);
    if (pd.length <= 41) continue; // the minor is (n − 1) × (n − 1): exact up to 41 crossings
    const exact = alexander(pd, signs, undefined, Infinity), fast = alexander(pd, signs);
    expect(exact.kind === 'exact' && fast.kind === 'mod').toBe(true);
    if (exact.kind !== 'exact' || fast.kind !== 'mod') continue;
    expect(fast.isOne, `braid ${word.join(' ')}`).toBe(isOne(exact.poly));
    knots++; if (fast.isOne) trivial++;
  }
  expect(knots).toBe(40);
  // every hard unknot over 41 crossings (all have Δ = 1)
  let big = 0;
  for (const p of PRESETS) {
    const d = p.load().diagram;
    if (!d || d.crossings.length <= 41) continue;
    const { pd, signs } = pdCode(d, [0], true), a = alexander(pd, signs);
    expect(a.kind === 'mod' && a.isOne, p.id).toBe(true);
    big++;
  }
  expect(big).toBeGreaterThan(8);
  console.log(`${knots} braid knots (${trivial} with Δ = 1), ${big} unknots`);
}, 120_000);
