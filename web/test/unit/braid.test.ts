import { expect, test } from 'vitest';
import { lift } from '../../src/diagram/lift';
import { parseBraid, braidToDiagram } from '../../src/import/braid';
import { analyze } from '../../src/invariants';
import { format } from '../../src/invariants/laurent';

const inv = (s: string) => {
  const a = analyze(braidToDiagram(parseBraid(s)));
  if (a.kind !== 'ok') throw new Error(a.kind);
  const c = a.comps[0];
  return { writhe: c.writhe, alex: c.alexander.kind === 'exact' ? format(c.alexander.poly) : '?', jones: format(c.jones as never), comps: a.comps.length };
};

test('parses every syntax to the same word', () => {
  const want = { word: [1, -2, 1], strands: 3 };
  for (const s of ['s1 s2^-1 s1', 'σ1 σ2⁻¹ σ1', '{1,-2,1}', '1 -2 1', 'aBa', 's_1 s_2^{-1} s_1', 's1 s2^−1 s1']) expect(parseBraid(s), s).toEqual(want);
  expect(parseBraid('s1^3')).toEqual({ word: [1, 1, 1], strands: 2 });
  expect(parseBraid('4: s1 s2')).toEqual({ word: [1, 2], strands: 4 });
  expect(() => parseBraid('s1 x2')).toThrow();
  expect(() => parseBraid('2: s2')).toThrow();
});

test('convention: closure of s1 s1 s1 is the trefoil with writhe +3, Jones t + t³ − t⁴', () => {
  expect(inv('s1 s1 s1')).toMatchObject({ writhe: 3, alex: 't − 1 + t⁻¹', jones: '−t⁴ + t³ + t' });
});
test('s1 s2^-1 s1 s2^-1 is the figure-eight', () => {
  expect(inv('s1 s2^-1 s1 s2^-1')).toMatchObject({ alex: '−t + 3 − t⁻¹', jones: 't² − t + 1 − t⁻¹ + t⁻²' });
});
test('s1 s2 is the unknot', () => {
  expect(inv('s1 s2')).toMatchObject({ alex: '1', jones: '1', comps: 1 });
});
test('components come from the permutation cycles', () => {
  expect(braidToDiagram(parseBraid('s1 s1')).comps.length).toBe(2);   // Hopf link
  expect(analyze(braidToDiagram(parseBraid('s1 s1'))).kind).toBe('nonsplit');
  expect(braidToDiagram(parseBraid('3: s1 s1 s1')).comps.length).toBe(2); // trefoil + a split circle
});

test('braid diagrams lift (crossings at t = 0.5 must not land on lift vertices)', () => {
  for (const w of ['s1 s1 s1', 's1 s2^-1 s1 s2^-1', 's1 s2', 's1^5', '{1,-2,1,-2,1,-2}']) expect(() => lift(braidToDiagram(parseBraid(w))), w).not.toThrow();
});
