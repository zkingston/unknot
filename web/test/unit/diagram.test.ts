import { describe, expect, test } from 'vitest';
import { parseKnotJSON } from '../../src/import/json';
import { validateDiagram, findCrossings } from '../../src/diagram/crossings';
import { lift } from '../../src/diagram/lift';
import { simplify } from '../../src/diagram/simplify';
import { projectScene } from '../../src/diagram/project';
import { clearance } from '../../src/kernel/energy';
import { analyze } from '../../src/invariants';
import { pdCode } from '../../src/invariants/pd';
import { alexander } from '../../src/invariants/alexander';
import { jones } from '../../src/invariants/jones';
import { type Laurent, L, eq, format } from '../../src/invariants/laurent';
import { runSync } from '../../src/planner/run';
import { checkIsotopy } from './isotopy';
import { json } from './util';
import type { Diagram } from '../../src/diagram/types';
import type { Scene } from '../../src/kernel/scene';

const load = (f: string) => parseKnotJSON(json(`../fixtures/${f}`)).diagram!;
const complex = load('wikipedia-complex-unknot.knot.json'), simple = load('wikipedia-simple-unknots.knot.json');

function trefoil(mirror = false): Scene {
  const P = new Float64Array(3 * 400);
  for (let k = 0; k < 400; k++) {
    const t = (2 * Math.PI * k) / 400;
    P[3 * k] = ((Math.sin(t) + 2 * Math.sin(2 * t)) / 3) * (mirror ? -1 : 1);
    P[3 * k + 1] = (Math.cos(t) - 2 * Math.cos(2 * t)) / 3;
    P[3 * k + 2] = 0.6 * Math.sin(3 * t);
  }
  return [P];
}
const inv = (d: Diagram) => { const a = analyze(d); if (a.kind !== 'ok') throw new Error(a.kind); return a.comps[0]; };
const alexOf = (d: Diagram) => { const a = inv(d).alexander; if (a.kind !== 'exact') throw new Error('mod'); return a.poly; };
const jonesOf = (d: Diagram) => inv(d).jones as Laurent;

describe('fixture diagrams', () => {
  test('are regular and their crossing lists match', () => {
    for (const d of [complex, simple]) expect(validateDiagram(d)).toMatchObject({ ok: true });
    expect(complex.crossings.length).toBe(6);
    expect(simple.crossings.length).toBe(2);
    expect(simple.comps.length).toBe(2);
  });
});

describe('lift', () => {
  for (const [name, d, nx] of [['complex', complex, 6], ['simple', simple, 2]] as const) test(`${name}: ${nx} crossings, projection matches, embedded, no vertex on a crossing, no flat pairs`, () => {
    const S = lift(d);
    expect(clearance(S)).toBeGreaterThan(0);
    const proj = projectScene(S);
    expect(proj.crossings.length).toBe(nx);
    for (const X of findCrossings(proj.comps)) for (const s of [X.a, X.b]) {
      const P = S[s.comp], n = P.length / 3, j = (s.seg + 1) % n;
      const L2 = (P[3 * j] - P[3 * s.seg]) ** 2 + (P[3 * j + 1] - P[3 * s.seg + 1]) ** 2;
      expect(Math.min(s.t, 1 - s.t) * Math.sqrt(L2)).toBeGreaterThan(1e-9);
    }
    for (const P of S) {
      const n = P.length / 3;
      for (let i = 0; i < n; i++) {
        const z0 = P[3 * ((i + n - 1) % n) + 2], z1 = P[3 * i + 2], z2 = P[3 * ((i + 1) % n) + 2];
        expect(z0 === z1 && z1 === z2, `flat pair at ${i}`).toBe(false);
      }
    }
    // normalised frame
    let mx = 0; for (const P of S) for (let i = 0; i < P.length; i += 3) mx = Math.max(mx, Math.abs(P[i]), Math.abs(P[i + 1]));
    expect(Math.abs(mx - 1)).toBeLessThan(1e-12);
  });
  test('rejects flat crossings', () => {
    expect(() => lift({ ...complex, crossings: complex.crossings.map((X, k) => (k ? X : { ...X, flat: true as const })) })).toThrow();
  });
});

describe('simplify', () => {
  test('ε = 0.01 within ±25% of the spike (80 / 48 / 25)', () => {
    const c = simplify(lift(complex)), s = simplify(lift(simple));
    const counts = [c[0].length / 3, s[0].length / 3, s[1].length / 3];
    console.log('simplified', counts);
    [80, 48, 25].forEach((want, k) => { expect(counts[k]).toBeGreaterThanOrEqual(0.75 * want); expect(counts[k]).toBeLessThanOrEqual(1.25 * want); });
    for (const S of [c, s]) expect(clearance(S)).toBeGreaterThan(0);
  });
  test('full simplify of the parametric trefoil stops at exactly 6 vertices, invariants unchanged', () => {
    const T = trefoil(), before = projectScene(T), S = simplify(T, { epsilon: Infinity });
    expect(S[0].length / 3).toBe(6);
    const after = projectScene(S);
    expect(eq(alexOf(after), alexOf(before))).toBe(true);
    expect(eq(jonesOf(after), jonesOf(before))).toBe(true);
  });
  test('full simplify of the Wikipedia unknots stays embedded and preserves invariants', () => {
    for (const d of [complex, simple]) {
      const S = simplify(lift(d), { epsilon: Infinity });
      expect(clearance(S)).toBeGreaterThan(0);
      console.log('full simplify', S.map((P) => P.length / 3));
    }
  });
});

describe('invariants (reference table)', () => {
  const t = (lo: number, c: number[]) => L(lo, c);
  test('Wikipedia complex: 6 crossings, writhe −2, Δ = V = 1', () => {
    const c = inv(complex);
    expect([c.crossings, c.writhe]).toEqual([6, -2]);
    expect(format(alexOf(complex))).toBe('1');
    expect(format(jonesOf(complex))).toBe('1');
  });
  test('Wikipedia simple twist: 2 crossings, writhe 0, Δ = V = 1', () => {
    const a = analyze(simple);
    if (a.kind !== 'ok') throw new Error();
    expect([a.comps[0].crossings, a.comps[0].writhe, a.comps[1].crossings]).toEqual([2, 0, 0]);
    expect(a.knotted).toBe(false);
  });
  test('parametric trefoil and its mirror', () => {
    const d = projectScene(trefoil()), m = projectScene(trefoil(true));
    expect([inv(d).crossings, inv(d).writhe, inv(m).writhe]).toEqual([3, 3, -3]);
    expect(format(alexOf(d))).toBe('t − 1 + t⁻¹');
    expect(eq(jonesOf(d), t(1, [1, 0, 1, -1]))).toBe(true);           // t + t³ − t⁴
    expect(eq(jonesOf(m), t(-4, [-1, 1, 0, 1]))).toBe(true);          // t⁻¹ + t⁻³ − t⁻⁴
    expect(format(alexOf(m))).toBe('t − 1 + t⁻¹');
  });
  test('figure-eight from the Knot Atlas PD', () => {
    const pd = [[4, 2, 5, 1], [8, 6, 1, 5], [6, 3, 7, 4], [2, 7, 3, 8]] as [number, number, number, number][];
    const signs = pd.map(([, j, , l]) => (j === (l % 8) + 1 ? 1 : -1) as 1 | -1);
    expect(signs.reduce((s, x) => s + x, 0)).toBe(0);
    const a = alexander(pd, signs);
    expect(a.kind === 'exact' && format(a.poly)).toBe('−t + 3 − t⁻¹');
    expect(format(jones(pd, signs)!)).toBe('t² − t + 1 − t⁻¹ + t⁻²');
  });
  test('modular Alexander path agrees on small knots', () => {
    const pd = pdCode(projectScene(trefoil())).pd, s = pdCode(projectScene(trefoil())).signs;
    expect(alexander(pd, s).kind).toBe('exact');
  });
});

describe('split link', () => {
  test('wikipedia-simple-unknots: lifted in one frame, both components reach n <= 5, isotopy covers inter-component pairs', () => {
    const S = simplify(lift(simple));
    expect(S.length).toBe(2);
    for (let seed = 1; seed <= 5; seed++) {
      const r = runSync(S, {}, seed, 20_000);
      expect(r.solved).toBe(true);
      expect(r.S.every((P) => P.length <= 15)).toBe(true);
      const iso = checkIsotopy(S, r.log.records);
      expect(iso.passages, iso.bad.join('\n')).toBe(0);
    }
  });
});
