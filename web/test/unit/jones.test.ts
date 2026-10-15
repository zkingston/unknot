import { expect, test } from 'vitest';
import { bracket } from '../../src/invariants/jones';
import { pdCode, type PD } from '../../src/invariants/pd';
import { projectScene } from '../../src/diagram/project';
import { type Laurent, L, add, eq, format, pow, scale, shift } from '../../src/invariants/laurent';
import { clearance } from '../../src/kernel/energy';
import { rand, randomPolygon } from './util';

/** Reference: the bracket as a plain sum over all 2^n states. */
function bruteBracket(pd: PD): Laurent {
  const n = pd.length, labels = [...new Set(pd.flat())], id = new Map(labels.map((x, i) => [x, i])), m = labels.length;
  const par = new Int32Array(m), find = (i: number): number => (par[i] === i ? i : (par[i] = find(par[i])));
  let br: Laurent = { lo: 0, c: [] };
  for (let s = 0; s < 1 << n; s++) {
    for (let i = 0; i < m; i++) par[i] = i;
    let nA = 0;
    pd.forEach((q, r) => { const [a, b, c, d] = q.map((x) => id.get(x)!); if ((s >> r) & 1) { par[find(a)] = find(d); par[find(b)] = find(c); } else { nA++; par[find(a)] = find(b); par[find(c)] = find(d); } });
    let loops = 0; for (let i = 0; i < m; i++) if (find(i) === i) loops++;
    br = add(br, shift(pow(L(-2, [-1, 0, 0, 0, -1]), loops - 1), 2 * nA - n));
  }
  return br;
}

test('frontier bracket equals the 2^n state sum on random diagrams', () => {
  const r = rand(11);
  let tried = 0;
  for (let k = 0; k < 400 && tried < 60; k++) {
    const S = [randomPolygon(r, 6 + Math.floor(r() * 6))];
    if (!(clearance(S) > 1e-4)) continue;
    const d = projectScene(S);
    if (d.crossings.some((X) => X.flat)) continue;
    const { pd } = pdCode(d);
    if (pd.length < 2 || pd.length > 13) continue;
    tried++;
    const f = bracket(pd)!;
    expect(eq(f, bruteBracket(pd)), `${JSON.stringify(pd)}: ${format(f, 'A')}`).toBe(true);
  }
  expect(tried).toBeGreaterThan(40);
  expect(eq(scale(L(0, [1]), 1n), bracket([])!)).toBe(true);
});
