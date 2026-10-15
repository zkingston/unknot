import { expect, test } from 'vitest';
import { energy, energyDelta, clearance, touches } from '../../src/kernel/energy';
import { segSeg, pt } from './isotopy';
import { rand, randomPolygon } from './util';
import type { Scene } from '../../src/kernel/scene';

function bruteEnergy(S: Scene) {
  const edges: { c: number; i: number; n: number }[] = [];
  S.forEach((P, c) => { for (let i = 0; i < P.length / 3; i++) edges.push({ c, i, n: P.length / 3 }); });
  let e = 0, m = Infinity;
  for (let x = 0; x < edges.length; x++) for (let y = x + 1; y < edges.length; y++) {
    const A = edges[x], B = edges[y];
    if (A.c === B.c && (B.i === (A.i + 1) % A.n || A.i === (B.i + 1) % A.n)) continue;
    const d = segSeg(pt(S[A.c], A.i), pt(S[A.c], (A.i + 1) % A.n), pt(S[B.c], B.i), pt(S[B.c], (B.i + 1) % B.n));
    e += 1 / (d * d); m = Math.min(m, d);
  }
  return { e, m };
}

test('energy and clearance equal brute force over all non-adjacent pairs, inter-component included', () => {
  const r = rand(5);
  for (let k = 0; k < 40; k++) {
    const S = Array.from({ length: 1 + (k % 3) }, () => randomPolygon(r, 4 + Math.floor(r() * 12)));
    const { e, m } = bruteEnergy(S);
    expect(Math.abs(energy(S) - e) / e).toBeLessThan(1e-9);
    expect(Math.abs(clearance(S) - m)).toBeLessThan(1e-12);
    // energyDelta agrees with the difference of totals
    const c = k % S.length, v = Math.floor(r() * (S[c].length / 3)), to: [number, number, number] = [2 * r() - 1, 2 * r() - 1, 2 * r() - 1];
    const d = energyDelta(S, c, v, to[0], to[1], to[2]);
    const S2 = S.map((P) => P.slice()); S2[c].set(to, 3 * v);
    const e2 = bruteEnergy(S2).e;
    expect(Math.abs(d - (e2 - e)) / Math.max(1, Math.abs(e2) + Math.abs(e))).toBeLessThan(1e-9);
  }
});

test('touches(S) is exactly !(clearance(S) > 0): random scenes, planar ones (crossing edges), exact contacts', () => {
  const r = rand(21);
  let touching = 0;
  for (let k = 0; k < 600; k++) {
    const planar = k % 3 === 0, S: Scene = [randomPolygon(r, 4 + Math.floor(r() * 30), planar)];
    if (k % 2) S.push(randomPolygon(r, 3 + Math.floor(r() * 10), planar));
    if (k % 5 === 1) { const P = S[0], n = P.length / 3, i = Math.floor(r() * n), j = (i + 2 + Math.floor(r() * (n - 3))) % n; P.set(P.subarray(3 * j, 3 * j + 3), 3 * i); } // a vertex onto another
    const t = touches(S);
    expect(t, `scene ${k}`).toBe(!(clearance(S) > 0));
    if (t) touching++;
  }
  expect(touching).toBeGreaterThan(100);
});
