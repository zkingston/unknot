import { expect, test } from 'vitest';
import { deltaTest, maxLegalMove, LEGAL, PIERCE, TAU } from '../../src/kernel/delta';
import { segSeg, segTriDist, moveGap, pt, lerp, type P3 } from './isotopy';
import { rand, randomPolygon } from './util';
import type { Scene } from '../../src/kernel/scene';


/** Other edges of the scene relative to moving vertex v of comp c, with adjacency flags. */
function others(S: Scene, c: number, v: number) {
  const n = S[c].length / 3, ia = (v + n - 1) % n, ic = (v + 1) % n, out: { p: P3; q: P3; endA: boolean; startC: boolean }[] = [];
  S.forEach((Q, k) => { const m = Q.length / 3; for (let i = 0; i < m; i++) {
    if (k === c && (i === ia || i === v)) continue;
    const j = (i + 1) % m;
    out.push({ p: pt(Q, i), q: pt(Q, j), endA: k === c && j === ia, startC: k === c && i === ic });
  } });
  return { out, a: pt(S[c], ia), b: pt(S[c], v), cc: pt(S[c], ic) };
}
/** Smallest distance from the moved edges, with v at x, to their non-adjacent edges. */
function movedClear(S: Scene, c: number, v: number, x: P3) {
  const { out, a, cc } = others(S, c, v);
  let m = Infinity;
  for (const e of out) { if (!e.endA) m = Math.min(m, segSeg(a, x, e.p, e.q)); if (!e.startC) m = Math.min(m, segSeg(x, cc, e.p, e.q)); }
  return m;
}

function cases(seed: number, count: number, planar: boolean) {
  const r = rand(seed), out: { S: Scene; c: number; v: number; to: P3; tau: number }[] = [];
  for (let k = 0; k < count; k++) {
    const n = 8 + Math.floor(r() * 23), P = randomPolygon(r, n, planar), v = Math.floor(r() * n), b = pt(P, v);
    const far = r() < 0.5, s = far ? 1 : 0.15;
    const to: P3 = [b[0] + s * (2 * r() - 1), b[1] + s * (2 * r() - 1), planar ? 0 : b[2] + s * (2 * r() - 1)];
    out.push({ S: [P], c: 0, v, to, tau: r() < 0.3 ? 0.05 : 1e-5 });
  }
  return out;
}

for (const planar of [false, true]) test(`Delta test outcomes are justified (${planar ? 'coplanar z = 0' : '3D'})`, () => {
  const counts = [0, 0, 0];
  for (const { S, c, v, to, tau } of cases(planar ? 22 : 21, planar ? 3000 : 10000, planar)) {
    const res = deltaTest(S, c, v, to[0], to[1], to[2], tau);
    counts[res]++;
    const { out, a, b, cc } = others(S, c, v);
    if (res === LEGAL) {
      expect(movedClear(S, c, v, to)).toBeGreaterThanOrEqual(tau);
      for (let k = 1; k <= 64; k++) expect(movedClear(S, c, v, lerp(b, to, k / 64))).toBeGreaterThan(0);
    } else if (res === PIERCE) {
      let m = Infinity;
      for (const e of out) {
        const q1 = e.endA ? lerp(e.q, e.p, 1e-9) : e.q, p2 = e.startC ? lerp(e.p, e.q, 1e-9) : e.p;
        m = Math.min(m, segTriDist(e.p, q1, a, b, to), segTriDist(p2, e.q, cc, b, to));
      }
      expect(m).toBeLessThanOrEqual(1e-9);
    } else {
      expect(res).toBe(TAU);
      expect(movedClear(S, c, v, to)).toBeLessThan(tau);
    }
  }
  expect(counts.every((x) => x > 0), `outcomes ${counts}`).toBe(true);
});

for (const planar of [false, true]) test(`maxLegalMove is safe and tight at contacts (${planar ? 'coplanar' : '3D'})`, () => {
  let contacts = 0, accepted = 0;
  for (const { S, c, v, to, tau } of cases(planar ? 32 : 31, planar ? 2000 : 10000, planar)) {
    const t = maxLegalMove(S, c, v, to[0], to[1], to[2], tau);
    expect(t >= 0 && t <= 1).toBe(true);
    const b = pt(S[c], v), x = lerp(b, to, t);
    if (t > 0) {
      expect(moveGap(S, c, v, x).gap).toBeGreaterThanOrEqual(1e-9);
      expect(movedClear(S, c, v, x)).toBeGreaterThanOrEqual(tau);
    }
    if (t > 0) accepted++;
    if (planar) continue; // every pair coplanar throughout: decided on the swept triangle, safety checked above
    const { tFirst } = moveGap(S, c, v, to);
    if (tFirst < Infinity) {
      contacts++;
      expect(t === 0 || t < tFirst).toBe(true);
      expect(moveGap(S, c, v, lerp(b, to, Math.min(1, 1.01 * tFirst))).gap).toBeLessThan(1e-9);
    }
  }
  if (!planar) expect(contacts).toBeGreaterThan(100);
  else expect(accepted).toBeGreaterThan(100); // in-plane moves that touch nothing go ahead
});
