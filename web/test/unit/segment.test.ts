import { expect, test } from 'vitest';
import { tsKernel } from '../../src/kernel/kernel';
import { ptSeg, lerp, type P3 } from './isotopy';
import { rand } from './util';


/** 4096-sample brute force, refined by golden-section on the convex distance-to-segment function. */
function brute(a0: P3, a1: P3, b0: P3, b1: P3) {
  const f = (s: number) => ptSeg(lerp(a0, a1, s), b0, b1);
  let best = 0;
  for (let k = 0; k <= 4096; k++) if (f(k / 4096) < f(best / 4096)) best = k;
  let lo = Math.max(0, (best - 1) / 4096), hi = Math.min(1, (best + 1) / 4096);
  for (let it = 0; it < 100; it++) { const m1 = lo + (hi - lo) * 0.382, m2 = lo + (hi - lo) * 0.618; if (f(m1) < f(m2)) hi = m2; else lo = m1; }
  return Math.min(f(lo), f(best / 4096), f(0), f(1));
}

test('segDist matches brute force on 10k pairs incl. parallel, collinear, zero-length', () => {
  const r = rand(11), p = (): P3 => [2 * r() - 1, 2 * r() - 1, 2 * r() - 1];
  for (let i = 0; i < 10000; i++) {
    let a0 = p(), a1 = p(), b0 = p(), b1 = p();
    const kind = i % 6;
    if (kind === 1) { const d: P3 = [a1[0] - a0[0], a1[1] - a0[1], a1[2] - a0[2]], s = 2 * r() - 1; b1 = [b0[0] + s * d[0], b0[1] + s * d[1], b0[2] + s * d[2]]; } // parallel
    if (kind === 2) { b0 = lerp(a0, a1, 3 * r() - 1); b1 = lerp(a0, a1, 3 * r() - 1); }                 // collinear
    if (kind === 3) a1 = [...a0];                                                                        // zero-length
    if (kind === 4) { a1 = [...a0]; b1 = [...b0]; }                                                      // both points
    if (kind === 5) { const e = 1e-9 * r(); b1 = [b0[0] + (a1[0] - a0[0]) + e, b0[1] + (a1[1] - a0[1]), b0[2] + (a1[2] - a0[2]) - e]; } // nearly parallel
    const { d, s, t } = tsKernel.segDist(a0, a1, b0, b1);
    const bf = brute(a0, a1, b0, b1);
    expect(Math.abs(d - bf), `pair ${i} kind ${kind}`).toBeLessThan(1e-9);
    expect(s >= 0 && s <= 1 && t >= 0 && t <= 1).toBe(true);
    const pa = lerp(a0, a1, s), pb = lerp(b0, b1, t);
    expect(Math.abs(Math.hypot(pa[0] - pb[0], pa[1] - pb[1], pa[2] - pb[2]) - d)).toBeLessThan(1e-12);
  }
});
