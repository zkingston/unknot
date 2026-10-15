import { expect, test } from 'vitest';
import { sfc32, onSphere } from '../../src/kernel/prng';

test('sfc32 seed 1 goldens', () => {
  const r = sfc32(1);
  expect(Array.from({ length: 10 }, r)).toEqual([1828152527, 3394835397, 2967886022, 2251045104, 4148684523, 4055680838, 3224489912, 3690843842, 1114091683, 537224786]);
});

test('onSphere lies on the sphere and is roughly uniform', () => {
  const r = sfc32(3), p = new Float64Array(3), m = [0, 0, 0];
  for (let i = 0; i < 20000; i++) {
    onSphere(r, Math.sqrt(3), p);
    expect(Math.abs(Math.hypot(p[0], p[1], p[2]) - Math.sqrt(3))).toBeLessThan(1e-12);
    for (let k = 0; k < 3; k++) m[k] += p[k] / 20000;
  }
  for (const x of m) expect(Math.abs(x)).toBeLessThan(0.05);
});
