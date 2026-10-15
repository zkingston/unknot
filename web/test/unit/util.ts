import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { sfc32 } from '../../src/kernel/prng';
import type { Scene } from '../../src/kernel/scene';
import { presetById } from '../../src/presets';
import { lift } from '../../src/diagram/lift';
import { simplify } from '../../src/diagram/simplify';
import type { Diagram } from '../../src/diagram/types';

export const json = (path: string) => JSON.parse(readFileSync(new URL(path, import.meta.url), 'utf8'));
export const fixtureScene = (name: string): Scene => [Float64Array.from((json(`../fixtures/${name}`) as number[][]).flat())];
export const sha = (a: Float64Array | Float64Array[]) => {
  const h = createHash('sha256');
  for (const x of Array.isArray(a) ? a : [a]) h.update(new Uint8Array(x.buffer, x.byteOffset, x.byteLength));
  return (h as unknown as { digest(e: 'hex'): string }).digest('hex');
};

/** Seeded test RNG in [0,1). */
export const rand = (seed: number) => { const r = sfc32(seed); return () => r() / 4294967296; };

/** Random n-gon with vertices in [-1,1]^3 (generic, so almost surely embedded). */
export function randomPolygon(r: () => number, n: number, planar = false): Float64Array {
  const P = new Float64Array(3 * n);
  if (planar) {
    // star-shaped about the origin, so simple; z = 0
    const ang = Array.from({ length: n }, () => r() * 2 * Math.PI).sort((a, b) => a - b);
    ang.forEach((a, i) => { const rr = 0.3 + 0.7 * r(); P[3 * i] = rr * Math.cos(a); P[3 * i + 1] = rr * Math.sin(a); P[3 * i + 2] = 0; });
  } else for (let i = 0; i < 3 * n; i++) P[i] = 2 * r() - 1;
  return P;
}

/** The preset's diagram (if it has one) as the app builds it. */
export function presetDiagram(id: string): Diagram | undefined {
  return presetById(id)!.load().diagram;
}
/** The planner input exactly as the app builds it. */
export function presetScene(id: string): Scene {
  const l = presetById(id)!.load();
  return l.scene ?? simplify(lift(l.diagram!));
}
/** Presets small enough for five seeds with isotopy checks; the large hard unknots are in hard.test.ts. */
export const CORE = ['culprit', 'monster', 'h', 'j', 'goeritz', 'thistlethwaite', 'ochiai', 'trefoil', 'figure-eight'];
