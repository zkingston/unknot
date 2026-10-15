import type { Scene } from './scene';

/** Centre the bounding box and scale so the largest extent spans [-1, 1]. Only + − × ÷. */
export function normalizeScene(S: Scene): Scene {
  const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
  for (const P of S) for (let i = 0; i < P.length; i++) { const a = i % 3; if (P[i] < lo[a]) lo[a] = P[i]; if (P[i] > hi[a]) hi[a] = P[i]; }
  const ext = Math.max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]), s = ext > 0 ? 2 / ext : 1;
  const mid = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2];
  return S.map((P) => P.map((x, i) => (x - mid[i % 3]) * s));
}

/** Legacy `[[x,y,z], ...]` -> one-component Scene. */
export const fromLegacy = (pts: number[][]): Scene => [Float64Array.from(pts.flat())];
