// The swappable kernel interface. Planners call only these and the PRNG, so a WASM or WebGPU
// kernel can replace this TypeScript one after profiling.
import type { V3 } from './vec3';
import type { Scene } from './scene';
import { segDist2, ST } from './segment';
import { maxLegalMove, removeLegal } from './delta';
import { energy, energyDelta, clearance } from './energy';

export interface Kernel {
  segDist(a0: V3, a1: V3, b0: V3, b1: V3): { d: number; s: number; t: number }; // closest params s,t in [0,1]
  // Scene calls treat every edge of every component as an obstacle, except the two edges incident to v
  maxLegalMove(S: Scene, c: number, v: number, target: V3, tau: number, soft?: number): number; // a safe t in [0,1], not necessarily the largest
  removeLegal(S: Scene, c: number, v: number, tau: number): boolean;            // can v be deleted by a Delta move?
  energyDelta(S: Scene, c: number, v: number, to: V3): number;                   // O(N) change in energy
  energy(S: Scene): number;                                                       // O(N^2), stats only
  clearance(S: Scene): number;                                                    // O(N^2), validation only
}

export const tsKernel: Kernel = {
  segDist(a0, a1, b0, b1) {
    const d = Math.sqrt(segDist2(a0[0], a0[1], a0[2], a1[0], a1[1], a1[2], b0[0], b0[1], b0[2], b1[0], b1[1], b1[2]));
    return { d, s: ST[0], t: ST[1] };
  },
  maxLegalMove: (S, c, v, p, tau, soft) => maxLegalMove(S, c, v, p[0], p[1], p[2], tau, soft),
  removeLegal,
  energyDelta: (S, c, v, p) => energyDelta(S, c, v, p[0], p[1], p[2]),
  energy,
  clearance,
};
