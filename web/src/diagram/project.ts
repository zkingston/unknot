// Orthographic projection of a Scene to a Diagram; the strand nearer the camera is over.
import type { Scene } from '../kernel/scene';
import type { V3 } from '../kernel/vec3';
import { type Diagram } from './types';
import { diagramFromCurves } from './crossings';

/** View basis: right R, up U, toward-viewer F (orthonormal). Default looks down −z. */
export interface View { R: V3; U: V3; F: V3 }
export const TOP: View = { R: [1, 0, 0], U: [0, 1, 0], F: [0, 0, 1] };

export function projectCurves(S: Scene, v: View = TOP): Float64Array[] {
  return S.map((P) => {
    const n = P.length / 3, Q = new Float64Array(2 * n);
    for (let i = 0; i < n; i++) {
      const x = P[3 * i], y = P[3 * i + 1], z = P[3 * i + 2];
      Q[2 * i] = x * v.R[0] + y * v.R[1] + z * v.R[2];
      Q[2 * i + 1] = x * v.U[0] + y * v.U[1] + z * v.U[2];
    }
    return Q;
  });
}

/** Depth (toward viewer) of the scene point at (comp, seg, t). */
export function depthAt(S: Scene, v: View, comp: number, seg: number, t: number) {
  const P = S[comp], n = P.length / 3, j = (seg + 1) % n;
  const d0 = P[3 * seg] * v.F[0] + P[3 * seg + 1] * v.F[1] + P[3 * seg + 2] * v.F[2];
  const d1 = P[3 * j] * v.F[0] + P[3 * j + 1] * v.F[1] + P[3 * j + 2] * v.F[2];
  return d0 + t * (d1 - d0);
}

export function projectScene(S: Scene, v: View = TOP): Diagram {
  const comps = projectCurves(S, v);
  return diagramFromCurves(comps, 'projection', (X) => {
    const da = depthAt(S, v, X.a.comp, X.a.seg, X.a.t), db = depthAt(S, v, X.b.comp, X.b.seg, X.b.t);
    return da === db ? null : da > db;
  });
}
