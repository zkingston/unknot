import { forPairs } from '../kernel/energy';
import { nextIdx, prevIdx } from '../kernel/scene';
import { Op } from './movelog';
import { RADIUS } from './lip';
import type { Ctx } from './run';

const target = new Float64Array(3);
const a0 = new Float64Array(3), a1 = new Float64Array(3), b0 = new Float64Array(3), b1 = new Float64Array(3);
const load = (o: Float64Array, P: Float64Array, i: number) => { o[0] = P[3 * i]; o[1] = P[3 * i + 1]; o[2] = P[3 * i + 2]; };

/**
 * Repulsive forces. Repo terms: segment pairs (each counted as two ordered pairs) and vertex pairs, magnitude
 * 1/d. `forcePower` p raises them to 1/d^p, so near contacts dominate the direction.
 */
function forces(ctx: Ctx): Float64Array[] {
  const { S, K, p } = ctx, pw = p.forcePower + 1;
  const F = S.map((P) => new Float64Array(P.length));
  // 1/d^pw by repeated multiplication (forcePower is an integer), so runs stay bit-identical across browsers
  const inv = (d: number) => { let q = d; for (let k = 1; k < pw; k++) q *= d; return 1 / q; };
  forPairs(S, (k, i, l, j) => {
    const P = S[k], Q = S[l], i1 = nextIdx(i, P.length / 3), j1 = nextIdx(j, Q.length / 3);
    load(a0, P, i); load(a1, P, i1); load(b0, Q, j); load(b1, Q, j1);
    const { d, s, t } = K.segDist(a0, a1, b0, b1);
    const w = 2 * inv(d); // (a - b) / d^(p+1), once per ordered pair
    for (let x = 0; x < 3; x++) {
      const g = ((a0[x] + s * (a1[x] - a0[x])) - (b0[x] + t * (b1[x] - b0[x]))) * w;
      F[k][3 * i + x] += g; F[k][3 * i1 + x] += g; F[l][3 * j + x] -= g; F[l][3 * j1 + x] -= g;
    }
  });
  const flat: { c: number; v: number }[] = [];
  S.forEach((P, c) => { for (let v = 0; v < P.length / 3; v++) flat.push({ c, v }); });
  for (let a = 0; a < flat.length; a++) {
    const P = S[flat[a].c], i = 3 * flat[a].v;
    for (let b = a + 1; b < flat.length; b++) {
      const Q = S[flat[b].c], j = 3 * flat[b].v;
      const dx = Q[j] - P[i], dy = Q[j + 1] - P[i + 1], dz = Q[j + 2] - P[i + 2], w = inv(Math.sqrt(dx * dx + dy * dy + dz * dz));
      const Fi = F[flat[a].c], Fj = F[flat[b].c];
      Fi[i] -= dx * w; Fi[i + 1] -= dy * w; Fi[i + 2] -= dz * w;
      Fj[j] += dx * w; Fj[j + 1] += dy * w; Fj[j + 2] += dz * w;
    }
  }
  return F;
}

/** opt: optSteps sweeps; each computes forces once, then moves every vertex via maxLegalMove. */
export function* opt(ctx: Ctx): Generator<boolean, void> {
  const { S, K, p } = ctx;
  for (let sweep = 0; sweep < p.optSteps; sweep++) {
    const F = forces(ctx);
    for (let c = 0; c < S.length; c++) {
      for (let v = 0; v < S[c].length / 3; v++) {
        const P = S[c], o = 3 * v, f = F[c];
        const fl = Math.sqrt(f[o] * f[o] + f[o + 1] * f[o + 1] + f[o + 2] * f[o + 2]);
        if (!(fl > 0)) continue;
        let gx = f[o] / fl, gy = f[o + 1] / fl, gz = f[o + 2] / fl;
        if (p.straighten > 0) {
          // pull toward the midpoint of the neighbours: straight vertices are the ones elm can delete
          const n = P.length / 3, a = 3 * prevIdx(v, n), b = 3 * nextIdx(v, n);
          const mx = 0.5 * (P[a] + P[b]) - P[o], my = 0.5 * (P[a + 1] + P[b + 1]) - P[o + 1], mz = 0.5 * (P[a + 2] + P[b + 2]) - P[o + 2];
          const ml = Math.sqrt(mx * mx + my * my + mz * mz);
          if (ml > 0) { gx += (p.straighten * mx) / ml; gy += (p.straighten * my) / ml; gz += (p.straighten * mz) / ml; }
          const gl = Math.sqrt(gx * gx + gy * gy + gz * gz);
          if (!(gl > 0)) continue;
          gx /= gl; gy /= gl; gz /= gl;
        }
        let x = P[o] + gx * p.optStep, y = P[o + 1] + gy * p.optStep, z = P[o + 2] + gz * p.optStep;
        const r = Math.sqrt(x * x + y * y + z * z);
        if (r > RADIUS) { x *= RADIUS / r; y *= RADIUS / r; z *= RADIUS / r; }
        target[0] = x; target[1] = y; target[2] = z;
        const t = K.maxLegalMove(S, c, v, target, p.tau, p.softMargin);
        if (t <= 0) continue;
        const nx = P[o] + t * (x - P[o]), ny = P[o + 1] + t * (y - P[o + 1]), nz = P[o + 2] + t * (z - P[o + 2]);
        P[o] = nx; P[o + 1] = ny; P[o + 2] = nz;
        ctx.log.push(Op.Move, c, v, nx, ny, nz);
      }
      yield false;
    }
  }
}
