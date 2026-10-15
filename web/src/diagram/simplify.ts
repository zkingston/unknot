// Greedy vertex removal; each removal is one Delta move (triangle abc pierced by nothing), so the knot type cannot change.
import { segTri, CONTACT, SHRINK } from '../kernel/delta';
import { segDist2 } from '../kernel/segment';
import { type Scene, cosAt, prevIdx, nextIdx, removeVertex } from '../kernel/scene';

export interface SimplifyOpts { epsilon?: number; margin?: number; near?: number; keep?: number }

/** Can vertex v of comp c be removed? Shape (epsilon), Delta move, and margin to edges more than `near` steps away. */
function removable(S: Scene, c: number, v: number, eps: number, margin: number, near: number): boolean {
  const P = S[c], n = P.length / 3, ia = prevIdx(v, n), ic = nextIdx(v, n);
  const ax = P[3 * ia], ay = P[3 * ia + 1], az = P[3 * ia + 2], bx = P[3 * v], by = P[3 * v + 1], bz = P[3 * v + 2];
  const cx = P[3 * ic], cy = P[3 * ic + 1], cz = P[3 * ic + 2];
  if (eps < Infinity) {
    const ux = cx - ax, uy = cy - ay, uz = cz - az, l2 = ux * ux + uy * uy + uz * uz;
    let t = l2 > 0 ? ((bx - ax) * ux + (by - ay) * uy + (bz - az) * uz) / l2 : 0;
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    const dx = ax + t * ux - bx, dy = ay + t * uy - by, dz = az + t * uz - bz;
    if (dx * dx + dy * dy + dz * dz > eps * eps) return false;
  }
  const m2 = margin * margin, c2 = CONTACT * CONTACT;
  // an edge whose box is further than this from the triangle's passes every test below (tolerances are ~1e-13)
  const far = Math.max(margin, CONTACT) + 1e-6;
  const x0 = Math.min(ax, bx, cx) - far, x1 = Math.max(ax, bx, cx) + far, y0 = Math.min(ay, by, cy) - far, y1 = Math.max(ay, by, cy) + far;
  const z0 = Math.min(az, bz, cz) - far, z1 = Math.max(az, bz, cz) + far;
  for (let k = 0; k < S.length; k++) {
    const Q = S[k], m = Q.length / 3, same = k === c;
    for (let i = 0; i < m; i++) {
      if (same && (i === ia || i === v)) continue;
      const j = i + 1 === m ? 0 : i + 1;
      let px = Q[3 * i], py = Q[3 * i + 1], pz = Q[3 * i + 2], qx = Q[3 * j], qy = Q[3 * j + 1], qz = Q[3 * j + 2];
      if ((px < x0 && qx < x0) || (px > x1 && qx > x1) || (py < y0 && qy < y0) || (py > y1 && qy > y1) || (pz < z0 && qz < z0) || (pz > z1 && qz > z1)) continue;
      const adjA = same && j === ia, adjC = same && i === ic;
      if (adjA) { qx += SHRINK * (px - qx); qy += SHRINK * (py - qy); qz += SHRINK * (pz - qz); }
      if (adjC) { px += SHRINK * (qx - px); py += SHRINK * (qy - py); pz += SHRINK * (qz - pz); }
      if (segTri(ax, ay, az, bx, by, bz, cx, cy, cz, px, py, pz, qx, qy, qz)) return false;
      if (adjA || adjC) continue;
      const d2 = segDist2(ax, ay, az, cx, cy, cz, px, py, pz, qx, qy, qz); // unshrunk: neither edge is adjacent here
      if (d2 < c2) return false;
      if (d2 >= m2) continue;
      // too close, unless "near" along the curve: within `near` steps, or within 3 × margin of arc length (dense samplings)
      if (!same) return false;
      const dk = Math.min((i - v + m) % m, (v - i + m) % m);
      if (dk > near && !arcWithin(Q, v, i, 3 * margin)) return false;
    }
  }
  return true;
}

/** Is edge i within arc length `L` of vertex v along polygon Q (either direction)? */
function arcWithin(Q: Float64Array, v: number, i: number, L: number) {
  const m = Q.length / 3;
  for (const dir of [1, -1]) {
    let s = 0, k = v;
    while (s <= L) {
      const j = (k + dir + m) % m;
      if ((dir > 0 ? k : j) === i) return true;
      const dx = Q[3 * j] - Q[3 * k], dy = Q[3 * j + 1] - Q[3 * k + 1], dz = Q[3 * j + 2] - Q[3 * k + 2];
      s += Math.sqrt(dx * dx + dy * dy + dz * dz); k = j;
      if (k === v) break;
    }
  }
  return false;
}

/** Repeated passes, straightest first; never two adjacent removals in one pass. epsilon = Infinity: full simplification. */
export function simplify(scene: Scene, opts: SimplifyOpts = {}): Scene {
  const eps = opts.epsilon ?? 0.01, margin = opts.margin ?? 0.01, near = opts.near ?? 3, keep = opts.keep ?? 3;
  const S = scene.map((P) => P.slice());
  for (let changed = true; changed;) {
    changed = false;
    for (let c = 0; c < S.length; c++) {
      const n0 = S[c].length / 3;
      if (n0 <= keep) continue;
      const order = Array.from({ length: n0 }, (_, v) => ({ v, k: cosAt(S[c], v) })).sort((a, b) => a.k - b.k);
      const ids = Array.from({ length: n0 }, (_, v) => v), blocked = new Uint8Array(n0);
      for (const { v: id } of order) {
        if (blocked[id] || ids.length <= keep) continue;
        const v = ids.indexOf(id), n = ids.length;
        if (!removable(S, c, v, eps, margin, near)) continue;
        blocked[ids[prevIdx(v, n)]] = 1; blocked[ids[nextIdx(v, n)]] = 1;
        removeVertex(S, c, v); ids.splice(v, 1);
        changed = true;
      }
    }
  }
  return S;
}
