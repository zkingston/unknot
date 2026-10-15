import { segDist2, edgeDist2 } from './segment';
import { type Scene, prevIdx, nextIdx } from './scene';

/** Visit every unordered pair of non-adjacent edges in the Scene, inter-component pairs included. */
export function forPairs(S: Scene, f: (k: number, i: number, l: number, j: number) => void) {
  for (let k = 0; k < S.length; k++) {
    const n = S[k].length / 3;
    for (let i = 0; i < n; i++) {
      for (let j = i + 2; j < n; j++) if (!(i === 0 && j === n - 1)) f(k, i, k, j);
      for (let l = k + 1; l < S.length; l++) {
        const m = S[l].length / 3;
        for (let j = 0; j < m; j++) f(k, i, l, j);
      }
    }
  }
}

/** Σ 1/d² over all non-adjacent edge pairs. O(N²), stats only. */
export function energy(S: Scene): number {
  let e = 0;
  forPairs(S, (k, i, l, j) => { e += 1 / edgeDist2(S[k], i, S[l], j); });
  return e;
}

/** Smallest distance between non-adjacent edges. O(N²), validation only. */
export function clearance(S: Scene): number {
  let m = Infinity;
  forPairs(S, (k, i, l, j) => { const d = edgeDist2(S[k], i, S[l], j); if (d < m) m = d; });
  return Math.sqrt(m);
}

/** Do two non-adjacent edges touch (clearance 0), or is a coordinate not finite? Sweep and prune: only edges whose
 *  boxes meet can touch, so this is clearance(S) > 0 negated without the O(N²) scan. */
export function touches(S: Scene): boolean {
  const k: number[] = [], e: number[] = [], lo: number[] = [], hi: number[] = [];
  for (let c = 0; c < S.length; c++) {
    const P = S[c], n = P.length / 3;
    for (let i = 0; i < n; i++) {
      const j = i + 1 === n ? 0 : i + 1;
      k.push(c); e.push(i);
      for (let x = 0; x < 3; x++) {
        const a = P[3 * i + x], b = P[3 * j + x];
        if (!Number.isFinite(a)) return true;
        lo.push(a < b ? a : b); hi.push(a < b ? b : a);
      }
    }
  }
  const ord = k.map((_, q) => q).sort((p, q) => lo[3 * p] - lo[3 * q]);
  for (let p = 0; p < ord.length; p++) {
    const a = ord[p];
    for (let q = p + 1; q < ord.length && lo[3 * ord[q]] <= hi[3 * a]; q++) {
      const b = ord[q];
      if (lo[3 * b + 1] > hi[3 * a + 1] || lo[3 * a + 1] > hi[3 * b + 1] || lo[3 * b + 2] > hi[3 * a + 2] || lo[3 * a + 2] > hi[3 * b + 2]) continue;
      if (k[a] === k[b]) {
        const n = S[k[a]].length / 3, i = Math.min(e[a], e[b]), j = Math.max(e[a], e[b]);
        if (j - i < 2 || (i === 0 && j === n - 1)) continue; // same or adjacent edge
      }
      const [u, w] = k[a] < k[b] || (k[a] === k[b] && e[a] < e[b]) ? [a, b] : [b, a]; // forPairs' order: segDist2 rounds asymmetrically
      if (!(edgeDist2(S[k[u]], e[u], S[k[w]], e[w]) > 0)) return true;
    }
  }
  return false;
}

/** Change in energy if vertex v of component c moves to (x,y,z). O(N). */
export function energyDelta(S: Scene, c: number, v: number, x: number, y: number, z: number): number {
  const P = S[c], n = P.length / 3, ia = prevIdx(v, n), ic = nextIdx(v, n);
  const ax = P[3 * ia], ay = P[3 * ia + 1], az = P[3 * ia + 2];
  const bx = P[3 * v], by = P[3 * v + 1], bz = P[3 * v + 2];
  const cx = P[3 * ic], cy = P[3 * ic + 1], cz = P[3 * ic + 2];
  let d = 0;
  for (let k = 0; k < S.length; k++) {
    const Q = S[k], m = Q.length / 3, same = k === c;
    for (let i = 0; i < m; i++) {
      if (same && (i === ia || i === v)) continue;
      const j = i + 1 === m ? 0 : i + 1;
      const px = Q[3 * i], py = Q[3 * i + 1], pz = Q[3 * i + 2], qx = Q[3 * j], qy = Q[3 * j + 1], qz = Q[3 * j + 2];
      if (!(same && j === ia)) d += 1 / segDist2(ax, ay, az, x, y, z, px, py, pz, qx, qy, qz) - 1 / segDist2(ax, ay, az, bx, by, bz, px, py, pz, qx, qy, qz);
      if (!(same && i === ic)) d += 1 / segDist2(x, y, z, cx, cy, cz, px, py, pz, qx, qy, qz) - 1 / segDist2(bx, by, bz, cx, cy, cz, px, py, pz, qx, qy, qz);
    }
  }
  return d;
}
