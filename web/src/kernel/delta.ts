// Delta test: moving vertex v from b to b' sweeps triangles (a,b,b') and (c,b,b').
// Segment–triangle test: T. Möller and B. Trumbore, 1997, with a coplanar branch.
import { segDist2 } from './segment';
import { type Scene, prevIdx, nextIdx } from './scene';

export const LEGAL = 0, PIERCE = 1, TAU = 2;
/** Segments meeting closer than this count as touching. */
export const CONTACT = 1e-9;
export const SHRINK = 1e-9;   // shrink edges sharing a vertex with a triangle at the shared end (fraction of length)
const PLANE_EPS = 1e-13;

const orient2 = (ax: number, ay: number, bx: number, by: number, cx: number, cy: number) => {
  const ux = bx - ax, uy = by - ay, vx = cx - ax, vy = cy - ay;
  const o = ux * vy - uy * vx;
  return Math.abs(o) <= 1e-14 * Math.sqrt((ux * ux + uy * uy) * (vx * vx + vy * vy)) ? 0 : o;
};
const onSeg2 = (ax: number, ay: number, bx: number, by: number, px: number, py: number) =>
  Math.min(ax, bx) <= px && px <= Math.max(ax, bx) && Math.min(ay, by) <= py && py <= Math.max(ay, by);
function segSeg2(ax: number, ay: number, bx: number, by: number, cx: number, cy: number, dx: number, dy: number) {
  const o1 = orient2(ax, ay, bx, by, cx, cy), o2 = orient2(ax, ay, bx, by, dx, dy);
  const o3 = orient2(cx, cy, dx, dy, ax, ay), o4 = orient2(cx, cy, dx, dy, bx, by);
  if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0))) return true;
  return (o1 === 0 && onSeg2(ax, ay, bx, by, cx, cy)) || (o2 === 0 && onSeg2(ax, ay, bx, by, dx, dy))
    || (o3 === 0 && onSeg2(cx, cy, dx, dy, ax, ay)) || (o4 === 0 && onSeg2(cx, cy, dx, dy, bx, by));
}
function inTri2(ax: number, ay: number, bx: number, by: number, cx: number, cy: number, px: number, py: number) {
  const o1 = orient2(ax, ay, bx, by, px, py), o2 = orient2(bx, by, cx, cy, px, py), o3 = orient2(cx, cy, ax, ay, px, py);
  return (o1 >= 0 && o2 >= 0 && o3 >= 0) || (o1 <= 0 && o2 <= 0 && o3 <= 0);
}

/** Does closed segment PQ meet closed triangle ABC? */
export function segTri(
  ax: number, ay: number, az: number, bx: number, by: number, bz: number, cx: number, cy: number, cz: number,
  px: number, py: number, pz: number, qx: number, qy: number, qz: number,
): boolean {
  const e1x = bx - ax, e1y = by - ay, e1z = bz - az, e2x = cx - ax, e2y = cy - ay, e2z = cz - az;
  const dx = qx - px, dy = qy - py, dz = qz - pz;
  const nx = e1y * e2z - e1z * e2y, ny = e1z * e2x - e1x * e2z, nz = e1x * e2y - e1y * e2x;
  const l1 = Math.sqrt(e1x * e1x + e1y * e1y + e1z * e1z), l2 = Math.sqrt(e2x * e2x + e2y * e2y + e2z * e2z);
  const nn = Math.sqrt(nx * nx + ny * ny + nz * nz);
  if (nn <= 1e-12 * l1 * l2) {
    // degenerate triangle: its hull is the longest of its three sides
    const l3x = cx - bx, l3y = cy - by, l3z = cz - bz, l3 = l3x * l3x + l3y * l3y + l3z * l3z;
    const L1 = l1 * l1, L2 = l2 * l2;
    let d2: number;
    if (L1 >= L2 && L1 >= l3) d2 = segDist2(ax, ay, az, bx, by, bz, px, py, pz, qx, qy, qz);
    else if (L2 >= l3) d2 = segDist2(ax, ay, az, cx, cy, cz, px, py, pz, qx, qy, qz);
    else d2 = segDist2(bx, by, bz, cx, cy, cz, px, py, pz, qx, qy, qz);
    return d2 <= PLANE_EPS * PLANE_EPS;
  }
  const pvx = dy * e2z - dz * e2y, pvy = dz * e2x - dx * e2z, pvz = dx * e2y - dy * e2x;
  const det = e1x * pvx + e1y * pvy + e1z * pvz;
  const ld = Math.sqrt(dx * dx + dy * dy + dz * dz);
  if (Math.abs(det) <= 1e-12 * l1 * l2 * ld) {
    // segment parallel to the plane: off-plane => miss; else exact 2D overlap in the plane
    const sp = (nx * (px - ax) + ny * (py - ay) + nz * (pz - az)) / nn;
    const sq = (nx * (qx - ax) + ny * (qy - ay) + nz * (qz - az)) / nn;
    if ((sp > PLANE_EPS && sq > PLANE_EPS) || (sp < -PLANE_EPS && sq < -PLANE_EPS)) return false;
    const anx = Math.abs(nx), any = Math.abs(ny), anz = Math.abs(nz);
    // drop the dominant normal axis
    let A0: number, A1: number, B0: number, B1: number, C0: number, C1: number, P0: number, P1: number, Q0: number, Q1: number;
    if (anx >= any && anx >= anz) { A0 = ay; A1 = az; B0 = by; B1 = bz; C0 = cy; C1 = cz; P0 = py; P1 = pz; Q0 = qy; Q1 = qz; }
    else if (any >= anz) { A0 = az; A1 = ax; B0 = bz; B1 = bx; C0 = cz; C1 = cx; P0 = pz; P1 = px; Q0 = qz; Q1 = qx; }
    else { A0 = ax; A1 = ay; B0 = bx; B1 = by; C0 = cx; C1 = cy; P0 = px; P1 = py; Q0 = qx; Q1 = qy; }
    return inTri2(A0, A1, B0, B1, C0, C1, P0, P1) || inTri2(A0, A1, B0, B1, C0, C1, Q0, Q1)
      || segSeg2(A0, A1, B0, B1, P0, P1, Q0, Q1) || segSeg2(B0, B1, C0, C1, P0, P1, Q0, Q1)
      || segSeg2(C0, C1, A0, A1, P0, P1, Q0, Q1);
  }
  const inv = 1 / det;
  const tx = px - ax, ty = py - ay, tz = pz - az;
  const u = (tx * pvx + ty * pvy + tz * pvz) * inv;
  if (u < 0 || u > 1) return false;
  const qvx = ty * e1z - tz * e1y, qvy = tz * e1x - tx * e1z, qvz = tx * e1y - ty * e1x;
  const v = (dx * qvx + dy * qvy + dz * qvz) * inv;
  if (v < 0 || u + v > 1) return false;
  const w = (e2x * qvx + e2y * qvy + e2z * qvz) * inv;
  return w >= 0 && w <= 1;
}

/** Delta test for moving vertex v of component c to (tx,ty,tz): LEGAL, PIERCE or TAU. */
export function deltaTest(S: Scene, c: number, v: number, tx: number, ty: number, tz: number, tau: number): number {
  const P = S[c], n = P.length / 3, ia = prevIdx(v, n), ic = nextIdx(v, n);
  const ax = P[3 * ia], ay = P[3 * ia + 1], az = P[3 * ia + 2];
  const bx = P[3 * v], by = P[3 * v + 1], bz = P[3 * v + 2];
  const cx = P[3 * ic], cy = P[3 * ic + 1], cz = P[3 * ic + 2];
  const tau2 = tau * tau;
  let tauFail = false;
  for (let k = 0; k < S.length; k++) {
    const Q = S[k], m = Q.length / 3, same = k === c;
    for (let i = 0; i < m; i++) {
      if (same && (i === ia || i === v)) continue;
      const j = i + 1 === m ? 0 : i + 1;
      const px = Q[3 * i], py = Q[3 * i + 1], pz = Q[3 * i + 2], qx = Q[3 * j], qy = Q[3 * j + 1], qz = Q[3 * j + 2];
      // the vertex path (b, b') is the edge both triangles share: test it robustly, not through barycentric rounding
      if (segDist2(bx, by, bz, tx, ty, tz, px, py, pz, qx, qy, qz) < CONTACT * CONTACT) return PIERCE;
      const endA = same && j === ia, startC = same && i === ic;
      // triangle (a, b, b'): shrink at a
      const q1x = endA ? qx + SHRINK * (px - qx) : qx, q1y = endA ? qy + SHRINK * (py - qy) : qy, q1z = endA ? qz + SHRINK * (pz - qz) : qz;
      if (segTri(ax, ay, az, bx, by, bz, tx, ty, tz, px, py, pz, q1x, q1y, q1z)) return PIERCE;
      // triangle (c, b, b'): shrink at c
      const p2x = startC ? px + SHRINK * (qx - px) : px, p2y = startC ? py + SHRINK * (qy - py) : py, p2z = startC ? pz + SHRINK * (qz - pz) : pz;
      if (segTri(cx, cy, cz, bx, by, bz, tx, ty, tz, p2x, p2y, p2z, qx, qy, qz)) return PIERCE;
      if (!tauFail) {
        if (!endA && segDist2(ax, ay, az, tx, ty, tz, px, py, pz, qx, qy, qz) < tau2) tauFail = true;
        else if (!startC && segDist2(tx, ty, tz, cx, cy, cz, px, py, pz, qx, qy, qz) < tau2) tauFail = true;
      }
    }
  }
  return tauFail ? TAU : LEGAL;
}

/** Can v be deleted by a Delta move to the midpoint of its neighbours? */
export function removeLegal(S: Scene, c: number, v: number, tau: number): boolean {
  const P = S[c], n = P.length / 3;
  if (n <= 3) return false;
  const ia = prevIdx(v, n), ic = nextIdx(v, n);
  return deltaTest(S, c, v, 0.5 * (P[3 * ia] + P[3 * ic]), 0.5 * (P[3 * ia + 1] + P[3 * ic + 1]),
    0.5 * (P[3 * ia + 2] + P[3 * ic + 2]), tau) === LEGAL;
}

/** Smallest distance from the two edges at v (v at its current place) to their non-adjacent edges. O(N). */
function movedEdgesClearance(S: Scene, c: number, v: number): number {
  const P = S[c], n = P.length / 3, ia = prevIdx(v, n), ic = nextIdx(v, n);
  const ax = P[3 * ia], ay = P[3 * ia + 1], az = P[3 * ia + 2], cx = P[3 * ic], cy = P[3 * ic + 1], cz = P[3 * ic + 2];
  const x = P[3 * v], y = P[3 * v + 1], z = P[3 * v + 2];
  let m = Infinity;
  for (let k = 0; k < S.length; k++) {
    const Q = S[k], mm = Q.length / 3, same = k === c;
    for (let i = 0; i < mm; i++) {
      if (same && (i === ia || i === v)) continue;
      const j = i + 1 === mm ? 0 : i + 1;
      const px = Q[3 * i], py = Q[3 * i + 1], pz = Q[3 * i + 2], qx = Q[3 * j], qy = Q[3 * j + 1], qz = Q[3 * j + 2];
      if (!(same && j === ia)) m = Math.min(m, segDist2(ax, ay, az, x, y, z, px, py, pz, qx, qy, qz));
      if (!(same && i === ic)) m = Math.min(m, segDist2(x, y, z, cx, cy, cz, px, py, pz, qx, qy, qz));
    }
  }
  return Math.sqrt(m);
}

/** Do both edges at v, with v placed at (x,y,z), clear every non-adjacent edge by tau? */
function movedEdgesClear(S: Scene, c: number, v: number, x: number, y: number, z: number, tau: number): boolean {
  const P = S[c], n = P.length / 3, ia = prevIdx(v, n), ic = nextIdx(v, n);
  const ax = P[3 * ia], ay = P[3 * ia + 1], az = P[3 * ia + 2], cx = P[3 * ic], cy = P[3 * ic + 1], cz = P[3 * ic + 2];
  const tau2 = tau * tau;
  for (let k = 0; k < S.length; k++) {
    const Q = S[k], m = Q.length / 3, same = k === c;
    for (let i = 0; i < m; i++) {
      if (same && (i === ia || i === v)) continue;
      const j = i + 1 === m ? 0 : i + 1;
      const px = Q[3 * i], py = Q[3 * i + 1], pz = Q[3 * i + 2], qx = Q[3 * j], qy = Q[3 * j + 1], qz = Q[3 * j + 2];
      if (!(same && j === ia) && segDist2(ax, ay, az, x, y, z, px, py, pz, qx, qy, qz) < tau2) return false;
      if (!(same && i === ic) && segDist2(x, y, z, cx, cy, cz, px, py, pz, qx, qy, qz) < tau2) return false;
    }
  }
  return true;
}

/**
 * A safe fraction t in [0,1] of the straight move of vertex v toward (tx,ty,tz), not necessarily the largest.
 * The coplanarity triple product of each moving edge with each non-adjacent edge is linear in t; the smallest
 * root where the segments actually meet bounds t. Then halve until the moved edges clear tau. O(N).
 */
export function maxLegalMove(S: Scene, c: number, v: number, tx: number, ty: number, tz: number, tau: number, soft = 0): number {
  const P = S[c], n = P.length / 3, ia = prevIdx(v, n), ic = nextIdx(v, n);
  const bx = P[3 * v], by = P[3 * v + 1], bz = P[3 * v + 2];
  const Dx = tx - bx, Dy = ty - by, Dz = tz - bz;
  const lD = Math.sqrt(Dx * Dx + Dy * Dy + Dz * Dz);
  if (lD === 0) return 1;
  let tstar = 1, contact = false;
  const C2 = CONTACT * CONTACT;
  for (let k = 0; k < S.length; k++) {
    const Q = S[k], m = Q.length / 3, same = k === c;
    for (let i = 0; i < m; i++) {
      if (same && (i === ia || i === v)) continue;
      const j = i + 1 === m ? 0 : i + 1;
      const px = Q[3 * i], py = Q[3 * i + 1], pz = Q[3 * i + 2], qx = Q[3 * j], qy = Q[3 * j + 1], qz = Q[3 * j + 2];
      for (let side = 0; side < 2; side++) {
        let F: number;
        if (side === 0) { if (same && j === ia) continue; F = 3 * ia; } else { if (same && i === ic) continue; F = 3 * ic; }
        const fx = P[F], fy = P[F + 1], fz = P[F + 2];
        const ux = px - fx, uy = py - fy, uz = pz - fz, wx = qx - fx, wy = qy - fy, wz = qz - fz;
        const Nx = uy * wz - uz * wy, Ny = uz * wx - ux * wz, Nz = ux * wy - uy * wx;
        const gx = bx - fx, gy = by - fy, gz = bz - fz;
        const f0 = gx * Nx + gy * Ny + gz * Nz, f1 = Dx * Nx + Dy * Ny + Dz * Nz;
        const lN = Math.sqrt(Nx * Nx + Ny * Ny + Nz * Nz);
        // the edge on a line through the fixed end (just split), or in the plane of the motion (moves within a flat
        // strand): the triple product says nothing, so ask the swept triangle itself (segTri decides coplanar overlap in 2D)
        if (lN <= 1e-12 * Math.sqrt((ux * ux + uy * uy + uz * uz) * (wx * wx + wy * wy + wz * wz))
          || (Math.abs(f1) <= 1e-12 * lD * lN && Math.abs(f0) <= 1e-12 * Math.sqrt(gx * gx + gy * gy + gz * gz) * lN)) {
          if (segTri(fx, fy, fz, bx, by, bz, tx, ty, tz, px, py, pz, qx, qy, qz)) return 0;
          continue;
        }
        if (f1 === 0) continue;
        const r = -f0 / f1;
        if (r < 0 || r > 1 || (contact && r >= tstar)) continue;
        const rx = bx + r * Dx, ry = by + r * Dy, rz = bz + r * Dz;
        if (segDist2(fx, fy, fz, rx, ry, rz, px, py, pz, qx, qy, qz) < C2) { tstar = r; contact = true; }
      }
    }
  }
  // soft margin: the moved edges must not end closer than `soft`, or than they already are if that is less
  const need = soft > 0 ? Math.max(tau, Math.min(soft, movedEdgesClearance(S, c, v))) : tau;
  let t = contact ? 0.98 * tstar : 1;
  for (let h = 0; h <= 20; h++, t *= 0.5) {
    if (movedEdgesClear(S, c, v, bx + t * Dx, by + t * Dy, bz + t * Dz, need)) return t;
  }
  return 0;
}
