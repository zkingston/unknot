// Tube mesh along a closed curve: rotation-minimising frames by double reflection (W. Wang et al.,
// "Computation of Rotation Minimizing Frames", ACM TOG 2008), closing twist spread evenly along the length.
const SIDES = 12;
const COS = Float64Array.from({ length: SIDES }, (_, k) => Math.cos((2 * Math.PI * k) / SIDES));
const SIN = Float64Array.from({ length: SIDES }, (_, k) => Math.sin((2 * Math.PI * k) / SIDES));

export interface TubeMesh { centers: Float32Array; normals: Float32Array; index: Uint32Array; rings: number }

/** Arrays of the last mesh, reused while the ring layout (points per curve) stays the same: the index then too. */
let last: { layout: string; mesh: TubeMesh } | null = null;

/** Per-vertex ring centre + unit normal (radius applied in the shader); one ring per curve point. */
export function tubeMesh(curves: Float64Array[]): TubeMesh {
  let rings = 0;
  for (const P of curves) rings += P.length / 3;
  const layout = curves.map((P) => P.length / 3).join(','), same = last?.layout === layout;
  const centers = same ? last!.mesh.centers : new Float32Array(rings * SIDES * 3), normals = same ? last!.mesh.normals : new Float32Array(rings * SIDES * 3);
  const index = same ? last!.mesh.index : new Uint32Array(rings * SIDES * 6);
  let vbase = 0, ib = 0;
  for (const P of curves) {
    const n = P.length / 3;
    if (n < 2) continue;
    const T = new Float64Array(3 * n), R = new Float64Array(3 * n);
    for (let i = 0; i < n; i++) {
      const a = (i + n - 1) % n, b = (i + 1) % n;
      let x = P[3 * b] - P[3 * a], y = P[3 * b + 1] - P[3 * a + 1], z = P[3 * b + 2] - P[3 * a + 2];
      const l = Math.sqrt(x * x + y * y + z * z) || 1; x /= l; y /= l; z /= l;
      T[3 * i] = x; T[3 * i + 1] = y; T[3 * i + 2] = z;
    }
    // initial normal: any vector perpendicular to T0
    {
      const tx = T[0], ty = T[1], tz = T[2];
      let rx = Math.abs(tx) < 0.9 ? 1 : 0, ry = Math.abs(tx) < 0.9 ? 0 : 1, rz = 0;
      const d = rx * tx + ry * ty + rz * tz; rx -= d * tx; ry -= d * ty; rz -= d * tz;
      const l = Math.sqrt(rx * rx + ry * ry + rz * rz); R[0] = rx / l; R[1] = ry / l; R[2] = rz / l;
    }
    const step = (i: number, out: Float64Array, o: number, from: Float64Array, fo: number) => {
      const j = (i + 1) % n;
      const v1x = P[3 * j] - P[3 * i], v1y = P[3 * j + 1] - P[3 * i + 1], v1z = P[3 * j + 2] - P[3 * i + 2];
      const c1 = v1x * v1x + v1y * v1y + v1z * v1z;
      let rx = from[fo], ry = from[fo + 1], rz = from[fo + 2];
      let tx = T[3 * i], ty = T[3 * i + 1], tz = T[3 * i + 2];
      if (c1 > 0) {
        const k1 = (2 / c1) * (v1x * rx + v1y * ry + v1z * rz); rx -= k1 * v1x; ry -= k1 * v1y; rz -= k1 * v1z;
        const k2 = (2 / c1) * (v1x * tx + v1y * ty + v1z * tz); tx -= k2 * v1x; ty -= k2 * v1y; tz -= k2 * v1z;
      }
      const v2x = T[3 * j] - tx, v2y = T[3 * j + 1] - ty, v2z = T[3 * j + 2] - tz, c2 = v2x * v2x + v2y * v2y + v2z * v2z;
      if (c2 > 0) { const k = (2 / c2) * (v2x * rx + v2y * ry + v2z * rz); rx -= k * v2x; ry -= k * v2y; rz -= k * v2z; }
      out[o] = rx; out[o + 1] = ry; out[o + 2] = rz;
    };
    for (let i = 0; i + 1 < n; i++) step(i, R, 3 * (i + 1), R, 3 * i);
    // closing twist: angle from the propagated frame back at 0 to R0, about T0
    const end = new Float64Array(3);
    step(n - 1, end, 0, R, 3 * (n - 1));
    const t0x = T[0], t0y = T[1], t0z = T[2];
    const s0x = t0y * R[2] - t0z * R[1], s0y = t0z * R[0] - t0x * R[2], s0z = t0x * R[1] - t0y * R[0];
    const phi = Math.atan2(end[0] * s0x + end[1] * s0y + end[2] * s0z, end[0] * R[0] + end[1] * R[1] + end[2] * R[2]);
    for (let i = 0; i < n; i++) {
      const ang = (-phi * i) / n, c = Math.cos(ang), s = Math.sin(ang);
      const tx = T[3 * i], ty = T[3 * i + 1], tz = T[3 * i + 2], rx = R[3 * i], ry = R[3 * i + 1], rz = R[3 * i + 2];
      // re-orthogonalise r against t, then s = t × r
      const d = rx * tx + ry * ty + rz * tz; let qx = rx - d * tx, qy = ry - d * ty, qz = rz - d * tz;
      const l = Math.sqrt(qx * qx + qy * qy + qz * qz) || 1; qx /= l; qy /= l; qz /= l;
      const bx = ty * qz - tz * qy, by = tz * qx - tx * qz, bz = tx * qy - ty * qx;
      const nx = c * qx + s * bx, ny = c * qy + s * by, nz = c * qz + s * bz;
      const mx = -s * qx + c * bx, my = -s * qy + c * by, mz = -s * qz + c * bz;
      for (let k = 0; k < SIDES; k++) {
        const ct = COS[k], st = SIN[k], o = 3 * (vbase + i * SIDES + k);
        centers[o] = P[3 * i]; centers[o + 1] = P[3 * i + 1]; centers[o + 2] = P[3 * i + 2];
        normals[o] = ct * nx + st * mx; normals[o + 1] = ct * ny + st * my; normals[o + 2] = ct * nz + st * mz;
      }
    }
    for (let i = 0; i < n && !same; i++) {
      const j = (i + 1) % n;
      for (let k = 0; k < SIDES; k++) {
        const k1 = (k + 1) % SIDES;
        const a = vbase + i * SIDES + k, b = vbase + i * SIDES + k1, c = vbase + j * SIDES + k, d = vbase + j * SIDES + k1;
        // counter-clockwise seen from outside: (b − a) × (c − a) points along the outward normal
        index[ib++] = a; index[ib++] = b; index[ib++] = c;
        index[ib++] = b; index[ib++] = d; index[ib++] = c;
      }
    }
    vbase += n * SIDES;
  }
  const mesh = { centers, normals, index: same ? last!.mesh.index : index.subarray(0, ib), rings };
  last = { layout, mesh };
  return mesh;
}

const STACKS = 8;
/** Ball and stick: a cylinder per edge and a sphere per vertex. Normals carry the radius factor in their length
 *  (the shader scales the radius by it), so both share the tube's shaders and passes. */
export function ballStickMesh(curves: Float64Array[], ball = 1.35, stick = 0.5): TubeMesh {
  let n = 0;
  for (const P of curves) n += P.length / 3;
  const per = 2 * SIDES + (STACKS + 1) * SIDES;
  const centers = new Float32Array(n * per * 3), normals = new Float32Array(n * per * 3), index = new Uint32Array(n * (SIDES * 6 + STACKS * SIDES * 6));
  let v = 0, ib = 0;
  const put = (x: number, y: number, z: number, nx: number, ny: number, nz: number) => {
    centers[3 * v] = x; centers[3 * v + 1] = y; centers[3 * v + 2] = z; normals[3 * v] = nx; normals[3 * v + 1] = ny; normals[3 * v + 2] = nz; v++;
  };
  // ring k of a grid with `cols` columns; quads between consecutive rows, counter-clockwise seen from outside
  const quads = (base: number, rows: number) => {
    for (let r = 0; r + 1 < rows; r++) for (let k = 0; k < SIDES; k++) {
      const k1 = (k + 1) % SIDES, a = base + r * SIDES + k, b = base + r * SIDES + k1, c = base + (r + 1) * SIDES + k, d = base + (r + 1) * SIDES + k1;
      index[ib++] = a; index[ib++] = b; index[ib++] = c; index[ib++] = b; index[ib++] = d; index[ib++] = c;
    }
  };
  for (const P of curves) {
    const m = P.length / 3;
    for (let i = 0; i < m; i++) {
      const j = (i + 1) % m, ax = P[3 * i], ay = P[3 * i + 1], az = P[3 * i + 2];
      let tx = P[3 * j] - ax, ty = P[3 * j + 1] - ay, tz = P[3 * j + 2] - az;
      const l = Math.sqrt(tx * tx + ty * ty + tz * tz) || 1; tx /= l; ty /= l; tz /= l;
      // frame (u, w) perpendicular to t, with (u, w, t) right-handed
      let ux = Math.abs(tx) < 0.9 ? 1 : 0, uy = Math.abs(tx) < 0.9 ? 0 : 1, uz = 0;
      const d = ux * tx + uy * ty + uz * tz; ux -= d * tx; uy -= d * ty; uz -= d * tz;
      const lu = Math.sqrt(ux * ux + uy * uy + uz * uz); ux /= lu; uy /= lu; uz /= lu;
      const wx = ty * uz - tz * uy, wy = tz * ux - tx * uz, wz = tx * uy - ty * ux;
      const base = v;
      for (const s of [0, 1]) for (let k = 0; k < SIDES; k++) {
        const th = (2 * Math.PI * k) / SIDES, c = Math.cos(th) * stick, sn = Math.sin(th) * stick;
        put(ax + s * l * tx, ay + s * l * ty, az + s * l * tz, c * ux + sn * wx, c * uy + sn * wy, c * uz + sn * wz);
      }
      quads(base, 2);
      // sphere at vertex i, rows from the south pole (−z) to the north pole (+z)
      const sb = v;
      for (let r = 0; r <= STACKS; r++) {
        const ph = -Math.PI / 2 + (Math.PI * r) / STACKS, cp = Math.cos(ph), sp = Math.sin(ph);
        for (let k = 0; k < SIDES; k++) { const th = (2 * Math.PI * k) / SIDES; put(ax, ay, az, ball * cp * Math.cos(th), ball * cp * Math.sin(th), ball * sp); }
      }
      quads(sb, STACKS + 1);
    }
  }
  return { centers, normals, index: index.subarray(0, ib), rings: n };
}
