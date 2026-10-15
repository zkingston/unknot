export type Polygon = Float64Array;  // x0,y0,z0,x1,...; closed; n = length/3 >= 3
export type Scene = Polygon[];       // one Polygon per component; a knot is a Scene of length 1

export const totalVerts = (S: Scene) => S.reduce((s, P) => s + P.length / 3, 0);
export const cloneScene = (S: Scene): Scene => S.map((P) => P.slice());
export const prevIdx = (i: number, n: number) => (i === 0 ? n - 1 : i - 1);
export const nextIdx = (i: number, n: number) => (i + 1 === n ? 0 : i + 1);

/** Per component, the identity index map i -> i. */
export const identityIds = (S: Scene) => S.map((P) => Array.from({ length: P.length / 3 }, (_, i) => i));

/** Insert (x,y,z) as vertex v of component c (between the old v − 1 and v). */
export function insertVertex(S: Scene, c: number, v: number, x: number, y: number, z: number) {
  const P = S[c], Q = new Float64Array(P.length + 3);
  Q.set(P.subarray(0, 3 * v));
  Q[3 * v] = x; Q[3 * v + 1] = y; Q[3 * v + 2] = z;
  Q.set(P.subarray(3 * v), 3 * v + 3);
  S[c] = Q;
}

export function removeVertex(S: Scene, c: number, v: number) {
  const P = S[c], Q = new Float64Array(P.length - 3);
  Q.set(P.subarray(0, 3 * v));
  Q.set(P.subarray(3 * v + 3), 3 * v);
  S[c] = Q;
}

/** Cosine of the angle at vertex v (−1 = straight). Straightest first = ascending. */
export function cosAt(P: Polygon, v: number): number {
  const n = P.length / 3, a = prevIdx(v, n), c = nextIdx(v, n);
  const ux = P[3 * a] - P[3 * v], uy = P[3 * a + 1] - P[3 * v + 1], uz = P[3 * a + 2] - P[3 * v + 2];
  const wx = P[3 * c] - P[3 * v], wy = P[3 * c + 1] - P[3 * v + 1], wz = P[3 * c + 2] - P[3 * v + 2];
  const d = Math.sqrt((ux * ux + uy * uy + uz * uz) * (wx * wx + wy * wy + wz * wz));
  return d > 0 ? (ux * wx + uy * wy + uz * wz) / d : 1;
}

/** Vertices (comp, v) of components with more than 3 vertices, straightest first; stable on ties. */
export function straightestFirst(S: Scene): { c: number; v: number; k: number }[] {
  const out: { c: number; v: number; k: number }[] = [];
  for (let c = 0; c < S.length; c++) {
    const n = S[c].length / 3;
    if (n <= 3) continue;
    for (let v = 0; v < n; v++) out.push({ c, v, k: cosAt(S[c], v) });
  }
  return out.sort((x, y) => x.k - y.k);
}
