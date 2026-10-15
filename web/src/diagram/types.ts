export interface StrandRef { comp: number; seg: number; t: number }   // point at param t in [0,1) on segment seg
export interface Crossing  { under: StrandRef; over: StrandRef; flat?: true } // flat = over/under unknown
export interface Diagram {
  comps: Float64Array[];          // each closed curve: x0,y0,x1,y1,...; orientation = point order
  crossings: Crossing[];          // every transverse intersection, exactly once, half-open [0,1) params
  source: 'draw'|'image'|'svg'|'pd'|'gauss'|'braid'|'json'|'preset'|'projection';
  meta?: { name?: string; credit?: string; sourceText?: string };  // sourceText = PD/Gauss/braid as typed
}

/** Arc-length position of a strand point: seg + t. */
export const pos = (s: StrandRef) => s.seg + s.t;

/** Events (passages through crossings) on component c, sorted along the curve. */
export function events(d: Diagram, c: number, selfOnly = false) {
  const ev: { x: number; over: boolean; at: number }[] = [];
  d.crossings.forEach((X, x) => {
    if (selfOnly && X.under.comp !== X.over.comp) return;
    if (X.under.comp === c) ev.push({ x, over: false, at: pos(X.under) });
    if (X.over.comp === c) ev.push({ x, over: true, at: pos(X.over) });
  });
  return ev.sort((a, b) => a.at - b.at);
}

/** Direction of segment seg of 2D component P. */
function segDir(P: Float64Array, seg: number): [number, number] {
  const n = P.length / 2, j = (seg + 1) % n;
  return [P[2 * j] - P[2 * seg], P[2 * j + 1] - P[2 * seg + 1]];
}

/** Sign of a crossing: positive iff u.x*o.y − u.y*o.x < 0 (u under, o over, y-up). */
export function crossingSign(d: Diagram, X: Crossing): 1 | -1 {
  const u = segDir(d.comps[X.under.comp], X.under.seg), o = segDir(d.comps[X.over.comp], X.over.seg);
  return u[0] * o[1] - u[1] * o[0] < 0 ? 1 : -1;
}

export function extent2(comps: Float64Array[]) {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const P of comps) for (let i = 0; i < P.length; i += 2) {
    if (P[i] < x0) x0 = P[i]; if (P[i] > x1) x1 = P[i]; if (P[i + 1] < y0) y0 = P[i + 1]; if (P[i + 1] > y1) y1 = P[i + 1];
  }
  return { x0, y0, x1, y1, ext: Math.max(x1 - x0, y1 - y0) };
}

/** Split = no crossings between different components. */
export const isSplit = (d: Diagram) => d.crossings.every((X) => X.under.comp === X.over.comp);
