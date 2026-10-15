// Steps 3–5: stroke width, split touching strands by erosion, centrelines as skeleton paths (y-up px).
import { edt, l1dist, erode, label8 } from './components';
import { zhangSuen, endpoints, longestPath, traceCycle } from './skeleton';

export interface Arc { xy: Float64Array; closed: boolean }
export interface Traced { arcs: Arc[]; sw: number; ink: Uint8Array }

/** Skeleton pixels grouped by component label. */
function groupByLabel(sk: Uint8Array, lab: Int32Array, n: number): number[][] {
  const g: number[][] = Array.from({ length: n + 1 }, () => []);
  for (let i = 0; i < sk.length; i++) if (sk[i] && lab[i]) g[lab[i]].push(i);
  return g;
}

export function traceArcs(mask: Uint8Array, w: number, h: number): Traced {
  // 3. stroke width = 2 × median distance-transform value on skeleton pixels
  const sk0 = zhangSuen(mask, w, h), dt = edt(mask, w, h), vals: number[] = [];
  for (let i = 0; i < sk0.length; i++) if (sk0[i]) vals.push(dt[i]);
  if (!vals.length) return { arcs: [], sw: 1, ink: mask };
  vals.sort((a, b) => a - b);
  const sw = 2 * vals[vals.length >> 1];
  // 4. erode by round(0.2 sw); while a component's skeleton has more than 2 endpoints, erode 1 px more
  const L1 = l1dist(mask, w, h), k0 = Math.round(0.2 * sw);
  // ponytail: cap so thin strokes survive; comps that still branch are traced along their longest path
  const kmax = Math.max(k0, Math.floor(0.45 * sw));
  let k = k0, er = erode(L1, k), L = label8(er, w, h), sk = zhangSuen(er, w, h), groups = groupByLabel(sk, L.lab, L.n);
  while (k < kmax && groups.some((px) => endpoints(sk, w, h, px).length > 2)) {
    const er2 = erode(L1, k + 1);
    if (!er2.some((x) => x)) break;
    k++; er = er2; L = label8(er, w, h); sk = zhangSuen(er, w, h); groups = groupByLabel(sk, L.lab, L.n);
  }
  // 5. centreline of each component: longest skeleton path; no endpoints = closed loop
  const arcs: Arc[] = [];
  for (let c = 1; c <= L.n; c++) {
    const px = groups[c];
    if (L.size[c] < sw * sw || px.length < 2) continue;
    const ends = endpoints(sk, w, h, px);
    const path = ends.length ? longestPath(sk, w, h, px, ends[0]) : traceCycle(sk, w, h, px);
    if (path.length < 2) continue;
    const xy = new Float64Array(2 * path.length);
    path.forEach((i, k2) => { const x = i % w; xy[2 * k2] = x; xy[2 * k2 + 1] = -(i - x) / w; });
    arcs.push({ xy, closed: !ends.length });
  }
  return { arcs, sw, ink: er };
}
