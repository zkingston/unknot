// Zhang–Suen thinning (T. Y. Zhang, C. Y. Suen, CACM 1984) and skeleton-graph helpers.

/** One-pixel skeleton of the mask (8-connected). */
export function zhangSuen(m: Uint8Array, w: number, h: number): Uint8Array {
  const W = w + 2, P = new Uint8Array(W * (h + 2));
  let pix: number[] = [];
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) if (m[y * w + x]) { const i = (y + 1) * W + x + 1; P[i] = 1; pix.push(i); }
  // P2..P9: N, NE, E, SE, S, SW, W, NW
  const off = [-W, -W + 1, 1, W + 1, W, W - 1, -1, -W - 1];
  const del: number[] = [];
  for (let changed = true; changed;) {
    changed = false;
    for (let sub = 0; sub < 2; sub++) {
      del.length = 0;
      for (const i of pix) {
        if (!P[i]) continue;
        let B = 0, A = 0;
        for (let k = 0; k < 8; k++) { const a = P[i + off[k]], b = P[i + off[(k + 1) & 7]]; B += a; if (!a && b) A++; }
        if (B < 2 || B > 6 || A !== 1) continue;
        const p2 = P[i + off[0]], p4 = P[i + off[2]], p6 = P[i + off[4]], p8 = P[i + off[6]];
        if (sub === 0 ? (p2 & p4 & p6) || (p4 & p6 & p8) : (p2 & p4 & p8) || (p2 & p6 & p8)) continue;
        del.push(i);
      }
      for (const i of del) P[i] = 0;
      if (del.length) { changed = true; pix = pix.filter((i) => P[i]); }
    }
  }
  const out = new Uint8Array(w * h);
  for (const i of pix) { const y = Math.floor(i / W) - 1, x = (i % W) - 1; out[y * w + x] = 1; }
  return out;
}

/** 8-neighbours of pixel i that are set in sk. */
function nbrs(sk: Uint8Array, w: number, h: number, i: number, out: number[] = []): number[] {
  out.length = 0;
  const x = i % w, y = (i - x) / w;
  for (let dy = -1; dy <= 1; dy++) { const yy = y + dy; if (yy < 0 || yy >= h) continue;
    for (let dx = -1; dx <= 1; dx++) { if (!dx && !dy) continue; const xx = x + dx; if (xx >= 0 && xx < w && sk[yy * w + xx]) out.push(yy * w + xx); } }
  return out;
}

/** Endpoints (degree 1) among the given skeleton pixels. */
export function endpoints(sk: Uint8Array, w: number, h: number, px: number[]): number[] {
  const nb: number[] = [];
  return px.filter((i) => nbrs(sk, w, h, i, nb).length === 1);
}

/** BFS from src within the pixel set; returns the path from the farthest pixel back to src. */
function bfsFar(sk: Uint8Array, w: number, h: number, src: number, allowed: Set<number>): number[] {
  const prev = new Map<number, number>([[src, -1]]), q = [src], nb: number[] = [];
  let last = src;
  for (let qi = 0; qi < q.length; qi++) {
    last = q[qi];
    for (const j of nbrs(sk, w, h, last, nb)) if (allowed.has(j) && !prev.has(j)) { prev.set(j, last); q.push(j); }
  }
  const path = [last];
  while (prev.get(path[path.length - 1])! >= 0) path.push(prev.get(path[path.length - 1])!);
  return path;
}

/** Shortest path [src .. dst] within the allowed set, or null. */
function bfsPath(sk: Uint8Array, w: number, h: number, src: number, dst: number, allowed: Set<number>): number[] | null {
  const prev = new Map<number, number>([[src, -1]]), q = [src], nb: number[] = [];
  for (let qi = 0; qi < q.length && !prev.has(dst); qi++) for (const j of nbrs(sk, w, h, q[qi], nb)) if (allowed.has(j) && !prev.has(j)) { prev.set(j, q[qi]); q.push(j); }
  if (!prev.has(dst)) return null;
  const out: number[] = []; for (let c = dst; c >= 0; c = prev.get(c)!) out.push(c);
  return out.reverse();
}

/** Longest path through a tree-like skeleton: two BFS passes. */
export function longestPath(sk: Uint8Array, w: number, h: number, px: number[], start: number): number[] {
  const S = new Set(px);
  return bfsFar(sk, w, h, bfsFar(sk, w, h, start, S)[0], S);
}

/** Closed loop: greedy walk, falling back to two disjoint BFS paths to the antipode. */
export function traceCycle(sk: Uint8Array, w: number, h: number, px: number[]): number[] {
  const S = new Set(px), nb: number[] = [];
  const start = px.reduce((a, b) => (b < a ? b : a), px[0]);
  const path = [start], seen = new Set([start]);
  let prev = -1, cur = start;
  for (;;) {
    const next = nbrs(sk, w, h, cur, nb).filter((j) => S.has(j) && j !== prev && !seen.has(j));
    if (!next.length) break;
    // prefer 4-neighbours so staircase corners are not skipped into a dead end
    const cx = cur % w, pick = next.find((j) => j % w === cx || Math.abs(j - cur) === 1) ?? next[0];
    prev = cur; cur = pick; path.push(cur); seen.add(cur);
  }
  const closes = nbrs(sk, w, h, cur, nb).includes(start);
  if (closes && path.length >= 0.8 * px.length) return path;
  // fallback: shortest path to the farthest pixel, then a second one back avoiding the first's interior
  const a = bfsFar(sk, w, h, start, S)[0], p1 = bfsPath(sk, w, h, start, a, S);
  if (!p1) return path;
  const T = new Set(S); for (let k = 1; k < p1.length - 1; k++) T.delete(p1[k]);
  const p2 = bfsPath(sk, w, h, a, start, T);
  return p2 ? [...p1, ...p2.slice(1, -1)] : path;
}
