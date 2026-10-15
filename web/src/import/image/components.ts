// Distance transforms, erosion and 8-connected labelling on binary masks (1 = ink).

/** Exact Euclidean distance from each ink pixel to the nearest background pixel (P. Felzenszwalb, D. Huttenlocher 2012). */
export function edt(m: Uint8Array, w: number, h: number): Float32Array {
  const INF = 1e20, f = new Float64Array(w * h);
  for (let i = 0; i < f.length; i++) f[i] = m[i] ? INF : 0;
  const n = Math.max(w, h), z = new Float64Array(n + 1), v = new Int32Array(n), d = new Float64Array(n), line = new Float64Array(n);
  const dt1 = (len: number) => {
    let k = 0; v[0] = 0; z[0] = -INF; z[1] = INF;
    for (let q = 1; q < len; q++) {
      let s = ((line[q] + q * q) - (line[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
      while (s <= z[k]) { k--; s = ((line[q] + q * q) - (line[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]); }
      k++; v[k] = q; z[k] = s; z[k + 1] = INF;
    }
    k = 0;
    for (let q = 0; q < len; q++) { while (z[k + 1] < q) k++; d[q] = (q - v[k]) * (q - v[k]) + line[v[k]]; }
  };
  for (let x = 0; x < w; x++) { for (let y = 0; y < h; y++) line[y] = f[y * w + x]; dt1(h); for (let y = 0; y < h; y++) f[y * w + x] = d[y]; }
  for (let y = 0; y < h; y++) { for (let x = 0; x < w; x++) line[x] = f[y * w + x]; dt1(w); for (let x = 0; x < w; x++) f[y * w + x] = d[x]; }
  const out = new Float32Array(w * h);
  for (let i = 0; i < out.length; i++) out[i] = Math.sqrt(f[i]);
  return out;
}

/** City-block distance to the nearest background pixel, outside the image counting as background. */
export function l1dist(m: Uint8Array, w: number, h: number): Int32Array {
  const d = new Int32Array(w * h);
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    const i = y * w + x;
    if (!m[i]) continue;
    d[i] = Math.min(y ? d[i - w] : 0, x ? d[i - 1] : 0) + 1;
  }
  for (let y = h - 1; y >= 0; y--) for (let x = w - 1; x >= 0; x--) {
    const i = y * w + x;
    if (!m[i]) continue;
    d[i] = Math.min(d[i], Math.min(y < h - 1 ? d[i + w] : 0, x < w - 1 ? d[i + 1] : 0) + 1);
  }
  return d;
}

/** k iterations of erosion by the 4-neighbour cross = keep pixels whose city-block distance exceeds k. */
export function erode(L1: Int32Array, k: number): Uint8Array {
  const o = new Uint8Array(L1.length);
  for (let i = 0; i < o.length; i++) o[i] = L1[i] > k ? 1 : 0;
  return o;
}

/** 8-connected component labels (1..n; 0 = background) and sizes. */
export function label8(m: Uint8Array, w: number, h: number): { lab: Int32Array; n: number; size: number[] } {
  const lab = new Int32Array(w * h), size = [0], stack: number[] = [];
  let n = 0;
  for (let s = 0; s < m.length; s++) {
    if (!m[s] || lab[s]) continue;
    lab[s] = ++n; size.push(0); stack.push(s);
    while (stack.length) {
      const i = stack.pop()!, x = i % w, y = (i - x) / w;
      size[n]++;
      for (let dy = -1; dy <= 1; dy++) { const yy = y + dy; if (yy < 0 || yy >= h) continue;
        for (let dx = -1; dx <= 1; dx++) { const xx = x + dx; if (xx < 0 || xx >= w) continue; const j = yy * w + xx; if (m[j] && !lab[j]) { lab[j] = n; stack.push(j); } } }
    }
  }
  return { lab, n, size };
}
