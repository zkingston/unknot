// Raster -> binary ink mask: composite over white, grey, area-average downscale, Otsu or Sauvola, despeckle.
export interface RGBAImage { width: number; height: number; data: Uint8ClampedArray | Uint8Array }
export interface Prepared { mask: Uint8Array; width: number; height: number; method: 'otsu' | 'sauvola' | 'fixed' }
export type Method = 'otsu' | 'sauvola' | 'auto';

/** Composite over white and convert to grey in [0,1]. */
function toGrey(img: RGBAImage): Float32Array {
  const n = img.width * img.height, g = new Float32Array(n), d = img.data;
  for (let i = 0; i < n; i++) {
    const a = d[4 * i + 3] / 255;
    g[i] = ((d[4 * i] + d[4 * i + 1] + d[4 * i + 2]) / 765) * a + (1 - a);
  }
  return g;
}

/** Area-average downscale (separable, fractional coverage) so the longest side is <= maxSide. */
function downscale(g: Float32Array, w: number, h: number, maxSide: number) {
  const f = Math.max(w, h) / maxSide;
  if (f <= 1) return { g, w, h, f: 1 };
  const W = Math.max(1, Math.round(w / f)), H = Math.max(1, Math.round(h / f)), fx = w / W, fy = h / H;
  const pass = (src: Float32Array, n: number, stride: number, lines: number, lineStride: number, N: number, fs: number) => {
    // resample each line of length n (step `stride`) to N samples
    const out = new Float32Array(N * lines);
    for (let l = 0; l < lines; l++) for (let x = 0; x < N; x++) {
      const a = x * fs, b = Math.min(n, (x + 1) * fs);
      let s = 0;
      for (let i = Math.floor(a); i < b; i++) { const wgt = Math.min(b, i + 1) - Math.max(a, i); s += wgt * src[l * lineStride + i * stride]; }
      out[l * N + x] = s / (b - a);
    }
    return out;
  };
  const hx = pass(g, w, 1, h, w, W, fx);                       // rows: h × W
  const t = new Float32Array(W * h);                           // transpose to columns
  for (let y = 0; y < h; y++) for (let x = 0; x < W; x++) t[x * h + y] = hx[y * W + x];
  const vy = pass(t, h, 1, W, h, H, fy);                       // W × H (column-major)
  const out = new Float32Array(W * H);
  for (let x = 0; x < W; x++) for (let y = 0; y < H; y++) out[y * W + x] = vy[x * H + y];
  return { g: out, w: W, h: H, f };
}

/** Otsu 1979: threshold maximising between-class variance of the 256-bin histogram. */
function otsu(g: Float32Array): number {
  const hist = new Float64Array(256);
  for (let i = 0; i < g.length; i++) hist[Math.min(255, Math.max(0, Math.round(g[i] * 255)))]++;
  let sum = 0;
  for (let i = 0; i < 256; i++) sum += i * hist[i];
  let wB = 0, sB = 0, best = 0, t = 127;
  for (let i = 0; i < 256; i++) {
    wB += hist[i]; if (!wB) continue;
    const wF = g.length - wB; if (!wF) break;
    sB += i * hist[i];
    const mB = sB / wB, mF = (sum - sB) / wF, v = wB * wF * (mB - mF) * (mB - mF);
    if (v > best) { best = v; t = i; }
  }
  return (t + 0.5) / 255;
}

/** Sauvola 2000: T = m (1 + k (s / R − 1)) over a win×win window, via integral images. */
function sauvola(g: Float32Array, w: number, h: number, win = 25, k = 0.2, R = 0.5): Uint8Array {
  const W1 = w + 1, I = new Float64Array(W1 * (h + 1)), I2 = new Float64Array(W1 * (h + 1));
  for (let y = 0; y < h; y++) {
    let r = 0, r2 = 0;
    for (let x = 0; x < w; x++) {
      const v = g[y * w + x]; r += v; r2 += v * v;
      I[(y + 1) * W1 + x + 1] = I[y * W1 + x + 1] + r; I2[(y + 1) * W1 + x + 1] = I2[y * W1 + x + 1] + r2;
    }
  }
  const m = new Uint8Array(w * h), hw = win >> 1;
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    const x0 = Math.max(0, x - hw), x1 = Math.min(w, x + hw + 1), y0 = Math.max(0, y - hw), y1 = Math.min(h, y + hw + 1), n = (x1 - x0) * (y1 - y0);
    const s = I[y1 * W1 + x1] - I[y0 * W1 + x1] - I[y1 * W1 + x0] + I[y0 * W1 + x0];
    const s2 = I2[y1 * W1 + x1] - I2[y0 * W1 + x1] - I2[y1 * W1 + x0] + I2[y0 * W1 + x0];
    const mean = s / n, sd = Math.sqrt(Math.max(0, s2 / n - mean * mean));
    m[y * w + x] = g[y * w + x] < mean * (1 + k * (sd / R - 1)) ? 1 : 0;
  }
  return m;
}

/** Does the background brightness vary (photos, uneven light)? Std of 32 px block maxima. */
function backgroundVaries(g: Float32Array, w: number, h: number): boolean {
  const B = 32, mx: number[] = [];
  for (let by = 0; by < h; by += B) for (let bx = 0; bx < w; bx += B) {
    let m = 0;
    for (let y = by; y < Math.min(h, by + B); y++) for (let x = bx; x < Math.min(w, bx + B); x++) if (g[y * w + x] > m) m = g[y * w + x];
    mx.push(m);
  }
  const mean = mx.reduce((a, b) => a + b, 0) / mx.length;
  return Math.sqrt(mx.reduce((a, b) => a + (b - mean) * (b - mean), 0) / mx.length) > 0.1;
}

/** 3×3 majority filter: removes salt-and-pepper specks and pinholes, keeps strokes >= 2 px. */
function majority(m: Uint8Array, w: number, h: number): Uint8Array {
  const o = new Uint8Array(w * h);
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    let c = 0;
    for (let dy = -1; dy <= 1; dy++) { const yy = y + dy; if (yy < 0 || yy >= h) continue; for (let dx = -1; dx <= 1; dx++) { const xx = x + dx; if (xx >= 0 && xx < w) c += m[yy * w + xx]; } }
    o[y * w + x] = c >= 5 ? 1 : 0;
  }
  return o;
}

/** Steps 1–2: grey, downscale to <= 1000 px, threshold (Otsu, or Sauvola when the background varies), despeckle. */
export function prepare(img: RGBAImage, opts: { threshold?: number; method?: Method } = {}): Prepared {
  const { g, w, h } = downscale(toGrey(img), img.width, img.height, 1000);
  let mask: Uint8Array, method: Prepared['method'];
  if (opts.threshold !== undefined) {
    const t = opts.threshold; mask = new Uint8Array(w * h); for (let i = 0; i < mask.length; i++) mask[i] = g[i] < t ? 1 : 0; method = 'fixed';
  } else if (opts.method === 'sauvola' || ((opts.method ?? 'auto') === 'auto' && backgroundVaries(g, w, h))) { mask = sauvola(g, w, h); method = 'sauvola'; }
  else { const t = otsu(g); mask = new Uint8Array(w * h); for (let i = 0; i < mask.length; i++) mask[i] = g[i] < t ? 1 : 0; method = 'otsu'; }
  return { mask: majority(mask, w, h), width: w, height: h, method };
}
