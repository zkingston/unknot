// Test helpers: minimal PNG decoder (8-bit grey/GA/RGB/RGBA, filters 0–4) and a thick-polyline rasteriser.
import { readFileSync } from 'node:fs';
import { inflateSync } from 'node:zlib';

export interface RGBA { width: number; height: number; data: Uint8ClampedArray }

function decodePNG(buf: Uint8Array): RGBA {
  const dv = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);
  let o = 8, w = 0, h = 0, depth = 0, ct = 0;
  const idat: Uint8Array[] = [];
  while (o < buf.length) {
    const len = dv.getUint32(o), type = String.fromCharCode(...buf.subarray(o + 4, o + 8)), body = buf.subarray(o + 8, o + 8 + len);
    if (type === 'IHDR') { w = dv.getUint32(o + 8); h = dv.getUint32(o + 12); depth = body[8]; ct = body[9]; if (body[12]) throw new Error('interlaced PNG'); }
    else if (type === 'IDAT') idat.push(body);
    else if (type === 'IEND') break;
    o += 12 + len;
  }
  if (depth !== 8) throw new Error(`bit depth ${depth}`);
  const ch = ({ 0: 1, 2: 3, 4: 2, 6: 4 } as Record<number, number>)[ct];
  if (!ch) throw new Error(`colour type ${ct}`);
  const all = new Uint8Array(idat.reduce((s, b) => s + b.length, 0));
  let p = 0; for (const b of idat) { all.set(b, p); p += b.length; }
  const raw = inflateSync(all), stride = w * ch, px = new Uint8Array(stride * h);
  for (let y = 0; y < h; y++) {
    const f = raw[y * (stride + 1)], src = raw.subarray(y * (stride + 1) + 1, (y + 1) * (stride + 1)), row = px.subarray(y * stride, (y + 1) * stride);
    for (let x = 0; x < stride; x++) {
      const a = x >= ch ? row[x - ch] : 0, b = y ? px[(y - 1) * stride + x] : 0, c = y && x >= ch ? px[(y - 1) * stride + x - ch] : 0;
      let v = src[x];
      if (f === 1) v += a; else if (f === 2) v += b; else if (f === 3) v += (a + b) >> 1;
      else if (f === 4) { const pp = a + b - c, pa = Math.abs(pp - a), pb = Math.abs(pp - b), pc = Math.abs(pp - c); v += pa <= pb && pa <= pc ? a : pb <= pc ? b : c; }
      row[x] = v & 255;
    }
  }
  const data = new Uint8ClampedArray(4 * w * h);
  for (let i = 0; i < w * h; i++) {
    const s = px.subarray(i * ch, i * ch + ch);
    if (ch === 1) data.set([s[0], s[0], s[0], 255], 4 * i);
    else if (ch === 2) data.set([s[0], s[0], s[0], s[1]], 4 * i);
    else if (ch === 3) data.set([s[0], s[1], s[2], 255], 4 * i);
    else data.set(s, 4 * i);
  }
  return { width: w, height: h, data };
}

export const loadPNG = (rel: string) => decodePNG(readFileSync(new URL(rel, import.meta.url)));

/** White canvas with black round-capped polylines of width `pen` (antialiased by distance coverage). */
export function rasterise(w: number, h: number, lines: number[][], pen: number): RGBA {
  const ink = new Float32Array(w * h), r = pen / 2;
  for (const L of lines) for (let k = 0; k + 3 < L.length; k += 2) {
    const ax = L[k], ay = L[k + 1], bx = L[k + 2], by = L[k + 3];
    const x0 = Math.max(0, Math.floor(Math.min(ax, bx) - r - 1)), x1 = Math.min(w - 1, Math.ceil(Math.max(ax, bx) + r + 1));
    const y0 = Math.max(0, Math.floor(Math.min(ay, by) - r - 1)), y1 = Math.min(h - 1, Math.ceil(Math.max(ay, by) + r + 1));
    const dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy;
    for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
      const px = x + 0.5, py = y + 0.5;
      let t = l2 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0; t = t < 0 ? 0 : t > 1 ? 1 : t;
      const d = Math.hypot(px - ax - t * dx, py - ay - t * dy), c = Math.min(1, Math.max(0, r + 0.5 - d));
      if (c > ink[y * w + x]) ink[y * w + x] = c;
    }
  }
  const data = new Uint8ClampedArray(4 * w * h);
  for (let i = 0; i < w * h; i++) { const v = Math.round(255 * (1 - ink[i])); data[4 * i] = data[4 * i + 1] = data[4 * i + 2] = v; data[4 * i + 3] = 255; }
  return { width: w, height: h, data };
}

/** Set a fraction of pixels to pure black or white (seeded). */
export function saltPepper(img: RGBA, frac: number, r: () => number): RGBA {
  const d = img.data;
  for (let i = 0; i < img.width * img.height; i++) if (r() < frac) { const v = r() < 0.5 ? 0 : 255; d[4 * i] = d[4 * i + 1] = d[4 * i + 2] = v; }
  return img;
}

/** "M x y L x y ... Z" path data -> polylines. */
export function parsePaths(ds: string[]): number[][] {
  return ds.map((d) => {
    const nums = d.match(/-?\d+(\.\d+)?(e-?\d+)?/g)!.map(Number);
    if (d.trim().endsWith('Z')) nums.push(nums[0], nums[1]);
    return nums;
  });
}
