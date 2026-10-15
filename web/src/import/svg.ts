// SVG import (main thread: workers have no DOM): sample every geometry element with getPointAtLength,
// apply its CTM, split subpaths, and hand the arcs to the image pipeline from step 6.
import type { Arc } from './image/trace';

export interface SampledSVG { arcs: Arc[]; sw: number; filled: boolean }

export function sampleSVG(text: string): SampledSVG {
  const doc = new DOMParser().parseFromString(text, 'image/svg+xml');
  if (doc.querySelector('parsererror') || doc.documentElement.nodeName.toLowerCase() !== 'svg') throw new Error('not an SVG file');
  const host = document.createElement('div');
  host.style.cssText = 'position:absolute;left:-10000px;top:0;width:1000px;height:1000px;visibility:hidden;pointer-events:none';
  const root = document.importNode(doc.documentElement, true) as unknown as SVGSVGElement;
  host.append(root); document.body.append(host);
  try {
    const els = [...root.querySelectorAll<SVGGeometryElement>('path, line, polyline, polygon, circle, ellipse, rect')];
    // pass 1: extent from coarse samples
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity, stroked = 0, filled = 0, swSum = 0;
    const geo = els.map((el) => {
      const L = el.getTotalLength?.() ?? 0, m = el.getCTM();
      if (!(L > 0) || !m) return null;
      const cs = getComputedStyle(el), sw = parseFloat(cs.strokeWidth) || 0, hasStroke = cs.stroke !== 'none' && sw > 0;
      const scale = Math.sqrt(Math.abs(m.a * m.d - m.b * m.c));
      if (hasStroke) { stroked++; swSum += sw * scale; } else if (cs.fill !== 'none') filled++;
      for (let k = 0; k <= 200; k++) { const p = el.getPointAtLength((L * k) / 200).matrixTransform(m); x0 = Math.min(x0, p.x); x1 = Math.max(x1, p.x); y0 = Math.min(y0, p.y); y1 = Math.max(y1, p.y); }
      return { el, L, m, scale, closedTag: /^(polygon|circle|ellipse|rect)$/i.test(el.tagName) || /z\s*$/i.test(el.getAttribute('d') ?? '') };
    });
    const ext = Math.max(x1 - x0, y1 - y0);
    if (!(ext > 0)) throw new Error('the SVG has no drawable geometry');
    const sw = stroked ? swSum / stroked : 0.01 * ext, step = ext / 1000;
    const arcs: Arc[] = [];
    for (const g of geo) {
      if (!g) continue;
      const n = Math.max(8, Math.ceil(g.L / (step * g.scale || 1)));
      let cur: number[] = [], px = NaN, py = NaN;
      const flush = (closedHint: boolean) => {
        if (cur.length >= 6) {
          const k = cur.length, closed = closedHint || Math.hypot(cur[0] - cur[k - 2], cur[1] - cur[k - 1]) < 0.5 * sw;
          if (closed && Math.hypot(cur[0] - cur[k - 2], cur[1] - cur[k - 1]) < 0.5 * step) cur.length -= 2;
          arcs.push({ xy: Float64Array.from(cur), closed });
        }
        cur = [];
      };
      for (let k = 0; k <= n; k++) {
        const p = g.el.getPointAtLength((g.L * k) / n).matrixTransform(g.m);
        // a jump between samples is a move-to: start a new subpath
        if (cur.length && Math.hypot(p.x - px, p.y - py) > 3 * step + 1e-9 * ext) flush(false);
        cur.push(p.x, -p.y); px = p.x; py = p.y;
      }
      flush(g.closedTag);
    }
    return { arcs, sw, filled: filled > 0 && stroked === 0 };
  } finally { host.remove(); }
}

/** Does the middle 20–80% of the segment cross any arc? (The image pipeline's "crosses ink" for vectors.) */
export function polylineInk(arcs: Arc[]) {
  return (x0: number, y0: number, x1: number, y1: number) => {
    const ax = x0 + 0.2 * (x1 - x0), ay = y0 + 0.2 * (y1 - y0), bx = x0 + 0.8 * (x1 - x0), by = y0 + 0.8 * (y1 - y0);
    const rx = bx - ax, ry = by - ay;
    for (const A of arcs) {
      const n = A.xy.length / 2, m = A.closed ? n : n - 1;
      for (let i = 0; i < m; i++) {
        const j = (i + 1) % n, cx = A.xy[2 * i], cy = A.xy[2 * i + 1], sx = A.xy[2 * j] - cx, sy = A.xy[2 * j + 1] - cy;
        const den = rx * sy - ry * sx; if (den === 0) continue;
        const qx = cx - ax, qy = cy - ay, t = (qx * sy - qy * sx) / den, u = (qx * ry - qy * rx) / den;
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) return true;
      }
    }
    return false;
  };
}
