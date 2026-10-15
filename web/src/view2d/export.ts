// Export the current 2D view as a standalone SVG, or as a PNG rendered from that SVG at 2×.
import { PEN } from './render';

export function standaloneSVG(paths: string[], w: number, h: number, ink: string, paper: string | null): string {
  const bg = paper ? `<rect width="100%" height="100%" fill="${paper}"/>` : '';
  const body = paths.map((d) => `<path d="${d}"/>`).join('');
  return `<?xml version="1.0" encoding="UTF-8"?>\n<svg xmlns="http://www.w3.org/2000/svg" width="${w}" height="${h}" viewBox="0 0 ${w} ${h}">${bg}<g fill="none" stroke="${ink}" stroke-width="${PEN}" stroke-linecap="round" stroke-linejoin="round">${body}</g></svg>\n`;
}

export function pngFromSVG(svg: string, w: number, h: number, scale = 2): Promise<Blob> {
  return new Promise((res, rej) => {
    const img = new Image();
    img.onload = () => {
      const c = document.createElement('canvas');
      c.width = Math.round(w * scale); c.height = Math.round(h * scale);
      const g = c.getContext('2d')!;
      g.scale(scale, scale);
      g.drawImage(img, 0, 0, w, h);
      c.toBlob((b) => (b ? res(b) : rej(new Error('png encoding failed'))), 'image/png');
    };
    img.onerror = () => rej(new Error('svg rasterisation failed'));
    img.src = 'data:image/svg+xml;charset=utf-8,' + encodeURIComponent(svg);
  });
}

export function download(name: string, data: Blob | string, type = 'application/json') {
  const blob = typeof data === 'string' ? new Blob([data], { type }) : data;
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob); a.download = name;
  document.body.append(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
