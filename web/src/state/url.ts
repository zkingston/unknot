// URL hash: #v=1&k=<payload>&s=<uint32>&view=<2d|3d>[&sticks=1], payload = base64url(deflate-raw(JSON {src, data, params?})).
import type { Diagram } from '../diagram/types';
import type { Scene } from '../kernel/scene';
import { findCrossings, validateDiagram } from '../diagram/crossings';
import { extent2 } from '../diagram/types';
import { touches } from '../kernel/energy';

export const HASH_LIMIT = 8192;

export type Payload =
  | { src: 'preset'; data: string; params?: Record<string, number> }
  | { src: 'pd' | 'gauss' | 'braid'; data: string; params?: Record<string, number> }
  | { src: 'draw' | 'image' | 'svg'; data: QuantDiagram; params?: Record<string, number> }
  | { src: 'json'; data: QuantScene; params?: Record<string, number> };

/** Curves on a 12-bit grid, delta-encoded per component; one over bit per crossing in canonical order. */
export interface QuantDiagram { o: [number, number]; s: number; c: number[][]; b: string }
/** xyz on a 16-bit grid over [-2, 2]. */
export interface QuantScene { c: number[][] }

const Q12 = 4095, Q16 = 65535, R16 = 2;

function quantizeDiagram(d: Diagram): QuantDiagram {
  const { x0, y0, ext } = extent2(d.comps), s = ext / Q12;
  // grid points with consecutive duplicates dropped; segMap[c][orig seg] = new seg containing it
  const segMap: Int32Array[] = [];
  const c = d.comps.map((P) => {
    const n = P.length / 2, qx: number[] = [], qy: number[] = [], first = new Int32Array(n);
    for (let i = 0; i < n; i++) {
      const x = Math.round((P[2 * i] - x0) / s), y = Math.round((P[2 * i + 1] - y0) / s);
      if (qx.length && qx[qx.length - 1] === x && qy[qy.length - 1] === y) { first[i] = qx.length - 1; continue; }
      qx.push(x); qy.push(y); first[i] = qx.length - 1;
    }
    let m = qx.length;
    while (m > 1 && qx[m - 1] === qx[0] && qy[m - 1] === qy[0]) { m--; for (let i = 0; i < n; i++) if (first[i] >= m) first[i] = 0; }
    // original segment i runs from point first[i]; it lies on new segment first[i] (or the one ending there)
    segMap.push(Int32Array.from(first, (f, i) => (first[(i + 1) % n] === f ? (f + m - 1) % m : f)));
    const out: number[] = [];
    let px = 0, py = 0;
    for (let i = 0; i < m; i++) { out.push(qx[i] - px, qy[i] - py); px = qx[i]; py = qy[i]; }
    return out;
  });
  const q: QuantDiagram = { o: [x0, y0], s, c, b: '' };
  // the over bit says whether strand a of each dequantised crossing is the source crossing's over strand
  const key = (a: { comp: number; seg: number }, b: { comp: number; seg: number }) => `${a.comp}:${a.seg}|${b.comp}:${b.seg}`;
  const map = (r: { comp: number; seg: number }) => ({ comp: r.comp, seg: segMap[r.comp][r.seg] });
  const over = new Map<string, boolean>();
  for (const C of d.crossings) { over.set(key(map(C.over), map(C.under)), true); over.set(key(map(C.under), map(C.over)), false); }
  const X = findCrossings(dequantizeCurves(q));
  if (X.length !== d.crossings.length) throw new Error('quantising changes the crossings');
  q.b = X.map((x) => {
    const o = over.get(key(x.a, x.b));
    if (o === undefined) throw new Error('quantising changes the crossings');
    return o ? '1' : '0';
  }).join('');
  return q;
}

function dequantizeCurves(q: QuantDiagram): Float64Array[] {
  return q.c.map((d) => {
    const P = new Float64Array(d.length);
    let qx = 0, qy = 0;
    for (let i = 0; i < d.length; i += 2) { qx += d[i]; qy += d[i + 1]; P[i] = q.o[0] + qx * q.s; P[i + 1] = q.o[1] + qy * q.s; }
    return P;
  });
}

export function dequantizeDiagram(q: QuantDiagram, source: Diagram['source']): Diagram {
  const comps = dequantizeCurves(q), X = findCrossings(comps);
  if (X.length !== q.b.length) throw new Error('link: crossing count changed');
  return { comps, crossings: X.map((x, k) => (q.b[k] === '1' ? { under: x.b, over: x.a } : { under: x.a, over: x.b })), source };
}

const quantizeScene = (S: Scene): QuantScene => ({ c: S.map((P) => Array.from(P, (x) => Math.round(((x + R16) / (2 * R16)) * Q16))) });
export const dequantizeScene = (q: QuantScene): Scene => q.c.map((c) => Float64Array.from(c, (v) => (v / Q16) * 2 * R16 - R16));

/** Snap a diagram to what a link reloads; null if the round trip changes its crossings. */
export function commitDiagram(d: Diagram): { diagram: Diagram; q: QuantDiagram } | null {
  try {
    const q = quantizeDiagram(d), r = dequantizeDiagram(q, d.source);
    if (r.crossings.length !== d.crossings.length || !validateDiagram(r).ok) return null;
    r.meta = d.meta;
    return { diagram: r, q };
  } catch { return null; }
}
export function commitScene(S: Scene): { scene: Scene; q: QuantScene } | null {
  const q = quantizeScene(S), r = dequantizeScene(q);
  return touches(r) ? null : { scene: r, q };
}

// ------------------------------------------------------------ encoding

const b64url = (u: Uint8Array) => { let s = ''; for (const b of u) s += String.fromCharCode(b); return btoa(s).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, ''); };
const unb64url = (s: string) => { const b = atob(s.replace(/-/g, '+').replace(/_/g, '/')); return Uint8Array.from(b, (c) => c.charCodeAt(0)); };

async function pipe(data: Uint8Array, stream: CompressionStream | DecompressionStream) {
  const out = new Response(new Blob([data as BlobPart]).stream().pipeThrough(stream));
  return new Uint8Array(await out.arrayBuffer());
}

export async function encodePayload(p: Payload): Promise<string> {
  return b64url(await pipe(new TextEncoder().encode(JSON.stringify(p)), new CompressionStream('deflate-raw')));
}
export async function decodePayload(s: string): Promise<Payload> {
  return JSON.parse(new TextDecoder().decode(await pipe(unb64url(s), new DecompressionStream('deflate-raw'))));
}

export interface HashState { k?: string; s: number; view: '2d' | '3d'; sticks?: boolean }
export function formatHash(h: HashState) { return `#v=1${h.k ? `&k=${h.k}` : ''}&s=${h.s >>> 0}&view=${h.view}${h.sticks ? '&sticks=1' : ''}`; }
export function parseHash(hash: string): HashState | null {
  const q = new URLSearchParams(hash.replace(/^#/, ''));
  if (q.get('v') !== '1') return null;
  const view = q.get('view'); // view=sticks: older links
  return { k: q.get('k') ?? undefined, s: Number(q.get('s') ?? 7) >>> 0, view: view === '3d' || view === 'sticks' ? '3d' : '2d', sticks: q.get('sticks') === '1' || view === 'sticks' };
}
