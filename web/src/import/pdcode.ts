// PD codes: Knot Atlas `X[1,5,2,4], X[3,1,4,6], ...` or KnotInfo `[[1,5,2,4],[3,1,4,6],...]`.
// Labels are kept as typed (both sources are 1-based). X[i,j,k,l] runs counter-clockwise from the incoming
// under-edge i; k is the outgoing under-edge; the over-strand runs l -> j iff j follows l along its component.
import type { PD } from '../invariants/pd';

export interface Pass { c: number; sIn: number; sOut: number }            // a strand passing crossing c from slot sIn to sOut
export interface PDComponent { passes: Pass[]; edges: number[] }          // edges[k] = label leaving passes[k]
export interface PDStructure { comps: PDComponent[]; signs: (1 | -1)[]; parts: number[] }

const QUAD = /(?:X\s*)?\[\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*\]/g;

export function parsePD(text: string): PD {
  const pd: PD = [];
  for (const m of text.matchAll(QUAD)) pd.push([+m[1], +m[2], +m[3], +m[4]]);
  const rest = text.replace(QUAD, '').replace(/PD|[\s,[\]()]/gi, '');
  if (rest) throw new Error(`unrecognised PD code near “${rest.slice(0, 12)}”`);
  if (!pd.length) throw new Error('no crossings: write X[i,j,k,l], … or [[i,j,k,l], …]');
  pdStructure(pd); // validates
  return pd;
}

/** Faces of the rotation system: leave crossing c by slot s, arrive at (c', t), leave next by slot t − 1. */
function faceCount(pd: PD, other: Int32Array): number {
  const seen = new Uint8Array(4 * pd.length);
  let faces = 0;
  for (let d0 = 0; d0 < seen.length; d0++) {
    if (seen[d0]) continue;
    faces++;
    for (let d = d0; !seen[d]; ) { seen[d] = 1; const t = other[d]; d = 4 * (t >> 2) + (((t & 3) + 3) & 3); }
  }
  return faces;
}

/** other[4c + s] = the slot index (4c' + s') holding the same label. */
function occurrences(pd: PD): Int32Array {
  const at = new Map<number, number[]>();
  pd.forEach((X, c) => X.forEach((e, s) => { const l = at.get(e); if (l) l.push(4 * c + s); else at.set(e, [4 * c + s]); }));
  const other = new Int32Array(4 * pd.length);
  for (const [e, l] of at) {
    if (l.length !== 2) throw new Error(`label ${e} appears ${l.length} time${l.length === 1 ? '' : 's'}; each must appear exactly twice`);
    other[l[0]] = l[1]; other[l[1]] = l[0];
  }
  return other;
}

/** Connected part of the 4-valent graph per crossing, faces, and the face count a planar diagram has. */
function planarity(pd: PD, other = occurrences(pd)) {
  const par = pd.map((_, c) => c), f = (c: number): number => (par[c] === c ? c : (par[c] = f(par[c])));
  for (let d = 0; d < 4 * pd.length; d++) par[f(d >> 2)] = f(other[d] >> 2);
  const parts = pd.map((_, c) => f(c));
  return { parts, faces: faceCount(pd, other), want: pd.length + 2 * new Set(parts).size };
}

export const isPlanar = (pd: PD) => { const p = planarity(pd); return p.faces === p.want; };

/** Components (strand traversals), crossing signs, connected parts; throws on malformed or non-planar codes. */
export function pdStructure(pd: PD): PDStructure {
  const n = pd.length, other = occurrences(pd), { parts, faces, want } = planarity(pd, other);
  if (faces !== want) throw new Error(`not planar: ${faces} faces, a planar diagram has ${want}`);
  // strands: enter slot s, leave by s + 2, follow the label to its other slot; start at under-entries (slot 0)
  const used = new Uint8Array(4 * n), comps: PDComponent[] = [];
  const starts = [...Array(n).keys()].map((c) => 4 * c).concat([...Array(n).keys()].flatMap((c) => [4 * c + 1, 4 * c + 3]));
  for (const s0 of starts) {
    if (used[s0]) continue;
    const passes: Pass[] = [], edges: number[] = [];
    for (let d = s0; !used[d]; ) {
      const c = d >> 2, sIn = d & 3, sOut = (sIn + 2) & 3;
      used[d] = used[4 * c + sOut] = 1;
      passes.push({ c, sIn, sOut }); edges.push(pd[c][sOut]);
      d = other[4 * c + sOut];
    }
    if (passes.some((p) => p.sIn === 2)) {
      if (passes.some((p) => p.sIn === 0)) throw new Error('labels do not follow the strands: an under-strand must run from i to k');
      reverse(passes, edges, pd);
    } else if (!passes.some((p) => p.sIn === 0)) {
      // only over-passes: orient so labels increase along the strand
      let up = 0;
      for (let k = 0; k < edges.length; k++) { const a = edges[k], b = edges[(k + 1) % edges.length]; up += b === a + 1 ? 1 : b === a - 1 ? -1 : 0; }
      if (up < 0) reverse(passes, edges, pd);
    }
    comps.push({ passes, edges });
  }
  const signs = new Array<1 | -1>(n);
  for (const C of comps) for (const p of C.passes) if (p.sIn & 1) signs[p.c] = p.sIn === 3 ? 1 : -1;
  return { comps, signs, parts };
}

function reverse(passes: Pass[], edges: number[], pd: PD) {
  passes.reverse();
  for (const p of passes) [p.sIn, p.sOut] = [p.sOut, p.sIn];
  edges.length = 0;
  for (const p of passes) edges.push(pd[p.c][p.sOut]);
}

/** Crossing signs: positive iff the over-strand runs l -> j. */
export const pdSigns = (pd: PD) => pdStructure(pd).signs;
export const pdComponents = (pd: PD) => pdStructure(pd).comps;
