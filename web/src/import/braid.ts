// Braid words -> closed-braid Diagram. Vertical strands, one row per generator, nested closing arcs on the right.
import type { Diagram } from '../diagram/types';
import { diagramFromCurves, validateDiagram } from '../diagram/crossings';

export interface Braid { word: number[]; strands: number }

const SUP: Record<string, string> = { '⁻': '-', '⁰': '0', '¹': '1', '²': '2', '³': '3', '⁴': '4', '⁵': '5', '⁶': '6', '⁷': '7', '⁸': '8', '⁹': '9' };

/**
 * Accepts `s1 s2^-1 s1`, `σ1 σ2⁻¹ σ1`, KnotInfo `{1,-2,1}`, plain `1 -2 1`, or letters `aBa` (a = σ₁, A = σ₁⁻¹).
 * Exponents: `^2`, `^-1`, `^{-1}`, superscripts. An optional `N:` prefix gives the strand count; otherwise max index + 1.
 */
export function parseBraid(text: string): Braid {
  let t = text.trim().replace(/[⁻⁰¹²³⁴⁵⁶⁷⁸⁹]+/g, (m) => '^' + [...m].map((c) => SUP[c]).join('')).replace(/−/g, '-');
  let strands = 0;
  const pre = /^(\d+)\s*:\s*/.exec(t);
  if (pre) { strands = Number(pre[1]); t = t.slice(pre[0].length); }
  const word: number[] = [];
  const push = (g: number, e: number) => {
    if (!Number.isInteger(g) || g < 1) throw new Error(`bad generator ${g}`);
    if (!Number.isInteger(e) || e === 0 || Math.abs(e) > 64) throw new Error(`bad exponent ${e}`);
    for (let k = 0; k < Math.abs(e); k++) word.push(e > 0 ? g : -g);
  };
  const body = t.replace(/^\{|\}$/g, '').trim();
  if (/^[a-zA-Z]+$/.test(body)) {
    for (const ch of body) { const lo = ch.toLowerCase(); push(lo.charCodeAt(0) - 96, ch === lo ? 1 : -1); }
  } else {
    const re = /\s*(?:[sσ]_?)?(-?\d+)(?:\s*\^\s*\{?\s*(-?\d+)\s*\}?)?\s*,?/gy;
    let m: RegExpExecArray | null, pos = 0;
    while (pos < body.length && (m = re.exec(body))) {
      const g = Number(m[1]), e = m[2] !== undefined ? Number(m[2]) : 1;
      push(Math.abs(g), g < 0 ? -e : e);
      pos = re.lastIndex;
    }
    if (pos < body.length) throw new Error(`cannot read braid word near “${body.slice(pos, pos + 12)}”`);
  }
  if (!word.length && !strands) throw new Error('empty braid word');
  const need = Math.max(0, ...word.map((g) => Math.abs(g) + 1));
  if (strands && strands < need) throw new Error(`${strands} strands is too few for σ${need - 1}`);
  return { word, strands: strands || need };
}

/**
 * Closure of the braid. Strands run upward; generator k occupies rows y ∈ [k, k+1]. σᵢ swaps the strands at
 * positions i−1 and i (0-based); for a positive exponent the strand coming from the left (strand i) is over.
 * Convention check (tests): the closure of σ₁³ is the trefoil with writhe +3 and Jones t + t³ − t⁴.
 */
export function braidToDiagram({ word, strands: n }: Braid, sourceText?: string): Diagram {
  const m = word.length;
  const next = (q: number, k: number) => { const g = Math.abs(word[k]); return q === g - 1 ? g : q === g ? g - 1 : q; };
  const seen = new Uint8Array(n), comps: Float64Array[] = [];
  for (let start = 0; start < n; start++) {
    if (seen[start]) continue;
    const pts: number[] = [];
    let q = start;
    do {
      seen[q] = 1;
      for (let k = 0; k < m; k++) { pts.push(q, k); q = next(q, k); }
      // closing arc from the top of position q back down to its bottom, nested: rightmost innermost
      const h = 0.4 * (n - q), X = n - 1 + 0.6 * (n - q);
      pts.push(q, m, q, m + h, X, m + h, X, -h, q, -h);
    } while (q !== start);
    comps.push(Float64Array.from(pts));
  }
  const d = diagramFromCurves(comps, 'braid', (X) => {
    const k = Math.floor(X.y), A = comps[X.a.comp], i = X.a.seg, j = (i + 1) % (A.length / 2);
    const aFromLeft = A[2 * j] - A[2 * i] > 0; // strand a moves right as it goes up
    return word[k] > 0 ? aFromLeft : !aFromLeft;
  });
  d.meta = { sourceText };
  const v = validateDiagram(d);
  if (!v.ok) throw new Error(`braid layout is not regular: ${v.error}`);
  return d;
}
