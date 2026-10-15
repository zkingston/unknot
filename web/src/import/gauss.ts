// Gauss codes: signed labels in traversal order (+ over, − under), e.g. `1, -2, 3, -1, 2, -3`, optionally
// `; +,+,+` with one crossing sign per label in ascending label order. Components split by `|` or newlines.
import type { PD } from '../invariants/pd';
import { isPlanar, pdStructure } from './pdcode';

const GAUSS_UNSIGNED_MAX = 20;

const norm = (s: string) => s.replace(/[−–—]/g, '-');

export function parseGauss(text: string): { pd: PD; chiralityUndetermined: boolean } {
  const [code, signPart, extra] = norm(text).split(';');
  if (extra !== undefined) throw new Error('Gauss code: at most one “;” (before the crossing signs)');
  const comps = code.split(/[|\n]/).map((s) => s.trim()).filter(Boolean).map((s) => {
    const toks = s.split(/[\s,]+/).filter(Boolean);
    return toks.map((t) => { if (!/^[+-]?\d+$/.test(t)) throw new Error(`Gauss code: “${t}” is not a signed label`); return +t; });
  });
  if (!comps.length || !comps.some((c) => c.length)) throw new Error('Gauss code: no crossings');
  const count = new Map<number, [number, number]>();
  for (const c of comps) for (const x of c) {
    if (x === 0) throw new Error('Gauss code: labels must be non-zero');
    const k = count.get(Math.abs(x)) ?? [0, 0]; k[x > 0 ? 0 : 1]++; count.set(Math.abs(x), k);
  }
  for (const [x, [o, u]] of count) if (o !== 1 || u !== 1) throw new Error(`Gauss code: crossing ${x} must appear once over (+${x}) and once under (−${x})`);
  const labels = [...count.keys()].sort((a, b) => a - b), n = labels.length;
  if (signPart !== undefined) {
    const s = signPart.split(/[\s,]+/).filter(Boolean);
    if (s.length !== n || s.some((t) => t !== '+' && t !== '-')) throw new Error(`Gauss code: give ${n} crossing signs (+ or −) after “;”`);
    const pd = gaussPD(comps, labels, s.map((t) => (t === '+' ? 1 : -1)));
    pdStructure(pd);
    return { pd, chiralityUndetermined: false };
  }
  if (n > GAUSS_UNSIGNED_MAX) throw new Error(`Gauss code: above ${GAUSS_UNSIGNED_MAX} crossings, add the signs (“; +,−,…”)`);
  // try every rotation choice with the first crossing positive (the other half are mirror images)
  const signs = new Array<number>(n).fill(1);
  for (let m = 0; m < 1 << (n - 1); m++) {
    for (let k = 1; k < n; k++) signs[k] = (m >> (k - 1)) & 1 ? -1 : 1;
    const pd = gaussPD(comps, labels, signs);
    if (isPlanar(pd)) { pdStructure(pd); return { pd, chiralityUndetermined: true }; }
  }
  throw new Error('Gauss code: no planar diagram has this code');
}

/** Edges labelled 1, 2, … along each component in turn; edge k leaves position k. */
function gaussPD(comps: number[][], labels: number[], signs: number[]): PD {
  const where = new Map<number, { inc: number; out: number }>(); // key ±label
  let base = 0;
  for (const c of comps) {
    const m = c.length;
    c.forEach((x, k) => where.set(x, { inc: base + ((k + m - 1) % m) + 1, out: base + k + 1 }));
    base += m;
  }
  return labels.map((x, r) => {
    const u = where.get(-x)!, o = where.get(x)!;
    return signs[r] > 0 ? [u.inc, o.out, u.out, o.inc] : [u.inc, o.inc, u.out, o.out];
  });
}
