// PD code from a Diagram (Knot Atlas convention): X[i,j,k,l] counter-clockwise from the incoming under-edge i;
// k is the outgoing under-edge; positive crossings have the over-strand running l -> j.
import { type Diagram, events, crossingSign } from '../diagram/types';

export type PD = [number, number, number, number][];

export interface PDResult { pd: PD; signs: (1 | -1)[] }

/**
 * Walk each component in point order, labelling edges between consecutive crossing events and continuing the
 * numbering from one component to the next (0-based here; show 1-based). `comps` restricts to some components
 * and `selfOnly` to their self-crossings (per-component invariants of split links).
 */
export function pdCode(d: Diagram, comps = d.comps.map((_, c) => c), selfOnly = false): PDResult {
  const inc = new Map<string, number>(), out = new Map<string, number>();
  const use = new Set<number>();
  let base = 0;
  for (const c of comps) {
    const ev = events(d, c, selfOnly);
    const m = ev.length;
    ev.forEach((e, k) => {
      const tag = `${e.x}:${e.over ? 'o' : 'u'}`;
      inc.set(tag, base + ((k + m - 1) % m));
      out.set(tag, base + k);
      use.add(e.x);
    });
    base += m;
  }
  const pd: PD = [], signs: (1 | -1)[] = [];
  for (const x of [...use].sort((a, b) => a - b)) {
    const s = crossingSign(d, d.crossings[x]);
    const i = inc.get(`${x}:u`)!, k = out.get(`${x}:u`)!, oi = inc.get(`${x}:o`)!, oo = out.get(`${x}:o`)!;
    pd.push(s > 0 ? [i, oo, k, oi] : [i, oi, k, oo]);
    signs.push(s);
  }
  return { pd, signs };
}

export const formatPD = (pd: PD) => pd.map((X) => `X[${X.map((a) => a + 1).join(',')}]`).join(', ');

/** Gauss code: per component, crossing labels in order of first appearance, + over, − under; then signs by label. */
export function gaussCode(d: Diagram): string {
  const label = new Map<number, number>(), parts: string[] = [];
  d.comps.forEach((_, c) => {
    const ev = events(d, c);
    if (!ev.length) return;
    parts.push(ev.map((e) => { if (!label.has(e.x)) label.set(e.x, label.size + 1); return `${e.over ? '' : '−'}${label.get(e.x)}`; }).join(', '));
  });
  if (!label.size) return '';
  const signs = [...label.entries()].sort((a, b) => a[1] - b[1]).map(([x]) => (crossingSign(d, d.crossings[x]) > 0 ? '+' : '−'));
  return `${parts.join(' | ')}; ${signs.join(',')}`;
}
