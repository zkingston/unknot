// Laurent polynomials with BigInt coefficients: c[i] is the coefficient of t^(i + lo).
export interface Laurent { lo: number; c: bigint[] }

export const L = (lo: number, c: (number | bigint)[]): Laurent => trim({ lo, c: c.map(BigInt) });
export const ONE: Laurent = { lo: 0, c: [1n] };

export function trim(p: Laurent): Laurent {
  let a = 0, b = p.c.length;
  while (a < b && p.c[a] === 0n) a++;
  while (b > a && p.c[b - 1] === 0n) b--;
  return a === b ? { lo: 0, c: [] } : { lo: p.lo + a, c: p.c.slice(a, b) };
}
export const isZero = (p: Laurent) => trim(p).c.length === 0;
export function add(p: Laurent, q: Laurent, s = 1n): Laurent {
  if (!p.c.length) return trim({ lo: q.lo, c: q.c.map((x) => s * x) });
  if (!q.c.length) return p;
  const lo = Math.min(p.lo, q.lo), hi = Math.max(p.lo + p.c.length, q.lo + q.c.length), c = new Array<bigint>(hi - lo).fill(0n);
  p.c.forEach((x, i) => { c[i + p.lo - lo] += x; });
  q.c.forEach((x, i) => { c[i + q.lo - lo] += s * x; });
  return trim({ lo, c });
}
export function mul(p: Laurent, q: Laurent): Laurent {
  if (!p.c.length || !q.c.length) return { lo: 0, c: [] };
  const c = new Array<bigint>(p.c.length + q.c.length - 1).fill(0n);
  for (let i = 0; i < p.c.length; i++) if (p.c[i]) for (let j = 0; j < q.c.length; j++) c[i + j] += p.c[i] * q.c[j];
  return trim({ lo: p.lo + q.lo, c });
}
export const scale = (p: Laurent, s: bigint): Laurent => trim({ lo: p.lo, c: p.c.map((x) => x * s) });
export const shift = (p: Laurent, k: number): Laurent => ({ lo: p.lo + k, c: p.c });
export function pow(p: Laurent, k: number): Laurent { let r = ONE; for (let i = 0; i < k; i++) r = mul(r, p); return r; }
export const eq = (p: Laurent, q: Laurent) => isZero(add(p, q, -1n));
export const isOne = (p: Laurent) => eq(p, ONE);
/** t -> t^-1 */
export const mirror = (p: Laurent): Laurent => trim({ lo: -(p.lo + p.c.length - 1), c: [...p.c].reverse() });
/** Value at t = 1. */
export const at1 = (p: Laurent) => p.c.reduce((s, x) => s + x, 0n);

/** Exact division of polynomials (lo >= 0 assumed for both, as plain polynomials). Throws if not exact. */
export function divExact(p: Laurent, q: Laurent): Laurent {
  p = trim(p); q = trim(q);
  if (!p.c.length) return p;
  const r = [...p.c], qc = q.c, dq = qc.length - 1, lead = qc[dq], out = new Array<bigint>(Math.max(0, r.length - dq)).fill(0n);
  for (let i = r.length - 1; i >= dq; i--) {
    if (r[i] === 0n) continue;
    if (r[i] % lead !== 0n) throw new Error('inexact division');
    const f = r[i] / lead;
    out[i - dq] = f;
    for (let j = 0; j <= dq; j++) r[i - dq + j] -= f * qc[j];
  }
  if (r.some((x) => x !== 0n)) throw new Error('inexact division');
  return trim({ lo: p.lo - q.lo, c: out });
}

const SUP: Record<string, string> = { '-': '⁻', 0: '⁰', 1: '¹', 2: '²', 3: '³', 4: '⁴', 5: '⁵', 6: '⁶', 7: '⁷', 8: '⁸', 9: '⁹' };
const sup = (k: number) => String(k).split('').map((ch) => SUP[ch]).join('');

/** Descending powers, U+2212 minus, superscript exponents: "t − 1 + t⁻¹". */
export function format(p: Laurent, v = 't'): string {
  p = trim(p);
  if (!p.c.length) return '0';
  const parts: string[] = [];
  for (let i = p.c.length - 1; i >= 0; i--) {
    const a = p.c[i];
    if (a === 0n) continue;
    const e = i + p.lo, neg = a < 0n, m = neg ? -a : a;
    const mono = e === 0 ? `${m}` : `${m === 1n ? '' : m}${v}${e === 1 ? '' : sup(e)}`;
    parts.push(parts.length ? `${neg ? ' − ' : ' + '}${mono}` : `${neg ? '−' : ''}${mono}`);
  }
  return parts.join('');
}
