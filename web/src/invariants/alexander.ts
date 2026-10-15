// Alexander polynomial from a PD code: Alexander matrix over Z[t], last row and column deleted, determinant.
import type { PD } from './pd';
import { type Laurent, L, ONE, add, mul, divExact, trim, isZero, scale, at1 } from './laurent';

export type AlexanderResult = { kind: 'exact'; poly: Laurent } | { kind: 'mod'; isOne: boolean };

const PRIMES = [33554393, 33554383, 33554371]; // largest three primes below 2^25

/** Arcs = strands between consecutive under-passages: labels j and l at a crossing lie on one arc. */
function alexanderMatrix(pd: PD, signs: (1 | -1)[]) {
  const labels = [...new Set(pd.flat())].sort((a, b) => a - b), id = new Map(labels.map((x, i) => [x, i]));
  const par = labels.map((_, i) => i), f = (i: number): number => (par[i] === i ? i : (par[i] = f(par[i])));
  for (const [, j, , l] of pd) par[f(id.get(j)!)] = f(id.get(l)!);
  const roots = [...new Set(labels.map((_, i) => f(i)))], arc = (x: number) => roots.indexOf(f(id.get(x)!));
  const n = pd.length;
  // entries as [constant, t-coefficient]
  const M = Array.from({ length: n }, () => Array.from({ length: roots.length }, () => [0, 0]));
  pd.forEach(([i, j, k], r) => {
    const o = M[r][arc(j)]; o[0] += 1; o[1] -= 1;
    const a = M[r][arc(i)], b = M[r][arc(k)];
    if (signs[r] > 0) { a[1] += 1; b[0] -= 1; } else { a[0] -= 1; b[1] += 1; }
  });
  return M;
}

/** Fraction-free Bareiss determinant over Z[t]. */
function bareiss(A: Laurent[][]): Laurent {
  const m = A.length;
  if (!m) return ONE;
  let sign = 1n, prev = ONE;
  for (let k = 0; k < m - 1; k++) {
    if (isZero(A[k][k])) {
      const r = A.findIndex((row, i) => i > k && !isZero(row[k]));
      if (r < 0) return { lo: 0, c: [] };
      [A[k], A[r]] = [A[r], A[k]]; sign = -sign;
    }
    for (let i = k + 1; i < m; i++) for (let j = k + 1; j < m; j++)
      A[i][j] = divExact(add(mul(A[k][k], A[i][j]), mul(A[i][k], A[k][j]), -1n), prev);
    prev = A[k][k];
  }
  return scale(A[m - 1][m - 1], sign);
}

/** Symmetric form with Δ(1) = 1 (up to ±t^k normalisation). */
function normalizeAlexander(D: Laurent): Laurent {
  D = trim(D);
  if (!D.c.length) return D;
  const hi = D.lo + D.c.length - 1, mid = (D.lo + hi) / 2;
  const sh = Math.floor(mid);
  let r: Laurent = { lo: D.lo - sh, c: D.c };
  if (at1(r) < 0n) r = scale(r, -1n);
  return r;
}

const modpow = (b: number, e: number, p: number) => { let r = 1; b %= p; while (e > 0) { if (e & 1) r = (r * b) % p; b = (b * b) % p; e >>= 1; } return r; };
// products of residues < 2^25 stay below 2^50, exact in doubles
const mulmod = (a: number, b: number, p: number) => (a * b) % p;

function detMod(M: number[][], p: number): number {
  const m = M.length; let det = 1;
  for (let k = 0; k < m; k++) {
    let r = k; while (r < m && M[r][k] === 0) r++;
    if (r === m) return 0;
    if (r !== k) { [M[k], M[r]] = [M[r], M[k]]; det = p - det; }
    det = mulmod(det, M[k][k], p);
    const inv = modpow(M[k][k], p - 2, p);
    for (let i = k + 1; i < m; i++) {
      if (!M[i][k]) continue;
      const f = mulmod(M[i][k], inv, p);
      for (let j = k; j < m; j++) M[i][j] = (M[i][j] - mulmod(f, M[k][j], p) + p) % p;
    }
  }
  return det % p;
}

/** Points where det M is evaluated (fixed, so results are reproducible). */
const POINTS = [2654435, 9876543, 31415926];

/**
 * Is det(M(t)) = ±t^k mod p? A unit ±t^k takes the value ±x^k at every x, for one sign and one k <= m. Any other
 * polynomial of degree <= m agrees with a given ±t^k at no more than m of the p points, so a (sign, k) that fits
 * det M at three points settles it (Schwartz–Zippel) with three determinants instead of m + 1 for interpolation.
 */
function isUnitMod(E: number[][][], p: number, progress?: (f: number) => void): boolean {
  const m = E.length;
  let fits: [number, number][] = [];   // (sign, k)
  POINTS.forEach((x, q) => {
    const d = detMod(E.map((row) => row.map(([a, b]) => (((a + b * x) % p) + p) % p)), p);
    if (q === 0) for (let k = 0, xk = 1; k <= m; k++, xk = mulmod(xk, x, p)) { if (xk === d) fits.push([1, k]); if (p - xk === d) fits.push([-1, k]); }
    else fits = fits.filter(([s, k]) => { const xk = modpow(x, k, p); return (s > 0 ? xk : (p - xk) % p) === d; });
    progress?.((q + 1) / POINTS.length);
  });
  return fits.length > 0;
}

/** Exact while the minor has at most `exactMax` rows (crossings − 1; Bareiss over Z[t]); beyond, only whether Δ = 1 mod three primes. */
export function alexander(pd: PD, signs: (1 | -1)[], progress?: (f: number) => void, exactMax = 40): AlexanderResult {
  if (pd.length <= 1) return { kind: 'exact', poly: ONE };
  const M = alexanderMatrix(pd, signs);
  const minor = M.slice(0, -1).map((row) => row.slice(0, -1));
  if (minor.length <= exactMax) return { kind: 'exact', poly: normalizeAlexander(bareiss(minor.map((row) => row.map(([a, b]) => L(0, [a, b]))))) };
  const one = PRIMES.every((p, k) => isUnitMod(minor, p, progress && ((f) => progress((k + f) / 3))));
  return { kind: 'mod', isOne: one };
}
