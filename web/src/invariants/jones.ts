// Jones polynomial via the Kauffman bracket, summed crossing by crossing: a state is the pairing of the open edge
// labels (how the smoothed strands processed so far connect them) with its polynomial, so the work grows with the
// number of pairings on the frontier rather than with 2^n.
import type { PD } from './pd';
import { type Laurent, L, ONE, add, mul, shift, scale, trim } from './laurent';

/** Frontier states allowed; beyond this the polynomial is skipped. */
const JONES_STATES = 1 << 13;

const DELTA = L(-2, [-1, 0, 0, 0, -1]); // −A⁻² − A²

/** Join edge labels x and y by a smoothing arc; returns the number of loops closed (0 or 1). */
function join(P: Map<number, number>, x: number, y: number): number {
  if (x === y) return 1;
  const px = P.get(x), py = P.get(y);
  if (px === y) { P.delete(x); P.delete(y); return 1; }
  const u = px ?? x, v = py ?? y;
  if (px !== undefined) P.delete(x);
  if (py !== undefined) P.delete(y);
  P.set(u, v); P.set(v, u);
  return 0;
}

const keyOf = (P: Map<number, number>) => { const k: number[] = []; for (const [a, b] of P) if (a < b) k.push(a, b); return k.join(','); };

/** p / δ for δ = −A⁻² − A² (exact): q at e + 2 is −p at e − q at e − 2. */
function divDelta(p: Laurent): Laurent {
  const lo = p.lo + 2, c: bigint[] = [];
  for (let i = 0; i + 4 < p.c.length; i++) c.push(-p.c[i] - (i >= 4 ? c[i - 4] : 0n));
  return trim({ lo, c });
}

/** Kauffman bracket ⟨D⟩ (⟨O⟩ = 1), or null when the frontier outgrows JONES_STATES. */
export function bracket(pd: PD, limit = JONES_STATES): Laurent | null {
  const n = pd.length;
  if (!n) return ONE;
  // crossing order: greedily the one with most labels already open, so the frontier stays small
  const seen = new Map<number, number>(), done = new Uint8Array(n), order: number[] = [];
  for (let k = 0; k < n; k++) {
    let best = -1, bs = -1;
    for (let i = 0; i < n; i++) if (!done[i]) { let s = 0; for (const x of pd[i]) if (seen.get(x) === 1) s++; if (s > bs) { bs = s; best = i; } }
    done[best] = 1; order.push(best);
    for (const x of pd[best]) seen.set(x, (seen.get(x) ?? 0) + 1);
  }
  const dpow = [ONE, DELTA, mul(DELTA, DELTA)];
  let states = new Map<string, { P: Map<number, number>; p: Laurent }>([['', { P: new Map(), p: ONE }]]);
  for (const i of order) {
    const [a, b, c, d] = pd[i], next = new Map<string, { P: Map<number, number>; p: Laurent }>();
    for (const { P, p } of states.values()) {
      // A-smoothing joins (a,b),(c,d) with weight A; B-smoothing joins (a,d),(b,c) with weight A⁻¹
      for (const [e, x1, y1, x2, y2] of [[1, a, b, c, d], [-1, a, d, b, c]]) {
        const Q = new Map(P), loops = join(Q, x1, y1) + join(Q, x2, y2), k = keyOf(Q), q = shift(loops ? mul(p, dpow[loops]) : p, e);
        const s = next.get(k);
        if (s) s.p = add(s.p, q); else next.set(k, { P: Q, p: q });
      }
    }
    if (next.size > limit) return null;
    states = next;
  }
  return divDelta(states.get('')!.p);
}

/** V(t) = (−A³)^(−writhe) · ⟨D⟩ at A = t^(−1/4); null when the bracket is too large to compute. */
export function jones(pd: PD, signs: (1 | -1)[], limit?: number): Laurent | null {
  const br = bracket(pd, limit);
  if (!br) return null;
  const w = signs.reduce((s, x) => s + x, 0);
  let V = shift(br, -3 * w);
  if (w % 2) V = scale(V, -1n);
  V = trim(V);
  // A = t^(-1/4): A^e -> t^(-e/4)
  let res: Laurent = { lo: 0, c: [] };
  V.c.forEach((x, i) => {
    if (!x) return;
    const e = i + V.lo;
    if (e % 4) throw new Error('Jones: exponent not divisible by 4 (not a knot?)');
    res = add(res, { lo: -e / 4, c: [x] });
  });
  return res;
}
