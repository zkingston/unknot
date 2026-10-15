import { type Diagram, isSplit } from '../diagram/types';
import { pdCode } from './pd';
import { alexander, type AlexanderResult } from './alexander';
import { jones } from './jones';
import { type Laurent, isOne } from './laurent';

export interface CompInvariants { crossings: number; writhe: number; alexander: AlexanderResult; jones: Laurent | 'skipped' }
export type Analysis =
  | { kind: 'flat'; flat: number }
  | { kind: 'nonsplit'; comps: number }
  | { kind: 'ok'; comps: CompInvariants[]; knotted: boolean };

/** Analysis as JSON, Laurent coefficients as numbers (throws if one is not a safe integer), and back. */
export const analysisToJSON = (a: Analysis) => JSON.stringify(a, (_, v) => {
  if (typeof v !== 'bigint') return v;
  if (!Number.isSafeInteger(Number(v))) throw new Error(`coefficient ${v} is too large for JSON`);
  return Number(v);
});
export const analysisFromJSON = (j: unknown): Analysis => JSON.parse(JSON.stringify(j), (k, v) => (k === 'c' && Array.isArray(v) ? v.map(BigInt) : v));

export const alexIsOne = (a: AlexanderResult) => (a.kind === 'exact' ? isOne(a.poly) : a.isOne);
const compKnotted = (c: CompInvariants) => !alexIsOne(c.alexander) || (c.jones !== 'skipped' && !isOne(c.jones));

/** Invariants per component, from that component's self-crossings. Non-split links skip invariants. */
export function analyze(d: Diagram, progress?: (f: number) => void): Analysis {
  const flat = d.crossings.filter((X) => X.flat).length;
  if (flat) return { kind: 'flat', flat };
  if (!isSplit(d)) return { kind: 'nonsplit', comps: d.comps.length };
  const comps = d.comps.map((_, c) => {
    const { pd, signs } = pdCode(d, [c], true);
    return {
      crossings: pd.length, writhe: signs.reduce((s, x) => s + x, 0),
      alexander: alexander(pd, signs, progress),
      jones: jones(pd, signs) ?? 'skipped' as const,
    };
  });
  return { kind: 'ok', comps, knotted: comps.some(compKnotted) };
}
