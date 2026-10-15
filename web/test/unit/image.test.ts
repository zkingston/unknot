import { describe, expect, test } from 'vitest';
import { loadPNG, rasterise, saltPepper, parsePaths } from './png';
import { rand } from './util';
import { traceImage, assembleArcs } from '../../src/import/image/assemble';
import { maxWeightMatching } from '../../src/import/image/match';
import { validateDiagram } from '../../src/diagram/crossings';
import { projectScene, TOP } from '../../src/diagram/project';
import { fibonacciHemisphere, basisFor, scoreView } from '../../src/diagram/bestView';
import { knotPaths } from '../../src/view2d/render';
import { analyze, alexIsOne } from '../../src/invariants';
import { eq, type Laurent } from '../../src/invariants/laurent';
import type { Scene } from '../../src/kernel/scene';

function curve(f: (t: number) => [number, number, number], N = 400): Scene {
  const P = new Float64Array(3 * N);
  for (let k = 0; k < N; k++) P.set(f((2 * Math.PI * k) / N), 3 * k);
  return [P];
}
const trefoil = curve((t) => [(Math.sin(t) + 2 * Math.sin(2 * t)) / 3, (Math.cos(t) - 2 * Math.cos(2 * t)) / 3, 0.6 * Math.sin(3 * t)]);
const figure8 = curve((t) => [((2 + Math.cos(2 * t)) * Math.cos(3 * t)) / 3, ((2 + Math.cos(2 * t)) * Math.sin(3 * t)) / 3, Math.sin(4 * t) / 3]);
const ok = (d: Parameters<typeof analyze>[0]) => { const a = analyze(d); if (a.kind !== 'ok') throw new Error(a.kind); return a; };

describe('maximum-weight matching', () => {
  /** Best (cardinality, weight) over all matchings, by brute force. */
  function brute(n: number, E: [number, number, number][], maxcard: boolean) {
    let best = [-1, -Infinity];
    const rec = (k: number, used: number, card: number, w: number) => {
      if (k === E.length) { if (maxcard ? card > best[0] || (card === best[0] && w > best[1]) : w > best[1]) best = [card, w]; return; }
      rec(k + 1, used, card, w);
      const [i, j, wt] = E[k];
      if (!(used & (1 << i)) && !(used & (1 << j))) rec(k + 1, used | (1 << i) | (1 << j), card + 1, w + wt);
    };
    rec(0, 0, 0, 0);
    void n;
    return best;
  }
  test('agrees with brute force on 300 random graphs', () => {
    const r = rand(9);
    for (let g = 0; g < 300; g++) {
      const n = 2 + Math.floor(r() * 7), E: [number, number, number][] = [];
      for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) if (r() < 0.5) E.push([i, j, Math.floor(r() * 20) - (g % 3 === 0 ? 5 : 0)]);
      if (!E.length) continue;
      for (const maxcard of [false, true]) {
        const mate = maxWeightMatching(E, maxcard);
        let card = 0, w = 0;
        mate.forEach((j, i) => { if (j > i) { card++; w += E.find((e) => (e[0] === i && e[1] === j) || (e[0] === j && e[1] === i))![2]; } expect(j < 0 || mate[j] === i).toBe(true); });
        const [bc, bw] = brute(n, E, maxcard);
        if (maxcard) expect([card, w], `graph ${g}`).toEqual([bc, bw]); else expect(w, `graph ${g}`).toBe(Math.max(0, bw));
      }
    }
  });
});

describe('Wikipedia images', () => {
  for (const [f, arcs, nx, comps] of [['complex.png', 6, 6, 1], ['simple.png', 3, 2, 2]] as const) test(`${f}: ${arcs} arcs, ${nx} crossings, ${comps} comps, 0 flat, within 1.5 s`, () => {
    const img = loadPNG(`../fixtures/${f}`);
    const t0 = performance.now(), r = traceImage(img), ms = performance.now() - t0;
    console.log(`${f}: ${ms.toFixed(0)} ms, sw ${r.sw.toFixed(1)}, ${JSON.stringify(r.diagnostics)}`);
    expect(r.diagnostics).toEqual({ arcs, unpaired: 0, flat: 0, components: comps });
    expect(r.diagram.crossings.length).toBe(nx);
    expect(validateDiagram(r.diagram).ok).toBe(true);
    expect(ms).toBeLessThan(1500);
    const a = ok(r.diagram);
    expect(a.knotted).toBe(false);
    if (f === 'complex.png') expect(a.comps[0].writhe).toBe(-2);
  });
  test('forbidding a matched pair and forcing it back', () => {
    const img = loadPNG('../fixtures/complex.png'), r = traceImage(img);
    const i = 0, j = r.endpoints[0].partner;
    const r2 = traceImage(img, { forbid: [[i, j]] });
    expect(r2.endpoints[i].partner).not.toBe(j);
    const r3 = traceImage(img, { forbid: [[i, j]], force: [[i, j]] });
    expect(r3.endpoints[i].partner).toBe(j);
  });
});

describe('robustness: our own renderer, pens 2/4/8 px, 2% salt-and-pepper', () => {
  for (const [name, S, want] of [['trefoil', trefoil, 3], ['figure-eight', figure8, 4]] as const) {
    // a gap of 3 × pen is visible only if 3 sin θ > 1, and only if no third strand runs through it:
    // render from the minimal-crossing view with angles >= 35° and the best-isolated crossings
    let view = TOP, best = -1;
    for (const d of fibonacciHemisphere(512)) { const v = basisFor(d), sc = scoreView(S, v); if (sc.c === want && sc.theta >= 35 && sc.s > best) { best = sc.s; view = v; } }
    const ref = ok(projectScene(S, view)).comps[0];
    test(`${name}: best view has ${want} crossings`, () => expect(ref.crossings).toBe(want));
    for (const pen of [2, 4, 8]) test(`${name} at ${pen} px`, () => {
      const W = 1000, f = { cx: W / 2, cy: W / 2, s: W * 0.42 };
      const { d } = knotPaths(S, view, f, Math.max(3 * pen, 4));
      const img = saltPepper(rasterise(W, W, parsePaths(d), pen), 0.02, rand(pen));
      const r = traceImage(img);
      expect(r.diagnostics.unpaired, JSON.stringify(r.diagnostics)).toBe(0);
      expect(r.diagram.crossings.length).toBe(want);
      expect(validateDiagram(r.diagram).ok).toBe(true);
      const c = ok(r.diagram).comps[0];
      expect(c.crossings).toBe(want);
      expect(alexIsOne(c.alexander)).toBe(false);
      expect(c.alexander.kind === 'exact' && ref.alexander.kind === 'exact' && eq(c.alexander.poly, ref.alexander.poly)).toBe(true);
      expect(eq(c.jones as Laurent, ref.jones as Laurent)).toBe(true);
    });
  }
});

test('assembleArcs works from vector arcs with a polyline ink test (SVG path)', () => {
  // two arcs of an unknot drawn as a circle with one fake gap crossing a straight bar: no candidates -> unpaired
  const xy = new Float64Array(2 * 50);
  for (let k = 0; k < 50; k++) { const t = (2 * Math.PI * k) / 50; xy[2 * k] = 100 * Math.cos(t); xy[2 * k + 1] = 100 * Math.sin(t); }
  const r = assembleArcs([{ xy, closed: true }], 4, () => false);
  expect(r.diagnostics).toEqual({ arcs: 1, unpaired: 0, flat: 0, components: 1 });
  expect(r.diagram.crossings.length).toBe(0);
});
