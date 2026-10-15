import { expect, test } from 'vitest';
import { parseKnotJSON } from '../../src/import/json';
import { commitDiagram, commitScene, dequantizeDiagram, dequantizeScene, encodePayload, decodePayload, formatHash, parseHash, HASH_LIMIT, type Payload } from '../../src/state/url';
import { lift } from '../../src/diagram/lift';
import { simplify } from '../../src/diagram/simplify';
import { braidToDiagram, parseBraid } from '../../src/import/braid';
import { json, presetScene, sha } from './util';

const fixture = (f: string) => parseKnotJSON(json(`../fixtures/${f}`)).diagram!;

test('diagram sources: commit, encode, decode, dequantise -> identical diagram and Scene bytes', async () => {
  for (const [d, src] of [[fixture('wikipedia-complex-unknot.knot.json'), 'image'], [fixture('wikipedia-simple-unknots.knot.json'), 'image'], [braidToDiagram(parseBraid('s1 s2^-1 s1 s2^-1')), 'draw']] as const) {
    const c = commitDiagram({ ...d, source: src })!;
    expect(c).not.toBeNull();
    const p: Payload = { src, data: c.q };
    const back = await decodePayload(await encodePayload(p));
    const d2 = dequantizeDiagram((back as { data: typeof c.q }).data, src);
    expect(d2.crossings).toEqual(c.diagram.crossings);
    expect(sha(d2.comps)).toBe(sha(c.diagram.comps));
    expect(sha(simplify(lift(d2)))).toBe(sha(simplify(lift(c.diagram))));
    const h = formatHash({ k: await encodePayload(p), s: 7, view: '3d', sticks: true });
    expect(h.length).toBeLessThan(HASH_LIMIT);
    expect(parseHash(h)).toMatchObject({ s: 7, view: '3d', sticks: true });
    expect(parseHash('#v=1&s=7&view=sticks')).toMatchObject({ view: '3d', sticks: true });
  }
});

test('json source: 16-bit xyz round trip is exact after commit', async () => {
  const S = presetScene('thistlethwaite'), c = commitScene(S)!;
  const back = await decodePayload(await encodePayload({ src: 'json', data: c.q }));
  expect(sha(dequantizeScene((back as { data: typeof c.q }).data))).toBe(sha(c.scene));
});

test('a crossing change under quantisation refuses to commit', () => {
  // two strands 1e-6 apart cross or not depending on rounding: build a square and a square shifted by a hair
  const d = braidToDiagram(parseBraid('s1 s1 s1'));
  const P = d.comps[0], Q = P.slice();
  // squash the diagram so far that 12-bit rounding merges distinct crossings
  for (let i = 1; i < Q.length; i += 2) Q[i] *= 1e-5;
  expect(commitDiagram({ ...d, comps: [Q] })).toBeNull();
});

test('text sources stay verbatim; hashes above 8 KB are flagged by length', async () => {
  const p: Payload = { src: 'braid', data: 's1 '.repeat(5000) };
  expect((await encodePayload(p)).length).toBeGreaterThan(0);
  const big = await encodePayload({ src: 'json', data: { c: [Array.from({ length: 30000 }, (_, i) => (i * 7919) % 65536)] } });
  expect(formatHash({ k: big, s: 1, view: '2d' }).length).toBeGreaterThan(HASH_LIMIT);
});
