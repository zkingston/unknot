// PD code -> drawn Diagram from the rotation system, without trig (so layouts match across browsers):
// subdivided planar map, face-node triangulation, Tutte embedding (W. T. Tutte, "How to Draw a Graph", 1963)
// by conjugate gradient, relaxation, mirror check, cubic splines; validated, with fallbacks.
import type { PD } from '../invariants/pd';
import type { Crossing, Diagram } from '../diagram/types';
import { findCrossings, validateDiagram } from '../diagram/crossings';
import { pdStructure, type PDComponent } from './pdcode';

export interface EmbedMeta { source?: 'pd' | 'gauss'; sourceText?: string; name?: string }

interface Graph {
  adj: number[][];            // map adjacency; crossings in slot order (counter-clockwise), others [prev, next]
  slotNbr: Int32Array;        // 4c + s -> neighbouring node along that slot's edge
  edgeNodes: number[][][];    // [comp][pass k] -> subdivision nodes of the edge leaving pass k
  nodes: number;
}

/** Subdivide each edge with 2 nodes, 3 if it returns to its own crossing, so no double edges. */
function buildGraph(n: number, comps: PDComponent[]): Graph {
  const adj: number[][] = Array.from({ length: n }, () => [0, 0, 0, 0]), slotNbr = new Int32Array(4 * n);
  const edgeNodes = comps.map((C) => C.passes.map((p, k) => {
    const q = C.passes[(k + 1) % C.passes.length], m = p.c === q.c ? 3 : 2, ids: number[] = [];
    for (let i = 0; i < m; i++) { ids.push(adj.length); adj.push([]); }
    ids.forEach((v, i) => { adj[v] = [i ? ids[i - 1] : p.c, i + 1 < m ? ids[i + 1] : q.c]; });
    slotNbr[4 * p.c + p.sOut] = ids[0]; slotNbr[4 * q.c + q.sIn] = ids[m - 1];
    adj[p.c][p.sOut] = ids[0]; adj[q.c][q.sIn] = ids[m - 1];
    return ids;
  }));
  return { adj, slotNbr, edgeNodes, nodes: adj.length };
}

/** Faces of the embedded map (each a cyclic node list), interior on the left of each dart. */
function faces(adj: number[][], nodes: number[]): number[][] {
  const seen = new Set<number>(), N = adj.length, out: number[][] = [];
  for (const u0 of nodes) for (const v0 of adj[u0]) {
    if (seen.has(u0 * N + v0)) continue;
    const f: number[] = [];
    for (let u = u0, v = v0; !seen.has(u * N + v); ) {
      seen.add(u * N + v); f.push(u);
      const a = adj[v], i = a.indexOf(u), w = a.length === 2 ? a[1 - i] : a[(i + a.length - 1) % a.length];
      u = v; v = w;
    }
    out.push(f);
  }
  return out;
}

/** Conjugate gradient on the Tutte system (free nodes at the mean of their neighbours). Fixed iteration order. */
function tutte(nbr: number[][], pinned: Uint8Array, X: Float64Array, Y: Float64Array) {
  const free = [...nbr.keys()].filter((v) => !pinned[v]), idx = new Int32Array(nbr.length).fill(-1);
  free.forEach((v, i) => { idx[v] = i; });
  const m = free.length;
  const mul = (x: Float64Array, out: Float64Array) => {
    for (let i = 0; i < m; i++) { const v = free[i]; let s = nbr[v].length * x[i]; for (const w of nbr[v]) if (idx[w] >= 0) s -= x[idx[w]]; out[i] = s; }
  };
  for (const P of [X, Y]) {
    const b = new Float64Array(m), x = new Float64Array(m), r = new Float64Array(m), p = new Float64Array(m), Ap = new Float64Array(m);
    for (let i = 0; i < m; i++) for (const w of nbr[free[i]]) if (idx[w] < 0) b[i] += P[w];
    r.set(b); p.set(b);
    let rs = 0; for (let i = 0; i < m; i++) rs += r[i] * r[i];
    for (let it = 0; it < 10 * m + 100 && Math.sqrt(rs) > 1e-12; it++) {
      mul(p, Ap);
      let pAp = 0; for (let i = 0; i < m; i++) pAp += p[i] * Ap[i];
      const a = rs / pAp;
      let rn = 0;
      for (let i = 0; i < m; i++) { x[i] += a * p[i]; r[i] -= a * Ap[i]; rn += r[i] * r[i]; }
      const beta = rn / rs; rs = rn;
      for (let i = 0; i < m; i++) p[i] = r[i] + beta * p[i];
    }
    for (let i = 0; i < m; i++) P[free[i]] = x[i];
  }
}

const hyp = (x: number, y: number) => Math.sqrt(x * x + y * y);
const unit = (x: number, y: number): [number, number] => { const l = Math.sqrt(x * x + y * y) || 1; return [x / l, y / l]; };

/** Pseudo-angle in [0, 4), monotone in the true angle (no trig). */
const pangle = (x: number, y: number) => (y >= 0 ? (x >= 0 ? y / (x + y) : 1 + -x / (-x + y)) : x < 0 ? 2 + -y / (-x - y) : 3 + x / (x - y));

/** Lay out one connected part; returns node positions (map nodes; face nodes discarded). */
function layoutPart(g: Graph, cross: number[], partNodes: number[], X: Float64Array, Y: Float64Array, rank = 0) {
  const F = faces(g.adj, partNodes);
  // outer face: the most boundary nodes, preferring a simple boundary; `rank` takes the next choices on a retry
  const simple = (f: number[]) => new Set(f).size === f.length;
  const order = F.map((_, i) => i).sort((i, j) => Number(simple(F[j])) - Number(simple(F[i])) || F[j].length - F[i].length || i - j);
  const outer = order[rank % order.length];
  // triangulate: a node per inner face, joined to its boundary
  const base = g.nodes + partNodes.length; // face and ring nodes get ids past every map node (local scratch)
  const nbr: number[][] = [];
  const all = new Map<number, number>(); // global id -> local
  const local = (v: number) => { let i = all.get(v); if (i === undefined) { i = nbr.length; all.set(v, i); nbr.push([]); } return i; };
  for (const v of partNodes) for (const w of g.adj[v]) { const a = local(v), b = local(w); if (!nbr[a].includes(b)) nbr[a].push(b); }
  const tris: number[][] = [];
  F.forEach((f, i) => {
    if (i === outer) return;
    const fn = local(base + i);
    for (const v of new Set(f)) { nbr[fn].push(local(v)); nbr[local(v)].push(fn); }
    for (let k = 0; k < f.length; k++) tris.push([fn, local(f[k]), local(f[(k + 1) % f.length])]);
  });
  // the outer face is attached to a ring pinned counter-clockwise on the unit circle, one ring node per boundary
  // node, at ((1−u²)/(1+u²), 2u/(1+u²)) with u evenly in [−3, 3]; so crossings on the outer face stay interior
  // (pinned crossings would have all four neighbours in one half-plane and could not be drawn transversally)
  const of = F[outer], m = of.length, ring = of.map((_, k) => local(base + F.length + k));
  const link = (a: number, b: number) => { if (!nbr[a].includes(b)) { nbr[a].push(b); nbr[b].push(a); } };
  for (let k = 0; k < m; k++) {
    const f0 = local(of[k]), f1 = local(of[(k + 1) % m]), r0 = ring[k], r1 = ring[(k + 1) % m];
    link(f0, r0); link(f0, r1); link(r0, r1);
    tris.push([f0, f1, r1], [f0, r1, r0]);
  }
  const L = nbr.length, px = new Float64Array(L), py = new Float64Array(L), pin = new Uint8Array(L);
  ring.forEach((i, k) => {
    // the outer face is traced clockwise (its interior, the unbounded region, on the left): u decreases
    const u = 3 - (6 * k) / Math.max(1, m - 1);
    px[i] = (1 - u * u) / (1 + u * u); py[i] = (2 * u) / (1 + u * u); pin[i] = 1;
  });
  tutte(nbr, pin, px, py);
  relax(g, cross, partNodes, local, nbr, tris, pin, px, py);
  // mirror check: the first crossing's slots must run counter-clockwise
  const c0 = cross[0], i0 = local(c0);
  let turn = 0;
  for (let s = 0; s < 4; s++) {
    const a = local(g.slotNbr[4 * c0 + s]), b = local(g.slotNbr[4 * c0 + ((s + 1) & 3)]);
    turn += (pangle(px[b] - px[i0], py[b] - py[i0]) - pangle(px[a] - px[i0], py[a] - py[i0]) + 4) % 4;
  }
  const flip = turn > 8 ? -1 : 1;
  for (const v of partNodes) { const i = local(v); X[v] = flip * px[i]; Y[v] = py[i]; }
}

/** 60 steps toward even edge lengths and perpendicular crossings; a step that flips a triangle is rejected. */
function relax(g: Graph, cross: number[], partNodes: number[], local: (v: number) => number, nbr: number[][], tris: number[][], pin: Uint8Array, px: Float64Array, py: Float64Array) {
  const edges: [number, number][] = [];
  for (const v of partNodes) for (const w of g.adj[v]) if (v < w) edges.push([local(v), local(w)]);
  const springs: [number, number][] = [];
  const mapCount = new Set(partNodes.map(local)).size;
  for (let i = mapCount; i < nbr.length; i++) for (const b of nbr[i]) springs.push([i, b]);
  const area = (t: number[], X: Float64Array, Y: Float64Array) => (X[t[1]] - X[t[0]]) * (Y[t[2]] - Y[t[0]]) - (Y[t[1]] - Y[t[0]]) * (X[t[2]] - X[t[0]]);
  const area0 = tris.map((t) => area(t, px, py)), sign0 = area0.map(Math.sign);
  const gx = new Float64Array(px.length), gy = new Float64Array(px.length), nx = new Float64Array(px.length), ny = new Float64Array(px.length);
  const slots = cross.map((c) => [0, 1, 2, 3].map((s) => local(g.slotNbr[4 * c + s])));
  const strands: [number, number, number][] = [];
  cross.forEach((c, k) => { const [s0, s1, s2, s3] = slots[k]; strands.push([local(c), s0, s2], [local(c), s1, s3]); });
  for (const v of partNodes) if (g.adj[v].length === 2) strands.push([local(v), local(g.adj[v][0]), local(g.adj[v][1])]);
  const eta = 0.05;
  for (let it = 0; it < 60; it++) {
    let Lm = 0; for (const [a, b] of edges) Lm += hyp(px[a] - px[b], py[a] - py[b]);
    Lm /= Math.max(1, edges.length);
    gx.fill(0); gy.fill(0);
    for (const [a, b] of edges) {
      const dx = px[a] - px[b], dy = py[a] - py[b], l = Math.sqrt(dx * dx + dy * dy) || 1e-12, k = (2 * (l - Lm)) / (Lm * Lm * l);
      gx[a] += k * dx; gy[a] += k * dy; gx[b] -= k * dx; gy[b] -= k * dy;
    }
    for (const [s0, s1, s2, s3] of slots) {
      const Ax = px[s2] - px[s0], Ay = py[s2] - py[s0], Bx = px[s1] - px[s3], By = py[s1] - py[s3];
      const a = Math.sqrt(Ax * Ax + Ay * Ay) || 1e-12, b = Math.sqrt(Bx * Bx + By * By) || 1e-12;
      const ux = Ax / a, uy = Ay / a, ox = Bx / b, oy = By / b, d = ux * ox + uy * oy;
      const w = 0.5, gux = (2 * w * d * (ox - d * ux)) / a, guy = (2 * w * d * (oy - d * uy)) / a, gox = (2 * w * d * (ux - d * ox)) / b, goy = (2 * w * d * (uy - d * oy)) / b;
      gx[s2] += gux; gy[s2] += guy; gx[s0] -= gux; gy[s0] -= guy; gx[s1] += gox; gy[s1] += goy; gx[s3] -= gox; gy[s3] -= goy;
    }
    // strands straight and evenly spaced: each node toward the midpoint of its strand neighbours
    for (const [v, a, b] of strands) {
      const k = 2 / (Lm * Lm), ex = px[v] - 0.5 * (px[a] + px[b]), ey = py[v] - 0.5 * (py[a] + py[b]);
      gx[v] += 2 * k * ex; gy[v] += 2 * k * ey; gx[a] -= k * ex; gy[a] -= k * ey; gx[b] -= k * ex; gy[b] -= k * ey;
    }
    for (const [f, b] of springs) {
      const k = 2 / (Lm * Lm), dx = px[f] - px[b], dy = py[f] - py[b];
      gx[f] += k * dx; gy[f] += k * dy; gx[b] -= k * dx; gy[b] -= k * dy;
    }
    // proposed step; nodes of any triangle it would flip (or collapse) stay put for this step
    for (let i = 0; i < px.length; i++) {
      let dx = -eta * Lm * Lm * gx[i], dy = -eta * Lm * Lm * gy[i];
      const l = hyp(dx, dy), cap = 0.1 * Lm;
      if (l > cap) { dx *= cap / l; dy *= cap / l; }
      nx[i] = pin[i] ? px[i] : px[i] + dx; ny[i] = pin[i] ? py[i] : py[i] + dy;
    }
    let bad = true;
    for (let pass = 0; pass < 12 && bad; pass++) {
      bad = false;
      tris.forEach((t, k) => {
        if (!sign0[k]) return;
        const a = area(t, nx, ny);
        if (Math.sign(a) === sign0[k] && Math.abs(a) >= 0.02 * Math.abs(area0[k])) return;
        bad = true;
        for (const v of t) { nx[v] = px[v]; ny[v] = py[v]; }
      });
    }
    if (bad) break;
    px.set(nx); py.set(ny);
  }
}

/** Cubic Bézier sample of the Hermite segment A->B with unit tangents tA, tB and tangent length λ|AB|. */
function hermite(out: number[], ax: number, ay: number, ta: [number, number], bx: number, by: number, tb: [number, number], lam: number, m: number) {
  const L = lam * hyp(bx - ax, by - ay);
  const c1x = ax + ta[0] * L, c1y = ay + ta[1] * L, c2x = bx - tb[0] * L, c2y = by - tb[1] * L;
  for (let i = 0; i < m; i++) {
    const t = (i + 0.5) / m, s = 1 - t;
    out.push(s * s * s * ax + 3 * s * s * t * c1x + 3 * s * t * t * c2x + t * t * t * bx, s * s * s * ay + 3 * s * s * t * c1y + 3 * s * t * t * c2y + t * t * t * by);
  }
}

export function pdToDiagram(pd: PD, meta: EmbedMeta = {}): Diagram {
  let lastWhy = '';
  const n = pd.length, st = pdStructure(pd), g = buildGraph(n, st.comps);
  const X = new Float64Array(g.nodes), Y = new Float64Array(g.nodes);
  // each connected part separately, side by side
  const partOf = new Map<number, number[]>();
  st.parts.forEach((p, c) => { const l = partOf.get(p); if (l) l.push(c); else partOf.set(p, [c]); });
  const layout = (rank: number) => {
    let off = 0;
    for (const cross of partOf.values()) {
      const inPart = new Set(cross), nodes = [...cross];
      st.comps.forEach((C, ci) => { if (inPart.has(C.passes[0].c)) for (const ids of g.edgeNodes[ci]) nodes.push(...ids); });
      layoutPart(g, cross, nodes, X, Y, rank);
      for (const v of nodes) X[v] += off;
      off += 2.5;
    }
  };
  const { source = 'pd', sourceText, name } = meta, dmeta = sourceText || name ? { meta: { ...(name && { name }), ...(sourceText && { sourceText }) } } : {};
  const through = (p: { c: number; sIn: number; sOut: number }) => {
    const a = g.slotNbr[4 * p.c + p.sIn], b = g.slotNbr[4 * p.c + p.sOut];
    return unit(X[b] - X[a], Y[b] - Y[a]);
  };
  // radius of the crossing circle: a third of the distance from the crossing node to anything not incident to it
  const rhoCache = new Map<number, number>();
  const rho = (c: number) => {
    let r = rhoCache.get(c);
    if (r !== undefined) return r;
    r = Infinity;
    for (let s = 0; s < 4; s++) { const v = g.slotNbr[4 * c + s]; r = Math.min(r, hyp(X[v] - X[c], Y[v] - Y[c])); }
    for (let v = 0; v < g.nodes; v++) for (const w of g.adj[v]) {
      if (w < v || v === c || w === c) continue;
      const ux = X[w] - X[v], uy = Y[w] - Y[v], l2 = ux * ux + uy * uy;
      let t = l2 > 0 ? ((X[c] - X[v]) * ux + (Y[c] - Y[v]) * uy) / l2 : 0; t = t < 0 ? 0 : t > 1 ? 1 : t;
      r = Math.min(r, hyp(X[v] + t * ux - X[c], Y[v] + t * uy - Y[c]));
    }
    r /= 3;
    rhoCache.set(c, r);
    return r;
  };
  const attempt = (lam: number | null): Diagram | null => {
    const comps: Float64Array[] = [], tags: Int32Array[] = [];
    st.comps.forEach((C, ci) => {
      const pts: number[] = [], tag: number[] = [], m = C.passes.length;
      const mark = (k: number) => { tag[pts.length / 2 - 1] = 2 * C.passes[k % m].c + (C.passes[k % m].sIn & 1); };
      for (let k = 0; k < m; k++) {
        const p = C.passes[k], q = C.passes[(k + 1) % m], ids = g.edgeNodes[ci][k];
        if (lam === null) {
          // fallback that cannot fail on a planar straight-line map: follow the map's own edges, and cross on a
          // circle of radius ρ around the crossing node (ρ below the distance to any other edge); the two chords
          // join points in alternating cyclic order on that circle, so they cross exactly once
          const a = g.slotNbr[4 * p.c + p.sIn], b = g.slotNbr[4 * p.c + p.sOut], r = rho(p.c);
          const da = unit(X[a] - X[p.c], Y[a] - Y[p.c]), db = unit(X[b] - X[p.c], Y[b] - Y[p.c]);
          pts.push(X[p.c] + r * da[0], Y[p.c] + r * da[1]);
          mark(k);
          pts.push(X[p.c] + r * db[0], Y[p.c] + r * db[1]);
          for (const v of ids) pts.push(X[v], Y[v]);
          continue;
        }
        const chain = [p.c, ...ids, q.c];
        const tan = (j: number): [number, number] => {
          if (j === 0) return through(p);
          if (j === chain.length - 1) return through(q);
          return unit(X[chain[j + 1]] - X[chain[j - 1]], Y[chain[j + 1]] - Y[chain[j - 1]]);
        };
        for (let j = 0; j + 1 < chain.length; j++) hermite(pts, X[chain[j]], Y[chain[j]], tan(j), X[chain[j + 1]], Y[chain[j + 1]], tan(j + 1), lam, 6);
        mark(k + 1);
      }
      comps.push(Float64Array.from(pts));
      tags.push(Int32Array.from({ length: pts.length / 2 }, (_, i) => tag[i] ?? -1));
    });
    const found = findCrossings(comps);
    if (found.length !== n) { if (lam === null) lastWhy = `found ${found.length} crossings, want ${n}`; return null; }
    const used = new Uint8Array(n), crossings: Crossing[] = [];
    for (const F of found) {
      const ta = tags[F.a.comp][F.a.seg], tb = tags[F.b.comp][F.b.seg];
      if (ta < 0 || tb < 0 || ta >> 1 !== tb >> 1 || (ta & 1) === (tb & 1) || used[ta >> 1]) return null;
      used[ta >> 1] = 1;
      crossings.push(ta & 1 ? { under: F.b, over: F.a } : { under: F.a, over: F.b });
    }
    const d: Diagram = { comps, crossings, source, ...dmeta };
    const v = validateDiagram(d);
    if (!v.ok && lam === null) lastWhy = v.error ?? '';
    return v.ok ? d : null;
  };
  // a few outer-face choices: an unlucky one can leave the Tutte embedding numerically degenerate
  for (let rank = 0; rank < 6; rank++) {
    layout(rank); rhoCache.clear();
    for (const lam of [0.4, 0.2, 0.1, 0.05, null]) { const d = attempt(lam); if (d) return d; }
  }
  throw new Error(`layout failed: could not draw this PD code without extra crossings (${lastWhy})`);
}
