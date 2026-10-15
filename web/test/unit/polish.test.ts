import { expect, test } from 'vitest';
import { Grouper, edgesClear, shortcut, lastProgress } from '../../src/planner/polish';
import { runSync } from '../../src/planner/run';
import { replay, REC, Op, PhaseKind } from '../../src/planner/movelog';
import { cloneScene, type Scene } from '../../src/kernel/scene';
import { segSeg, pt, lerp, checkIsotopy, type P3 } from './isotopy';
import { Trajectory } from '../../src/planner/trajectory';
import { smoothShown } from '../../src/diagram/smooth';
import { rand, presetScene, CORE } from './util';


/** Independent sampled check of a simultaneous linear motion S0 -> S1 (same vertex counts): no strand passage. */
let lastFail = '';
function sampledClear(S0: Scene, S1: Scene, samples = 96): boolean {
  const edges: { c: number; i: number; j: number; moving: boolean }[] = [];
  S0.forEach((P, c) => { const n = P.length / 3; for (let i = 0; i < n; i++) { const j = (i + 1) % n; const mv = (k: number) => P[3 * k] !== S1[c][3 * k] || P[3 * k + 1] !== S1[c][3 * k + 1] || P[3 * k + 2] !== S1[c][3 * k + 2]; edges.push({ c, i, j, moving: mv(i) || mv(j) }); } });
  const at = (c: number, k: number, t: number) => lerp(pt(S0[c], k), pt(S1[c], k), t);
  for (let x = 0; x < edges.length; x++) for (let y = x + 1; y < edges.length; y++) {
    const e = edges[x], f = edges[y];
    if (!e.moving && !f.moving) continue;
    if (e.c === f.c && (e.i === f.i || e.i === f.j || e.j === f.i || e.j === f.j)) continue;
    const vol = (t: number) => { const a = at(e.c, e.i, t), b = at(e.c, e.j, t), c = at(f.c, f.i, t), d = at(f.c, f.j, t);
      const u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], w = [c[0] - a[0], c[1] - a[1], c[2] - a[2]], z = [d[0] - a[0], d[1] - a[1], d[2] - a[2]];
      return u[0] * (w[1] * z[2] - w[2] * z[1]) + u[1] * (w[2] * z[0] - w[0] * z[2]) + u[2] * (w[0] * z[1] - w[1] * z[0]); };
    let prev = vol(0);
    for (let s = 1; s <= samples; s++) {
      const t = s / samples, cur = vol(t);
      if ((prev < 0 && cur > 0) || (prev > 0 && cur < 0)) {
        let lo = (s - 1) / samples, hi = t, flo = prev;
        for (let k = 0; k < 60; k++) { const m = 0.5 * (lo + hi), fm = vol(m); if ((fm < 0) === (flo < 0)) { lo = m; flo = fm; } else hi = m; }
        const dd = segSeg(at(e.c, e.i, lo), at(e.c, e.j, lo), at(f.c, f.i, lo), at(f.c, f.j, lo));
        if (dd < 1e-7) { lastFail = `edges ${e.c}:${e.i}-${e.j} (moving ${e.moving}) vs ${f.c}:${f.i}-${f.j} (moving ${f.moving}) at t=${lo} d=${dd}`; return false; }
      }
      prev = cur;
    }
  }
  return true;
}

/** Check each group of (recs, ends) from S as one simultaneous motion with the sampled checker; returns totals. */
function checkGroups(S: Scene, recs: Float64Array, ends: number[], label: string) {
  let groups = 0, moves = 0, start = 0;
  const cur = cloneScene(S);
  for (const end of ends) {
    // the group's motion: every Move target mapped back to the start state, deletions applied after
    const S1 = cloneScene(cur), alive = cur.map((P) => Array.from({ length: P.length / 3 }, (_, i) => i));
    let m = 0;
    for (let q = start; q < end; q++) {
      const o = q * REC, op = recs[o], c = recs[o + 1];
      if (op === Op.Move) { const v = alive[c][recs[o + 2]]; S1[c].set(recs.subarray(o + 3, o + 6), 3 * v); m++; }
      else if (op === Op.Delete) alive[c].splice(recs[o + 2], 1);
      else if (op === Op.Insert) expect(end - start, `${label}: an insert is a group of its own`).toBe(1);
    }
    if (m > 1) { const ok = sampledClear(cur, S1); if (!ok) console.log(label, start, end, lastFail); expect(ok, `${label} group ${start}-${end}`).toBe(true); }
    replay(cur, recs, start, end);
    groups++; moves += m; start = end;
  }
  return { groups, moves, final: cur };
}

test('groups are verified simultaneous motions that reproduce the log exactly', () => {
  const r = rand(5);
  let groups = 0, moves = 0;
  for (const id of CORE) {
    const S = presetScene(id);
    for (let seed = 1; seed <= 2; seed++) {
      // the knotted ones stop once they stall, as in the app (a time budget would make the log depend on the machine)
      const recs = runSync(S, id === 'trefoil' || id === 'figure-eight' ? { restart: 0, stall: 50 } : {}, seed, 20_000).log.records;
      const n = recs.length / REC, g = new Grouper(S), ends: number[] = [];
      // stream in random-sized chunks, as the worker does
      for (let o = 0; o < n;) { const k = Math.min(n - o, 1 + Math.floor(r() * 400)); ends.push(...g.push(recs.subarray(o * REC, (o + k) * REC))); o += k; }
      ends.push(...g.finish());
      expect(ends[ends.length - 1]).toBe(n);
      for (let k = 1; k < ends.length; k++) expect(ends[k]).toBeGreaterThan(ends[k - 1]);
      const t = checkGroups(S, recs, ends, `${id} seed ${seed}`);
      groups += t.groups; moves += t.moves;
    }
  }
  console.log(`${groups} groups, ${(moves / groups).toFixed(1)} moves per group`);
  expect(moves / groups).toBeGreaterThan(2);
}, 600_000);

/** Length-weighted mean angle (degrees) between a vertex's displacements in consecutive groups: 90+ is zig-zag. */
function meanTurn(S0: Scene, recs: Float64Array, ends: number[]) {
  const S = cloneScene(S0), ids = S0.map((P) => Array.from({ length: P.length / 3 }, (_, i) => i)), prev = new Map<string, number[]>();
  let w = 0, a = 0, r = 0;
  for (const e of ends) {
    const before = S.map((P) => P.slice());
    const keys = ids.map((I) => I.slice());
    for (; r < e; r++) {
      const op = recs[r * REC], c = recs[r * REC + 1], v = recs[r * REC + 2];
      if (op === Op.Delete) ids[c].splice(v, 1);
      else if (op === Op.Insert) ids[c].splice(v, 0, 1e6 + r); // a fresh id
      replay(S, recs, r, r + 1);
    }
    keys.forEach((I, c) => I.forEach((id, k) => {
      const j = ids[c].indexOf(id);
      if (j < 0) return;
      const d = [0, 1, 2].map((x) => S[c][3 * j + x] - before[c][3 * k + x]), l = Math.hypot(...d), p = prev.get(`${c}:${id}`);
      if (!l) return;
      if (p) { const lp = Math.hypot(...p), m = Math.min(l, lp); w += m; a += m * Math.acos(Math.max(-1, Math.min(1, (p[0] * d[0] + p[1] * d[1] + p[2] * d[2]) / (l * lp)))); }
      prev.set(`${c}:${id}`, d);
    }));
  }
  return (a / w) * 180 / Math.PI;
}

test('shortcut playback: verified simultaneous motions, the same final Scene, far fewer groups, straighter paths', () => {
  let before = 0, after = 0, turn = 0, runs = 0;
  for (const id of CORE.filter((k) => k !== 'trefoil' && k !== 'figure-eight')) {
    const S = presetScene(id), recs = runSync(S, {}, 3, 20_000).log.records, g = new Grouper(S);
    g.push(recs); g.finish();
    const sc = shortcut(S, recs, g.ends), t = checkGroups(S, sc.records, sc.ends, `${id} shortcut`);
    expect(sc.ends[sc.ends.length - 1]).toBe(sc.records.length / REC);
    expect(t.final.map((P) => Array.from(P)), id).toEqual(replay(cloneScene(S), recs).map((P) => Array.from(P)));
    before += g.ends.length; after += sc.ends.length; turn += meanTurn(S, sc.records, sc.ends); runs++;
  }
  console.log(`shortcut: ${before} -> ${after} groups, mean turn ${(turn / runs).toFixed(0)}°`);
  expect(after / before).toBeLessThan(0.4);
  // without smoothing, vertices reverse direction between consecutive groups (about 110°)
  expect(turn / runs).toBeLessThan(75);
}, 600_000);

test('edgesClear never clears a pair the sampled checker sees touch (random, incl. near-quadratic cubics)', () => {
  const r = rand(9), p = (): P3 => [2 * r() - 1, 2 * r() - 1, 2 * r() - 1];
  let contacts = 0;
  for (let k = 0; k < 20000; k++) {
    const a0 = p(), b0 = p(), c0 = p(), d0 = p();
    const mv = (x: P3, s = 1): P3 => [x[0] + s * (2 * r() - 1), x[1] + s * (2 * r() - 1), x[2] + s * (2 * r() - 1)];
    // k % 4: both edges moving / one endpoint each fixed (makes the cubic term vanish) / tiny moves
    const s = k % 4 === 3 ? 0.05 : 1;
    const a1 = mv(a0, s), b1 = k % 4 === 1 ? b0 : mv(b0, s), c1 = mv(c0, s), d1 = k % 4 === 1 ? d0 : mv(d0, s);
    const S0 = [Float64Array.from([...a0, ...b0]), Float64Array.from([...c0, ...d0])], S1 = [Float64Array.from([...a1, ...b1]), Float64Array.from([...c1, ...d1])];
    const touch = !sampledClear(S0, S1, 512);
    if (touch) { contacts++; expect(edgesClear(a0, a1, b0, b1, c0, c1, d0, d1, 1e-5), `pair ${k}`).toBe(false); }
  }
  expect(contacts).toBeGreaterThan(500);
}, 600_000);

test('shortcut playback is smooth on screen: no single-frame jump of the displayed curve', () => {
  // one-sided Hausdorff distance between polylines, symmetrised
  const near = (a: Scene, b: Scene) => {
    let m = 0;
    a.forEach((P, c) => { const Q = b[c], n = Q.length / 3;
      for (let i = 0; i < P.length / 3; i++) { let best = Infinity;
        for (let j = 0; j < n; j++) { const k = (j + 1) % n, u = [Q[3 * k] - Q[3 * j], Q[3 * k + 1] - Q[3 * j + 1], Q[3 * k + 2] - Q[3 * j + 2]], l2 = u[0] ** 2 + u[1] ** 2 + u[2] ** 2;
          let t = l2 > 0 ? ((P[3 * i] - Q[3 * j]) * u[0] + (P[3 * i + 1] - Q[3 * j + 1]) * u[1] + (P[3 * i + 2] - Q[3 * j + 2]) * u[2]) / l2 : 0; t = Math.max(0, Math.min(1, t));
          best = Math.min(best, (Q[3 * j] + t * u[0] - P[3 * i]) ** 2 + (Q[3 * j + 1] + t * u[1] - P[3 * i + 1]) ** 2 + (Q[3 * j + 2] + t * u[2] - P[3 * i + 2]) ** 2); }
        m = Math.max(m, Math.sqrt(best)); } });
    return m;
  };
  const S = presetScene('culprit'), recs = runSync(S, {}, 7, 20_000).log.records, g = new Grouper(S);
  g.push(recs); g.finish();
  const sc = shortcut(S, recs, g.ends), t = new Trajectory(S);
  t.append(sc.records); t.addGroups(sc.ends, true); t.done = true;
  let prev = smoothShown(t.sceneAt(0).S), worst = 0, cursor;
  for (let tau = 16; tau <= t.T; tau += 16) {
    const f = t.sceneAt(tau, cursor), cur = smoothShown(f.S, f.ghost);
    cursor = f.cursor;
    worst = Math.max(worst, near(cur, prev), near(prev, cur));
    prev = cur;
  }
  console.log(`worst frame-to-frame change ${worst.toFixed(3)} over ${t.T.toFixed(0)} ms`);
  expect(worst).toBeLessThan(0.06);
}, 300_000);

test('a stalled knot plays only up to its last stick lost', () => {
  const S = presetScene('trefoil'), r = runSync(S, { restart: 0, stall: 50 }, 7, 30_000), recs = r.log.records, g = new Grouper(S);
  g.push(recs); g.finish();
  const cut = lastProgress(recs, g.ends), n = recs.length / REC;
  expect(cut).toBeGreaterThan(0);
  expect(cut).toBeLessThan(n);
  expect(g.ends).toContain(cut);
  let dels = 0; for (let q = cut; q < n; q++) if (recs[q * REC] === Op.Delete) dels++;
  expect(dels, 'no stick is lost after the cut').toBe(0);
  const at = replay(cloneScene(S), recs, 0, cut);
  expect(at[0].length / 3).toBe(r.S[0].length / 3);
});

test('strand pickup: inserts and lifts are a verified isotopy, and playback groups and shortcuts stay verified', () => {
  // no restarts, stop on stalling: the log depends only on the seed
  const S = presetScene('pz78'), r = runSync(S, { pickup: 3, restart: 0, stall: 40 }, 2, 120_000), recs = r.log.records, n = recs.length / REC;
  let picks = 0, inserts = 0;
  for (let q = 0; q < n; q++) { if (recs[q * REC] === Op.Phase && recs[q * REC + 1] === PhaseKind.pickup) picks++; if (recs[q * REC] === Op.Insert) inserts++; }
  expect(picks).toBeGreaterThan(0);
  expect(inserts).toBeGreaterThan(0);
  const iso = checkIsotopy(S, recs);
  expect(iso.passages, iso.bad.join('\n')).toBe(0);
  const g = new Grouper(S); g.push(recs); g.finish();
  const final = replay(cloneScene(S), recs).map((P) => Array.from(P));
  expect(checkGroups(S, recs, g.ends, 'pickup groups').final.map((P) => Array.from(P))).toEqual(final);
  const sc = shortcut(S, recs, g.ends);
  expect(sc.ends.length).toBeLessThan(g.ends.length);
  expect(checkGroups(S, sc.records, sc.ends, 'pickup shortcut').final.map((P) => Array.from(P))).toEqual(final);
  console.log(`pickup run: ${r.solved ? 'solved' : 'stalled'}, ${picks} pickups, ${inserts} inserts, ${g.ends.length} -> ${sc.ends.length} groups`);
}, 600_000);
