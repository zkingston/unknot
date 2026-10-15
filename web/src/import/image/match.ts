// Maximum-weight matching on a general graph: Edmonds' blossom algorithm (J. Edmonds 1965), primal-dual O(n³),
// a direct port of Joris van Rantwijk's mwmatching.py (the algorithm networkx max_weight_matching uses).

/** edges: [i, j, weight]; returns mate[v] (vertex matched to v, or -1). */
export function maxWeightMatching(edges: [number, number, number][], maxcardinality = false): number[] {
  if (!edges.length) return [];
  const nedge = edges.length;
  let nvertex = 0;
  for (const [i, j] of edges) { if (i < 0 || j < 0 || i === j) throw new Error('bad edge'); nvertex = Math.max(nvertex, i + 1, j + 1); }
  const maxweight = Math.max(0, ...edges.map((e) => e[2]));
  const endpoint = Array.from({ length: 2 * nedge }, (_, p) => edges[p >> 1][p & 1]);
  const neighbend: number[][] = Array.from({ length: nvertex }, () => []);
  edges.forEach(([i, j], k) => { neighbend[i].push(2 * k + 1); neighbend[j].push(2 * k); });
  const mate = new Array<number>(nvertex).fill(-1);
  const label = new Array<number>(2 * nvertex).fill(0);
  const labelend = new Array<number>(2 * nvertex).fill(-1);
  const inblossom = Array.from({ length: nvertex }, (_, i) => i);
  const blossomparent = new Array<number>(2 * nvertex).fill(-1);
  const blossomchilds: (number[] | null)[] = new Array(2 * nvertex).fill(null);
  const blossombase = [...Array.from({ length: nvertex }, (_, i) => i), ...new Array<number>(nvertex).fill(-1)];
  const blossomendps: (number[] | null)[] = new Array(2 * nvertex).fill(null);
  const bestedge = new Array<number>(2 * nvertex).fill(-1);
  const blossombestedges: (number[] | null)[] = new Array(2 * nvertex).fill(null);
  const unusedblossoms = Array.from({ length: nvertex }, (_, i) => nvertex + i);
  const dualvar = [...new Array<number>(nvertex).fill(maxweight), ...new Array<number>(nvertex).fill(0)];
  const allowedge = new Array<boolean>(nedge).fill(false);
  let queue: number[] = [];
  const at = <T>(a: T[], j: number) => a[j < 0 ? j + a.length : j];

  const slack = (k: number) => { const [i, j, wt] = edges[k]; return dualvar[i] + dualvar[j] - 2 * wt; };

  const blossomLeaves = (b: number, out: number[] = []): number[] => {
    if (b < nvertex) out.push(b);
    else for (const t of blossomchilds[b]!) { if (t < nvertex) out.push(t); else blossomLeaves(t, out); }
    return out;
  };

  const assignLabel = (w: number, t: number, p: number) => {
    const b = inblossom[w];
    label[w] = label[b] = t;
    labelend[w] = labelend[b] = p;
    bestedge[w] = bestedge[b] = -1;
    if (t === 1) queue.push(...blossomLeaves(b));
    else if (t === 2) { const base = blossombase[b]; assignLabel(endpoint[mate[base]], 1, mate[base] ^ 1); }
  };

  const scanBlossom = (v: number, w: number): number => {
    const path: number[] = [];
    let base = -1;
    while (v !== -1 || w !== -1) {
      let b = inblossom[v];
      if (label[b] & 4) { base = blossombase[b]; break; }
      path.push(b);
      label[b] = 5;
      if (labelend[b] === -1) v = -1;
      else { v = endpoint[labelend[b]]; b = inblossom[v]; v = endpoint[labelend[b]]; }
      if (w !== -1) { const t = v; v = w; w = t; }
    }
    for (const b of path) label[b] = 1;
    return base;
  };

  const addBlossom = (base: number, k: number) => {
    let [v, w] = edges[k];
    const bb = inblossom[base];
    let bv = inblossom[v], bw = inblossom[w];
    const b = unusedblossoms.pop()!;
    blossombase[b] = base; blossomparent[b] = -1; blossomparent[bb] = b;
    const path: number[] = [], endps: number[] = [];
    blossomchilds[b] = path; blossomendps[b] = endps;
    while (bv !== bb) { blossomparent[bv] = b; path.push(bv); endps.push(labelend[bv]); v = endpoint[labelend[bv]]; bv = inblossom[v]; }
    path.push(bb); path.reverse(); endps.reverse(); endps.push(2 * k);
    while (bw !== bb) { blossomparent[bw] = b; path.push(bw); endps.push(labelend[bw] ^ 1); w = endpoint[labelend[bw]]; bw = inblossom[w]; }
    label[b] = 1; labelend[b] = labelend[bb]; dualvar[b] = 0;
    for (const lv of blossomLeaves(b)) { if (label[inblossom[lv]] === 2) queue.push(lv); inblossom[lv] = b; }
    const bestedgeto = new Array<number>(2 * nvertex).fill(-1);
    for (const sb of path) {
      const nblists = blossombestedges[sb] === null ? blossomLeaves(sb).map((lv) => neighbend[lv].map((p) => p >> 1)) : [blossombestedges[sb]!];
      for (const nblist of nblists) for (const kk of nblist) {
        let [i, j] = edges[kk];
        if (inblossom[j] === b) { const t = i; i = j; j = t; }
        const bj = inblossom[j];
        if (bj !== b && label[bj] === 1 && (bestedgeto[bj] === -1 || slack(kk) < slack(bestedgeto[bj]))) bestedgeto[bj] = kk;
      }
      blossombestedges[sb] = null; bestedge[sb] = -1;
    }
    blossombestedges[b] = bestedgeto.filter((kk) => kk !== -1);
    bestedge[b] = -1;
    for (const kk of blossombestedges[b]!) if (bestedge[b] === -1 || slack(kk) < slack(bestedge[b])) bestedge[b] = kk;
  };

  const expandBlossom = (b: number, endstage: boolean) => {
    for (const s of blossomchilds[b]!) {
      blossomparent[s] = -1;
      if (s < nvertex) inblossom[s] = s;
      else if (endstage && dualvar[s] === 0) expandBlossom(s, endstage);
      else for (const lv of blossomLeaves(s)) inblossom[lv] = s;
    }
    if (!endstage && label[b] === 2) {
      const ch = blossomchilds[b]!, ep = blossomendps[b]!;
      const entrychild = inblossom[endpoint[labelend[b] ^ 1]];
      let j = ch.indexOf(entrychild), jstep: number, endptrick: number;
      if (j & 1) { j -= ch.length; jstep = 1; endptrick = 0; } else { jstep = -1; endptrick = 1; }
      let p = labelend[b];
      while (j !== 0) {
        label[endpoint[p ^ 1]] = 0;
        label[endpoint[at(ep, j - endptrick) ^ endptrick ^ 1]] = 0;
        assignLabel(endpoint[p ^ 1], 2, p);
        allowedge[at(ep, j - endptrick) >> 1] = true;
        j += jstep;
        p = at(ep, j - endptrick) ^ endptrick;
        allowedge[p >> 1] = true;
        j += jstep;
      }
      let bv = at(ch, j);
      label[endpoint[p ^ 1]] = label[bv] = 2;
      labelend[endpoint[p ^ 1]] = labelend[bv] = p;
      bestedge[bv] = -1;
      j += jstep;
      while (at(ch, j) !== entrychild) {
        bv = at(ch, j);
        if (label[bv] === 1) { j += jstep; continue; }
        let lv = -1;
        for (const x of blossomLeaves(bv)) { lv = x; if (label[x] !== 0) break; }
        if (label[lv] !== 0) {
          label[lv] = 0;
          label[endpoint[mate[blossombase[bv]]]] = 0;
          assignLabel(lv, 2, labelend[lv]);
        }
        j += jstep;
      }
    }
    label[b] = labelend[b] = -1;
    blossomchilds[b] = blossomendps[b] = null;
    blossombase[b] = -1; blossombestedges[b] = null; bestedge[b] = -1;
    unusedblossoms.push(b);
  };

  const augmentBlossom = (b: number, v: number) => {
    let t = v;
    while (blossomparent[t] !== b) t = blossomparent[t];
    if (t >= nvertex) augmentBlossom(t, v);
    const ch = blossomchilds[b]!, ep = blossomendps[b]!;
    const i = ch.indexOf(t);
    let j = i, jstep: number, endptrick: number;
    if (i & 1) { j -= ch.length; jstep = 1; endptrick = 0; } else { jstep = -1; endptrick = 1; }
    while (j !== 0) {
      j += jstep;
      t = at(ch, j);
      const p = at(ep, j - endptrick) ^ endptrick;
      if (t >= nvertex) augmentBlossom(t, endpoint[p]);
      j += jstep;
      t = at(ch, j);
      if (t >= nvertex) augmentBlossom(t, endpoint[p ^ 1]);
      mate[endpoint[p]] = p ^ 1;
      mate[endpoint[p ^ 1]] = p;
    }
    blossomchilds[b] = [...ch.slice(i), ...ch.slice(0, i)];
    blossomendps[b] = [...ep.slice(i), ...ep.slice(0, i)];
    blossombase[b] = blossombase[blossomchilds[b]![0]];
  };

  const augmentMatching = (k: number) => {
    const [v, w] = edges[k];
    for (let [s, p] of [[v, 2 * k + 1], [w, 2 * k]]) {
      for (;;) {
        const bs = inblossom[s];
        if (bs >= nvertex) augmentBlossom(bs, s);
        mate[s] = p;
        if (labelend[bs] === -1) break;
        const t = endpoint[labelend[bs]], bt = inblossom[t];
        s = endpoint[labelend[bt]];
        const j = endpoint[labelend[bt] ^ 1];
        if (bt >= nvertex) augmentBlossom(bt, j);
        mate[j] = labelend[bt];
        p = labelend[bt] ^ 1;
      }
    }
  };

  for (let stage = 0; stage < nvertex; stage++) {
    label.fill(0); bestedge.fill(-1);
    for (let b = nvertex; b < 2 * nvertex; b++) blossombestedges[b] = null;
    allowedge.fill(false);
    queue = [];
    for (let v = 0; v < nvertex; v++) if (mate[v] === -1 && label[inblossom[v]] === 0) assignLabel(v, 1, -1);
    let augmented = false;
    for (;;) {
      while (queue.length && !augmented) {
        const v = queue.pop()!;
        for (const p of neighbend[v]) {
          const k = p >> 1, w = endpoint[p];
          if (inblossom[v] === inblossom[w]) continue;
          let kslack = 0;
          if (!allowedge[k]) { kslack = slack(k); if (kslack <= 0) allowedge[k] = true; }
          if (allowedge[k]) {
            if (label[inblossom[w]] === 0) assignLabel(w, 2, p ^ 1);
            else if (label[inblossom[w]] === 1) {
              const base = scanBlossom(v, w);
              if (base >= 0) addBlossom(base, k);
              else { augmentMatching(k); augmented = true; break; }
            } else if (label[w] === 0) { label[w] = 2; labelend[w] = p ^ 1; }
          } else if (label[inblossom[w]] === 1) {
            const b = inblossom[v];
            if (bestedge[b] === -1 || kslack < slack(bestedge[b])) bestedge[b] = k;
          } else if (label[w] === 0) {
            if (bestedge[w] === -1 || kslack < slack(bestedge[w])) bestedge[w] = k;
          }
        }
      }
      if (augmented) break;
      let deltatype = -1, delta = 0, deltaedge = -1, deltablossom = -1;
      if (!maxcardinality) { deltatype = 1; delta = Math.min(...dualvar.slice(0, nvertex)); }
      for (let v = 0; v < nvertex; v++) if (label[inblossom[v]] === 0 && bestedge[v] !== -1) {
        const d = slack(bestedge[v]);
        if (deltatype === -1 || d < delta) { delta = d; deltatype = 2; deltaedge = bestedge[v]; }
      }
      for (let b = 0; b < 2 * nvertex; b++) if (blossomparent[b] === -1 && label[b] === 1 && bestedge[b] !== -1) {
        const d = slack(bestedge[b]) / 2;
        if (deltatype === -1 || d < delta) { delta = d; deltatype = 3; deltaedge = bestedge[b]; }
      }
      for (let b = nvertex; b < 2 * nvertex; b++) if (blossombase[b] >= 0 && blossomparent[b] === -1 && label[b] === 2 && (deltatype === -1 || dualvar[b] < delta)) {
        delta = dualvar[b]; deltatype = 4; deltablossom = b;
      }
      if (deltatype === -1) { deltatype = 1; delta = Math.max(0, Math.min(...dualvar.slice(0, nvertex))); }
      for (let v = 0; v < nvertex; v++) {
        if (label[inblossom[v]] === 1) dualvar[v] -= delta;
        else if (label[inblossom[v]] === 2) dualvar[v] += delta;
      }
      for (let b = nvertex; b < 2 * nvertex; b++) if (blossombase[b] >= 0 && blossomparent[b] === -1) {
        if (label[b] === 1) dualvar[b] += delta; else if (label[b] === 2) dualvar[b] -= delta;
      }
      if (deltatype === 1) break;
      else if (deltatype === 2) {
        allowedge[deltaedge] = true;
        let [i, j] = edges[deltaedge];
        if (label[inblossom[i]] === 0) { const t = i; i = j; j = t; }
        queue.push(i);
      } else if (deltatype === 3) { allowedge[deltaedge] = true; queue.push(edges[deltaedge][0]); }
      else expandBlossom(deltablossom, false);
    }
    if (!augmented) break;
    for (let b = nvertex; b < 2 * nvertex; b++) if (blossomparent[b] === -1 && blossombase[b] >= 0 && label[b] === 1 && dualvar[b] === 0) expandBlossom(b, true);
  }
  for (let v = 0; v < nvertex; v++) if (mate[v] >= 0) mate[v] = endpoint[mate[v]];
  return mate;
}
