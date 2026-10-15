"""Raster knot diagram -> closed 2D curve(s) + crossings (over/under)."""
import json, sys
from collections import deque
import numpy as np
import networkx as nx
from PIL import Image
from scipy import ndimage
from skimage.morphology import skeletonize


def load_mask(path, max_dim=1000):
    im = Image.open(path).convert("RGBA")
    s = max(im.size) / max_dim
    if s > 1:
        im = im.resize((round(im.size[0] / s), round(im.size[1] / s)), Image.LANCZOS)
    a = np.asarray(im).astype(float) / 255
    gray = (a[..., :3].mean(-1) * a[..., 3] + (1 - a[..., 3]))
    return gray < 0.5


NB = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def skel_graph(sk):
    pts = set(zip(*np.nonzero(sk)))
    adj = {p: [(p[0] + dy, p[1] + dx) for dy, dx in NB if (p[0] + dy, p[1] + dx) in pts] for p in pts}
    return adj


def bfs_far(adj, src):
    prev = {src: None}
    q = deque([src])
    last = src
    while q:
        last = q.popleft()
        for n in adj[last]:
            if n not in prev:
                prev[n] = last
                q.append(n)
    path = [last]
    while prev[path[-1]] is not None:
        path.append(prev[path[-1]])
    return path


def trace_cycle(adj):
    start = next(iter(adj))
    path, prev, cur = [start], None, start
    while True:
        nxt = [n for n in adj[cur] if n != prev and n not in path[1:]]
        if not nxt:
            break
        prev, cur = cur, nxt[0]
        if cur == start:
            break
        path.append(cur)
    return path


def arcs_from_mask(mask):
    sw = 2 * np.median(ndimage.distance_transform_edt(mask)[skeletonize(mask)])
    er = ndimage.binary_erosion(mask, iterations=max(1, int(round(sw * 0.2))))
    lab, n = ndimage.label(er, structure=np.ones((3, 3)))
    while True:  # split components whose skeleton branches (touching strands) by eroding further
        bad = []
        for k in range(1, n + 1):
            adj = skel_graph(skeletonize(lab == k))
            if sum(len(v) == 1 for v in adj.values()) > 2:
                bad.append(k)
        if not bad or er.sum() == 0:
            break
        er = ndimage.binary_erosion(er)
        lab, n = ndimage.label(er, structure=np.ones((3, 3)))
    arcs = []
    for k in range(1, n + 1):
        comp = lab == k
        if comp.sum() < sw * sw:
            continue
        adj = skel_graph(skeletonize(comp))
        if not adj:
            continue
        ends = [p for p, v in adj.items() if len(v) == 1]
        if ends:
            path = bfs_far(adj, bfs_far(adj, ends[0])[0])
            closed = False
        else:
            path = trace_cycle(adj)
            closed = True
        xy = np.array([(c, -r) for r, c in path], float)  # y up
        arcs.append(dict(xy=xy, closed=closed, label=k))
    return arcs, lab, sw


def endpoint_tangent(xy, end, L):
    pts = xy if end == 1 else xy[::-1]
    d = np.cumsum(np.r_[0, np.linalg.norm(np.diff(pts[::-1], axis=0), axis=1)])
    j = min(np.searchsorted(d, L), len(pts) - 1)
    t = pts[-1] - pts[::-1][j]
    return pts[-1], t / (np.linalg.norm(t) + 1e-12)


def line_hits_ink(lab, p, q, skip=()):
    ts = np.linspace(0.2, 0.8, 25)
    pts = p[None] + ts[:, None] * (q - p)[None]
    r, c = np.round(-pts[:, 1]).astype(int), np.round(pts[:, 0]).astype(int)
    ok = (r >= 0) & (r < lab.shape[0]) & (c >= 0) & (c < lab.shape[1])
    vals = lab[r[ok], c[ok]]
    return bool(np.any((vals > 0) & ~np.isin(vals, skip)))


def pair_endpoints(arcs, lab, sw):
    E = []
    for ai, a in enumerate(arcs):
        if a["closed"]:
            continue
        for end in (0, 1):
            p, t = endpoint_tangent(a["xy"], end, 2.0 * sw)
            E.append((ai, end, p, t))
    G = nx.Graph()
    for i in range(len(E)):
        for j in range(i + 1, len(E)):
            ai, ei, pi, ti = E[i]
            aj, ej, pj, tj = E[j]
            v = pj - pi
            d = np.linalg.norm(v)
            if d > 14 * sw or d < 1e-6:
                continue
            ci, cj = ti @ v / d, -(tj @ v) / d
            if ci < 0.4 or cj < 0.4:
                continue
            if not line_hits_ink(lab, pi, pj):
                continue
            cost = d / sw + 4 * (2 - ci - cj)
            G.add_edge(i, j, weight=100 - cost)
    M = nx.max_weight_matching(G, maxcardinality=True)
    return E, M


def hermite(p0, t0, p1, t1, n=24):
    d = np.linalg.norm(p1 - p0)
    m0, m1 = t0 * d, -t1 * d
    s = np.linspace(0, 1, n)[1:-1, None]
    h00, h10, h01, h11 = 2 * s**3 - 3 * s**2 + 1, s**3 - 2 * s**2 + s, -2 * s**3 + 3 * s**2, s**3 - s**2
    return h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1


def assemble(arcs, E, M):
    partner = {}
    for i, j in M:
        partner[i] = j
        partner[j] = i
    eidx = {(e[0], e[1]): k for k, e in enumerate(E)}
    used, comps = set(), []
    for ai, a in enumerate(arcs):
        if ai in used:
            continue
        if a["closed"]:
            used.add(ai)
            comps.append(dict(xy=a["xy"], gap=np.zeros(len(a["xy"]), bool), arc=np.full(len(a["xy"]), ai)))
            continue
        pts, gap, arcid = [], [], []
        cur, entry = ai, 0
        while True:
            used.add(cur)
            xy = arcs[cur]["xy"] if entry == 0 else arcs[cur]["xy"][::-1]
            pts.append(xy); gap.append(np.zeros(len(xy), bool)); arcid.append(np.full(len(xy), cur))
            k = eidx[(cur, 1 - entry)]
            if k not in partner:
                raise RuntimeError(f"unpaired endpoint on arc {cur}")
            nk = partner[k]
            na, ne = E[nk][0], E[nk][1]
            h = hermite(E[k][2], E[k][3], E[nk][2], E[nk][3])
            pts.append(h); gap.append(np.ones(len(h), bool)); arcid.append(np.full(len(h), -1))
            if na == ai and ne == 0:
                break
            cur, entry = na, ne
        comps.append(dict(xy=np.vstack(pts), gap=np.concatenate(gap), arc=np.concatenate(arcid)))
    return comps


def resample(comp, step):
    xy, gap, arc = comp["xy"], comp["gap"], comp["arc"]
    seg = np.linalg.norm(np.diff(np.vstack([xy, xy[:1]]), axis=0), axis=1)
    s = np.r_[0, np.cumsum(seg)]
    L = s[-1]
    m = max(int(L / step), 12)
    u = np.linspace(0, L, m, endpoint=False)
    idx = np.searchsorted(s, u, side="right") - 1
    f = ((u - s[idx]) / np.maximum(seg[idx], 1e-12))[:, None]
    nxt = (idx + 1) % len(xy)
    out = xy[idx] * (1 - f) + xy[nxt] * f
    return dict(xy=out, gap=gap[idx] | gap[nxt], arc=np.where(gap[idx], arc[nxt], arc[idx]))


def seg_intersections(P, Q):
    """All proper intersections between closed polylines P (segments i) and Q (segments j)."""
    a0, a1 = P, np.roll(P, -1, 0)
    b0, b1 = Q, np.roll(Q, -1, 0)
    r = (a1 - a0)[:, None]
    s = (b1 - b0)[None]
    qp = b0[None] - a0[:, None]
    den = r[..., 0] * s[..., 1] - r[..., 1] * s[..., 0]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (qp[..., 0] * s[..., 1] - qp[..., 1] * s[..., 0]) / den
        u = (qp[..., 0] * r[..., 1] - qp[..., 1] * r[..., 0]) / den
    hit = (np.abs(den) > 1e-12) & (t >= 0) & (t < 1) & (u >= 0) & (u < 1)
    I, J = np.nonzero(hit)
    return I, J, t[I, J], u[I, J]


def find_crossings(comps):
    """Crossings across all components; returns list of (ci, i, ti, cj, j, tj, kind) with (ci,i) the under."""
    out, flat = [], 0
    for ci, A in enumerate(comps):
        for cj, B in enumerate(comps):
            if cj < ci:
                continue
            I, J, T, U = seg_intersections(A["xy"], B["xy"])
            nA, nB = len(A["xy"]), len(B["xy"])
            for i, j, t, u in zip(I, J, T, U):
                if ci == cj:
                    if j <= i or min((i - j) % nA, (j - i) % nA) <= 1:
                        continue
                ga, gb = A["gap"][i], B["gap"][j]
                if ga and not gb:
                    out.append((ci, i, t, cj, j, u))
                elif gb and not ga:
                    out.append((cj, j, u, ci, i, t))
                else:
                    flat += 1
    return out, flat


if __name__ == "__main__":
    path, outp = sys.argv[1], sys.argv[2]
    mask = load_mask(path)
    arcs, lab, sw = arcs_from_mask(mask)
    E, M = pair_endpoints(arcs, lab, sw)
    comps = [resample(c, sw * 0.5) for c in assemble(arcs, E, M)]
    X, flat = find_crossings(comps)
    print(f"{path.split('/')[-1]}: stroke={sw:.1f}px arcs={len(arcs)} endpoints={len(E)} pairs={len(M)} "
          f"components={len(comps)} crossings={len(X)} flat={flat}")
    json.dump(dict(sw=sw, comps=[dict(xy=c["xy"].tolist(), gap=c["gap"].tolist(), arc=c["arc"].tolist()) for c in comps],
                   crossings=[[int(a), int(b), float(c), int(d), int(e), float(f)] for a, b, c, d, e, f in X]),
              open(outp, "w"))
