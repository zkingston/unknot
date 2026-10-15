"""Diagram -> PD / invariants; diagram -> 3D polygon (lift) + Delta-move simplification."""
import itertools
import numpy as np
import sympy as sp


def component_events(diag, ci):
    """Events on component ci: (pos, kind, crossing_id), kind 'U' or 'O'."""
    ev = []
    for x, (cu, iu, tu, co, io, to) in enumerate(diag["crossings"]):
        if cu == ci:
            ev.append((iu + tu, "U", x))
        if co == ci:
            ev.append((io + to, "O", x))
    return sorted(ev)


def seg_dir(P, i):
    return P[(i + 1) % len(P)] - P[i]


def pd_and_signs(diag, ci):
    P = np.asarray(diag["comps"][ci]["xy"])
    ev = component_events(diag, ci)
    m = len(ev)
    pos_of = {}
    for k, (_, kind, x) in enumerate(ev):
        pos_of[(x, kind)] = k
    pd, signs, arcs = [], [], []
    xs = sorted({x for _, _, x in ev})
    for x in xs:
        cu, iu, tu, co, io, to = diag["crossings"][x]
        ku, ko = pos_of[(x, "U")], pos_of[(x, "O")]
        u, o = seg_dir(P, iu), seg_dir(P, io)
        positive = (u[0] * o[1] - u[1] * o[0]) < 0
        a, b = (ku - 1) % m, ku
        oi, oo = (ko - 1) % m, ko
        pd.append((a, oo, b, oi) if positive else (a, oi, b, oo))
        signs.append(1 if positive else -1)
    return pd, signs, ev


def kauffman_jones(pd, signs):
    A, t = sp.symbols("A t")
    if not pd:
        return sp.Integer(1)
    labels = sorted({e for X in pd for e in X})
    idx = {e: i for i, e in enumerate(labels)}
    d = -A**2 - A**-2
    total = 0
    for state in itertools.product((0, 1), repeat=len(pd)):
        par = list(range(len(labels)))

        def f(i):
            while par[i] != i:
                par[i] = par[par[i]]
                i = par[i]
            return i

        def u(a, b):
            par[f(idx[a])] = f(idx[b])

        for (a, b, c, e), s in zip(pd, state):
            if s == 0:
                u(a, b); u(c, e)
            else:
                u(a, e); u(b, c)
        loops = len({f(i) for i in range(len(labels))})
        na = state.count(0)
        total += A**(na - (len(pd) - na)) * d**(loops - 1)
    w = sum(signs)
    V = sp.expand((-A**3)**(-w) * total)
    return sp.simplify(V.subs(A, t**sp.Rational(-1, 4)))


def alexander(diag, ci):
    P = np.asarray(diag["comps"][ci]["xy"])
    ev = component_events(diag, ci)
    n = sum(1 for e in ev if e[1] == "U")
    if n == 0:
        return sp.Integer(1)
    t = sp.symbols("t")
    under_pos = [k for k, e in enumerate(ev) if e[1] == "U"]

    def arc_at(k):  # arc index of the strand just after event k
        return sum(1 for q in under_pos if q <= k) % n

    M = sp.zeros(n, n)
    xs = sorted({e[2] for e in ev if e[1] == "U"})
    for r, x in enumerate(xs):
        cu, iu, tu, co, io, to = diag["crossings"][x]
        ku = next(k for k, e in enumerate(ev) if e[2] == x and e[1] == "U")
        ko = next(k for k, e in enumerate(ev) if e[2] == x and e[1] == "O")
        kk, ii, jj = arc_at(ko), arc_at((ku - 1) % len(ev)), arc_at(ku)
        u, o = seg_dir(P, iu), seg_dir(P, io)
        positive = (u[0] * o[1] - u[1] * o[0]) < 0
        M[r, kk] += 1 - t
        M[r, ii] += t if positive else -1
        M[r, jj] += -1 if positive else t
    D = sp.expand(M[:-1, :-1].det())
    if D == 0:
        return sp.Integer(0)
    p = sp.Poly(sp.expand(D * t**(2 * n)), t)
    low = min(m[0] for m in p.monoms())
    D = sp.expand(p.as_expr() / t**low)
    lc = sp.Poly(D, t).LC()
    return sp.expand(D / lc * abs(lc) / abs(lc)) if lc > 0 else sp.expand(-D)


def lift(diag, ci, h=0.35):
    P = np.asarray(diag["comps"][ci]["xy"], float)
    ev = component_events(diag, ci)
    pts, z_known = [], []
    evi = 0
    n = len(P)
    for i in range(n):
        pts.append(P[i]); z_known.append(None)
        while evi < len(ev) and int(ev[evi][0]) == i:
            f = ev[evi][0] - i
            pts.append(P[i] + f * (P[(i + 1) % n] - P[i]))
            z_known.append(1.0 if ev[evi][1] == "O" else -1.0)
            evi += 1
    Q = np.array(pts)
    seg = np.linalg.norm(np.roll(Q, -1, 0) - Q, axis=1)
    s = np.r_[0, np.cumsum(seg)[:-1]]
    L = seg.sum()
    K = [k for k, z in enumerate(z_known) if z is not None]
    z = np.zeros(len(Q))
    if K:
        ks = np.array(K)
        zs = np.array([z_known[k] for k in K])
        sk = s[ks]
        sx = np.r_[sk - L, sk, sk + L]
        zx = np.r_[zs, zs, zs]
        z = np.interp(s, sx, zx)
    c = (Q.max(0) + Q.min(0)) / 2
    sc = np.abs(Q - c).max()
    X = np.c_[(Q - c) / sc, z * h]
    return X


def seg_seg_dist(a0, a1, B0, B1):
    """Distance from segment a0a1 to many segments B0B1 (vectorized)."""
    d1 = a1 - a0
    d2 = B1 - B0
    r = a0[None] - B0
    a = d1 @ d1
    e = np.einsum("ij,ij->i", d2, d2)
    f = np.einsum("ij,ij->i", d2, r)
    c = r @ d1
    b = d2 @ d1
    den = a * e - b * b
    s = np.where(den > 1e-14, np.clip((b * f - c * e) / np.where(den > 1e-14, den, 1), 0, 1), 0.0)
    tt = (b * s + f) / np.maximum(e, 1e-14)
    s = np.where(tt < 0, np.clip(-c / a, 0, 1), np.where(tt > 1, np.clip((b - c) / a, 0, 1), s))
    tt = np.clip(tt, 0, 1)
    return np.linalg.norm(a0[None] + d1[None] * s[:, None] - (B0 + d2 * tt[:, None]), axis=1)


def clearance(X):
    n = len(X)
    m = np.inf
    for i in range(n):
        J = np.array([j for j in range(n) if min((i - j) % n, (j - i) % n) > 1])
        if len(J) == 0:
            continue
        m = min(m, seg_seg_dist(X[i], X[(i + 1) % n], X[J], X[(J + 1) % n]).min())
    return m


def tri_pierced(a, b, c, S0, S1, eps=1e-9):
    e1, e2 = b - a, c - a
    d = S1 - S0
    p = np.cross(d, e2)
    det = p @ e1
    ok = np.abs(det) > eps
    inv = np.where(ok, 1 / np.where(ok, det, 1), 0)
    tv = S0 - a
    u = np.einsum("ij,ij->i", tv, p) * inv
    q = np.cross(tv, e1)
    v = (q @ d.T).diagonal() * inv if False else np.einsum("ij,ij->i", d, q) * inv
    w = (q @ e2) * inv
    return np.any(ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (w >= 0) & (w <= 1))


def simplify(X, margin=0.01, keep=3, tol=None, near=3):
    """Greedy Delta-move vertex removal (isotopy-preserving). tol: max deviation of removed vertex (shape fidelity)."""
    X = [np.asarray(p) for p in X]
    changed = True
    while changed and len(X) > keep:
        changed = False
        n = len(X)
        A = np.array(X)
        prv, nxt = np.roll(A, 1, 0), np.roll(A, -1, 0)
        v1, v2 = prv - A, nxt - A
        cosang = np.einsum("ij,ij->i", v1, v2) / (np.linalg.norm(v1, axis=1) * np.linalg.norm(v2, axis=1) + 1e-12)
        order = np.argsort(cosang)  # straightest first
        removed = set()
        for i in order:
            if len(X) - len(removed) <= keep:
                break
            if i in removed or (i - 1) % n in removed or (i + 1) % n in removed:
                continue
            a, b, c = A[(i - 1) % n], A[i], A[(i + 1) % n]
            if tol is not None:
                ac = c - a
                tt = np.clip((b - a) @ ac / max(ac @ ac, 1e-14), 0, 1)
                if np.linalg.norm(a + tt * ac - b) > tol:
                    continue
            # segments in current (partially removed) polygon, excluding the two edges at b
            live = [j for j in range(n) if j not in removed]
            pos = {j: k for k, j in enumerate(live)}
            L = len(live)
            segs, nearflag = [], []
            for k, j in enumerate(live):
                jn = live[(k + 1) % L]
                if j in ((i - 1) % n, i):
                    continue
                segs.append((j, jn))
                dk = min((k - pos[i]) % L, (pos[i] - k) % L)
                nearflag.append(dk <= near)
            S0 = np.array([A[j] for j, _ in segs])
            S1 = np.array([A[jn] for _, jn in segs])
            # shrink segments that share an endpoint with the triangle
            share0 = np.array([j in ((i - 1) % n, (i + 1) % n) for j, _ in segs])
            share1 = np.array([jn in ((i - 1) % n, (i + 1) % n) for _, jn in segs])
            S0s = np.where(share0[:, None], S0 + 1e-6 * (S1 - S0), S0)
            S1s = np.where(share1[:, None], S1 + 1e-6 * (S0 - S1), S1)
            if tri_pierced(a, b, c, S0s, S1s):
                continue
            nonadj = ~(share0 | share1) & ~np.array(nearflag)
            if nonadj.any() and seg_seg_dist(a, c, S0[nonadj], S1[nonadj]).min() < margin:
                continue
            removed.add(i)
            changed = True
        X = [X[i] for i in range(n) if i not in removed]
    return np.array(X)
