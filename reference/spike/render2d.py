"""3D polygon -> knot diagram (projection, crossings, gaps) -> matplotlib drawing."""
import numpy as np


def chaikin(X, iters=3):
    for _ in range(iters):
        Y = np.roll(X, -1, 0)
        X = np.stack([0.75 * X + 0.25 * Y, 0.25 * X + 0.75 * Y], 1).reshape(-1, X.shape[1])
    return X


def proj_crossings(X):
    P = X[:, :2]
    n = len(P)
    a0, a1 = P, np.roll(P, -1, 0)
    r = (a1 - a0)
    out = []
    for i in range(n):
        J = np.arange(i + 2, n)
        if i == 0:
            J = J[J != n - 1]
        if len(J) == 0:
            continue
        s = r[J]
        qp = a0[J] - a0[i]
        den = r[i, 0] * s[:, 1] - r[i, 1] * s[:, 0]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (qp[:, 0] * s[:, 1] - qp[:, 1] * s[:, 0]) / den
            u = (qp[:, 0] * r[i, 1] - qp[:, 1] * r[i, 0]) / den
        hit = (np.abs(den) > 1e-14) & (t >= 0) & (t < 1) & (u >= 0) & (u < 1)
        for j, tt, uu in zip(J[hit], t[hit], u[hit]):
            zi = X[i, 2] + tt * (X[(i + 1) % n, 2] - X[i, 2])
            zj = X[j, 2] + uu * (X[(j + 1) % n, 2] - X[j, 2])
            if zi < zj:
                out.append((0, i, tt, 0, j, uu))
            else:
                out.append((0, j, uu, 0, i, tt))
    return out


def diagram_from_polygon(X):
    return dict(comps=[dict(xy=X[:, :2].tolist())], crossings=proj_crossings(X))


def draw(ax, X, lw=3.0, gap=None, smooth=3):
    Xs = chaikin(np.asarray(X, float), smooth) if smooth else np.asarray(X, float)
    P = Xs[:, :2]
    n = len(P)
    seg = np.linalg.norm(np.roll(P, -1, 0) - P, axis=1)
    s = np.r_[0, np.cumsum(seg)]
    L = s[-1]
    ext = np.ptp(P, 0).max()
    gap = gap if gap is not None else 0.035 * ext
    cuts = []
    for _, iu, tu, _, _, _ in proj_crossings(Xs):
        su = s[iu] + tu * seg[iu]
        cuts.append((su - gap, su + gap))
    keep = np.ones(n, bool)
    for a, b in cuts:
        for off in (-L, 0, L):
            keep &= ~((s[:-1] > a + off) & (s[:-1] < b + off))
    Pc = np.vstack([P, P[:1]])
    kc = np.r_[keep, keep[:1]]
    run = []
    for k in range(n + 1):
        if kc[k]:
            run.append(Pc[k])
        elif run:
            if len(run) > 1:
                ax.plot(*np.array(run).T, color="black", lw=lw, solid_capstyle="round")
            run = []
    if len(run) > 1:
        ax.plot(*np.array(run).T, color="black", lw=lw, solid_capstyle="round")
    ax.set_aspect("equal")
    ax.axis("off")
    return len(cuts)
