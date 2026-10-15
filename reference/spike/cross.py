import json, sys
import numpy as np

def pairs(n):
    I, J = [], []
    for i in range(n):
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            I.append(i); J.append(j)
    return np.array(I), np.array(J)

def vol(P, I, J):
    n = len(P)
    a0, a1, b0, b1 = P[I], P[(I + 1) % n], P[J], P[(J + 1) % n]
    return np.einsum("ij,ij->i", np.cross(a1 - a0, b0 - a0), b1 - a0)

def intersects(P, i, j, tol):
    n = len(P)
    a0, a1, b0, b1 = P[i], P[(i + 1) % n], P[j], P[(j + 1) % n]
    d1, d2, r = a1 - a0, b1 - b0, a0 - b0
    a, e, f, c, b = d1 @ d1, d2 @ d2, d2 @ r, d1 @ r, d1 @ d2
    den = a * e - b * b
    s = np.clip((b * f - c * e) / den, 0, 1) if den > 1e-14 else 0.0
    t = (b * s + f) / e
    if t < 0: t, s = 0.0, np.clip(-c / a, 0, 1)
    elif t > 1: t, s = 1.0, np.clip((b - c) / a, 0, 1)
    return np.linalg.norm(a0 + d1 * s - (b0 + d2 * t)) < tol

d = [np.array(f, dtype=np.float64) for f in json.load(open(sys.argv[1]))]
ts = np.linspace(0, 1, 513)
multi = crossings = frames_bad = 0
for A, B in zip(d, d[1:]):
    if len(A) != len(B):
        continue
    if int((np.abs(A - B).sum(1) > 1e-9).sum()) <= 1:
        continue
    multi += 1
    I, J = pairs(len(A))
    V = np.stack([vol(A + (B - A) * t, I, J) for t in ts])
    hit = 0
    for k in np.nonzero((np.sign(V[:-1]) * np.sign(V[1:]) < 0).any(1))[0]:
        for p in np.nonzero(np.sign(V[k]) * np.sign(V[k + 1]) < 0)[0]:
            lo, hi = ts[k], ts[k + 1]
            f = lambda t: vol(A + (B - A) * t, I[p:p+1], J[p:p+1])[0]
            flo = f(lo)
            for _ in range(60):
                mid = 0.5 * (lo + hi)
                if np.sign(f(mid)) == np.sign(flo): lo = mid
                else: hi = mid
            if intersects(A + (B - A) * lo, I[p], J[p], 1e-5):
                hit += 1
    crossings += hit
    frames_bad += hit > 0
print(f"{sys.argv[1].split('/')[-1]}: multi_vertex={multi} frames_with_strand_passage={frames_bad} passages={crossings}")
