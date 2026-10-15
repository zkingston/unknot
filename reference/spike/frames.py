import json, sys
import numpy as np

def segdist(a0, a1, b0, b1):
    d1, d2, r = a1 - a0, b1 - b0, a0 - b0
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    c, b = d1 @ r, d1 @ d2
    den = a * e - b * b
    s = np.clip((b * f - c * e) / den, 0, 1) if den > 1e-12 else 0.0
    t = (b * s + f) / e
    if t < 0:
        t, s = 0.0, np.clip(-c / a, 0, 1)
    elif t > 1:
        t, s = 1.0, np.clip((b - c) / a, 0, 1)
    return np.linalg.norm(a0 + d1 * s - (b0 + d2 * t))

def clearance(P):
    n = len(P)
    m = np.inf
    for i in range(n):
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:
                continue
            m = min(m, segdist(P[i], P[(i + 1) % n], P[j], P[(j + 1) % n]))
    return m

d = [np.array(f) for f in json.load(open(sys.argv[1]))]
multi = bad = 0
for a, b in zip(d, d[1:]):
    if len(a) != len(b):
        continue
    moved = int((np.abs(a - b).sum(1) > 1e-9).sum())
    if moved > 1:
        multi += 1
        if min(clearance(a + (b - a) * t) for t in np.linspace(0, 1, 41)[1:-1]) < 1e-6:
            bad += 1
print(f"{sys.argv[1].split('/')[-1]}: transitions={len(d)-1} multi_vertex={multi} lerp_self_intersects={bad}")
