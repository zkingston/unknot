import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
sys.path.insert(0, sys.argv[3])
from extract import load_mask, arcs_from_mask, pair_endpoints

mask = load_mask(sys.argv[1])
arcs, lab, sw = arcs_from_mask(mask)
E, M = pair_endpoints(arcs, lab, sw)
fig, ax = plt.subplots(figsize=(8, 8))
ax.imshow(mask, cmap="gray_r", alpha=0.25, extent=(0, mask.shape[1], -mask.shape[0], 0))
for ai, a in enumerate(arcs):
    ax.plot(*a["xy"].T, lw=1)
    m = a["xy"][len(a["xy"]) // 2]
    ax.text(*m, str(ai), color="blue", fontsize=12)
paired = set(x for p in M for x in p)
for k, (ai, e, p, t) in enumerate(E):
    ax.plot(*p, "o", color="green" if k in paired else "red")
    ax.arrow(*p, *(t * 2 * sw), color="orange", width=0.5)
    ax.text(*(p + 8), str(k), fontsize=8)
for i, j in M:
    ax.plot([E[i][2][0], E[j][2][0]], [E[i][2][1], E[j][2][1]], "m--")
ax.set_aspect("equal")
fig.savefig(sys.argv[2], dpi=90)
print("sw", sw, "arcs", len(arcs), "E", len(E), "M", sorted(M))
for ai, a in enumerate(arcs):
    print(ai, a["closed"], len(a["xy"]), a["xy"][0].round(), a["xy"][-1].round())
