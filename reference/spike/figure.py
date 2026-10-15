import json, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
sys.path.insert(0, sys.argv[1])
from render2d import draw, proj_crossings, chaikin

S = sys.argv[1]
traj = [np.array(f) for f in json.load(open(f"{S}/out/complex-c0.traj.json"))]
picks = [0, int(len(traj) * 0.25), int(len(traj) * 0.5), int(len(traj) * 0.75), len(traj) - 1]

fig = plt.figure(figsize=(16, 7.4), facecolor="white")
gs = fig.add_gridspec(2, 6, height_ratios=[1, 1])
ax = fig.add_subplot(gs[0, 0:2])
ax.imshow(Image.open(f"{S}/in/complex.png").convert("RGBA").crop((150, 140, 560, 560)))
ax.set_title("input image (Wikipedia, Knot theory)", fontsize=10)
ax.axis("off")
ax = fig.add_subplot(gs[0, 2:4])
draw(ax, traj[0], lw=5)
ax.set_title(f"re-rendered from lifted 3D polygon ({len(traj[0])} verts)", fontsize=10)
ax = fig.add_subplot(gs[0, 4:6])
cr = [len(proj_crossings(chaikin(f, 2))) for f in traj]
ax.plot(cr, color="black", lw=1.2)
ax.set_xlabel("frame")
ax.set_ylabel("crossings in projection")
ax.set_title("diagram crossing count during planner run", fontsize=10)
for s in ("top", "right"):
    ax.spines[s].set_visible(False)
for k, i in enumerate(picks):
    ax = fig.add_subplot(gs[1, k])
    n = draw(ax, traj[i], lw=3.5)
    ax.set_title(f"frame {i}: {len(traj[i])} verts, {n} crossings", fontsize=9)
ax = fig.add_subplot(gs[1, 5])
simple = json.load(open(f"{S}/out/simple-c0.json"))
draw(ax, np.array(simple), lw=3.5)
ax.set_title("simple image, comp 0 (2 crossings)", fontsize=9)
fig.tight_layout()
fig.savefig(f"{S}/out/spike.png", dpi=110)
print("frames", len(traj), "crossings start/end", cr[0], cr[-1], "max", max(cr))
