"""Control: lift + full simplify on a trefoil must not unknot it (stick number 6, Jones != 1)."""
import sys
import numpy as np
sys.path.insert(0, sys.argv[1])
from knot import pd_and_signs, kauffman_jones, alexander, simplify, clearance
from render2d import diagram_from_polygon

t = np.linspace(0, 2 * np.pi, 400, endpoint=False)
xy = np.c_[np.sin(t) + 2 * np.sin(2 * t), np.cos(t) - 2 * np.cos(2 * t)]
z = np.sin(3 * t) * 0.6  # standard trefoil embedding
X = np.c_[xy / 3, z]
for name, tol in (("shape-preserving tol=0.01", 0.01), ("full", None)):
    Y = simplify(X, tol=tol)
    d = diagram_from_polygon(Y)
    pd, signs, _ = pd_and_signs(d, 0)
    print(f"trefoil {name}: {len(X)} -> {len(Y)} verts, clearance={clearance(Y):.3f}, proj crossings={len(pd)}, "
          f"Alexander={alexander(d, 0)}, Jones={kauffman_jones(pd, signs)}")
