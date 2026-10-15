import json, sys, time
import numpy as np
sys.path.insert(0, sys.argv[2])
from knot import pd_and_signs, kauffman_jones, alexander, lift, simplify, clearance

diag = json.load(open(sys.argv[1]))
name = sys.argv[1].split("/")[-1].split(".")[0]
for ci, comp in enumerate(diag["comps"]):
    pd, signs, ev = pd_and_signs(diag, ci)
    inter = [x for x in diag["crossings"] if (x[0] == ci) != (x[3] == ci)]
    print(f"[{name} comp {ci}] crossings={len(pd)} inter-component={len(inter)} writhe={sum(signs)}")
    print("  PD:", " ".join(f"X[{a},{b},{c},{d}]" for a, b, c, d in pd) or "(none)")
    print("  Alexander:", alexander(diag, ci))
    print("  Jones:", kauffman_jones(pd, signs))
    X = lift(diag, ci)
    t0 = time.perf_counter()
    c0 = clearance(X[:: max(1, len(X) // 300)]) if len(X) > 300 else clearance(X)
    Y = simplify(X, tol=0.01)
    Yf = simplify(Y)
    print(f"  full simplify: {len(Yf)} verts")
    dt = time.perf_counter() - t0
    print(f"  lift: {len(X)} verts (clearance~{c0:.3f}) -> simplified {len(Y)} verts, clearance={clearance(Y):.3f}, {dt:.2f}s")
    out = f"{sys.argv[3]}/{name}-c{ci}.json"
    json.dump(Y.tolist(), open(out, "w"))
    print("  wrote", out)
