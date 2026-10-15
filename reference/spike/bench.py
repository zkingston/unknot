import json, subprocess, sys, time, os, re

repo = sys.argv[1]
out = sys.argv[2]
for k in ["goeritz", "ochiai", "thistlethwaite"]:
    for p in ["heur", "rand"]:
        for s in [1, 2, 3]:
            traj = os.path.join(out, f"{k}-{p}-{s}.json")
            args = [f"{repo}/build/unknot", "-f", f"{repo}/resources/{k}.json", "-s", str(s), "-o", traj]
            if p == "rand":
                args.append("-p")
            t = time.perf_counter()
            r = subprocess.run(args, capture_output=True, text=True, timeout=60)
            dt = time.perf_counter() - t
            d = json.load(open(traj))
            iters = len(re.findall(r"^Iter", r.stdout, re.M))
            print(f"{k:15s} {p} s={s} wall={dt:5.2f}s iters={iters:4d} frames={len(d):6d} n0={len(d[0])} nf={len(d[-1])}", flush=True)
