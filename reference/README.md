# Reference material (not shipped)

Feasibility spike behind SPEC.md. Python, written to prove the pipeline, not to be copied line for line.
Port the algorithms to TypeScript as SPEC.md describes; use the outputs here as test fixtures.

Requirements: Python 3.11+, numpy, scipy, scikit-image, networkx, sympy, matplotlib, pillow.

## spike/

| File | What it does |
| --- | --- |
| `extract.py` | Raster diagram -> closed curves + crossings. Erosion-split components, skeleton longest path, tangent-matched endpoint pairing (networkx max-weight matching), Hermite gap bridges. `python3 -I extract.py in.png out.diagram.json` |
| `knot.py` | PD code + signs, Kauffman bracket -> Jones, Alexander (Fox matrix, sympy det), `lift`, Delta-move `simplify`, `clearance` |
| `render2d.py` | Polygon -> projection crossings (z decides over), gap cutting, Chaikin smoothing (unchecked), matplotlib drawing |
| `run.py` | diagram.json -> invariants, lift, simplify (eps 0.01 and full), writes `<name>-c<k>.json` |
| `trefoil_test.py` | Control: full simplify of a parametric trefoil stops at 6 verts, invariants unchanged |
| `cross.py` | Independent isotopy checker: finds strand passages under linear interpolation of a trajectory (coplanarity roots + segment intersection) |
| `frames.py` | Coarse sampled version of the same check (superseded by cross.py) |
| `bench.py` | Times the native C++ binary on the repo presets |
| `figure.py`, `debug.py` | Produce `spike.png` and the extraction debug plot |

## fixtures/

| File | Contents |
| --- | --- |
| `complex.png`, `simple.png` | The two Wikipedia "Knot theory" lead images used in the spike |
| `complex.diagram.json`, `simple.diagram.json` | Spike format (below) |
| `wikipedia-complex-unknot.knot.json`, `wikipedia-simple-unknots.knot.json` | Same diagrams in the SPEC `.knot.json` format (y-up pixel units) |
| `complex-c0.json`, `simple-c0.json`, `simple-c1.json` | Lifted + simplified polygons (eps 0.01): 80, 48, 25 verts |
| `complex-c0.traj.json` | Native planner trajectory from `complex-c0.json`, seed 1 (918 frames, legacy format) |
| `native-bench.txt` | Native timings, frame counts, nondeterminism and strand-passage measurements |

### Spike diagram format

```
{ "sw": stroke_px,
  "comps": [ { "xy": [[x, y], ...], "gap": [bool, ...], "arc": [int, ...] }, ... ],   // y-up pixels; gap = bridge sample
  "crossings": [ [under_comp, under_seg, under_t, over_comp, over_seg, over_t], ... ] }
```

## Spike shortcuts the app must not copy

- Chaikin smoothing in `render2d.py` is not Delta-checked (SPEC requires the check).
- Threshold is fixed at 0.5 (SPEC: Otsu, Sauvola for photos).
- Python loops are O(n^2) per pass; budgets in SPEC assume the TypeScript port with a grid broad phase.
