# Unknot Web

Static single-page port of the planner in `../src` (see `../SPEC.md`). Draw or import a knot diagram, watch it untangle, flip between 2D notation and a 3D ink view.

```bash
npm ci
npm run dev          # http://localhost:5173
npm run build        # dist/index.html only (fonts, workers, presets inlined)
npm test             # Vitest: kernel, planner, playback, diagram core, inputs, invariants, budgets
npx playwright test  # Chromium, Firefox, WebKit against vite preview and file://dist/index.html
```

- `GEN_GOLDENS=1 npm test` rewrites `test/fixtures/determinism.json` (seed-7 SHA-256 of each preset's Scene and MoveLog; the e2e check compares every browser to it).
- `GEN_PRESETS=1 npx vitest run test/unit/presets.gen.test.ts` regenerates the known-knot presets (3₁, 4₁, 5₁, 5₂) from their Knot Atlas PD codes, and `src/presets/invariants.json` (every preset's invariants, so loading skips them; a test fails when it is stale).
- `tools/hard-unknots.py` rebuilds `src/presets/hard-unknots.json` (needs `pip install snappy`): the classic hard unknot diagrams from Burton et al. and DeepMind's very hard unknots from `gs://gdm-unknotting`, each certified an unknot by SnapPy and stored as spherogram's orthogonal grid drawing (not deterministic between runs: a rerun can redraw presets and change the goldens).
- Sources and licences: `CREDITS.md`.
- `tools/subset-fonts.sh` rebuilds the Iosevka Term Slab subsets from the TTFs in `fonts-src/` (needs `fonttools` + `brotli`).
- Headless browsers without WebGL2 skip the 3D-only e2e checks; the no-WebGL2 path has its own test.
