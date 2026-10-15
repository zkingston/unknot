#!/usr/bin/env bash
# Subset Iosevka Term Slab v34.9.0 (SIL OFL 1.1) to WOFF2. Run once; commit the output.
# Source TTFs: PkgTTF-Unhinted-IosevkaTermSlab-34.9.0.zip from
# https://github.com/be5invis/Iosevka/releases/tag/v34.9.0, unzipped into web/fonts-src/.
set -euo pipefail
cd "$(dirname "$0")/.."
SRC=${1:-fonts-src}
OUT=src/fonts
# Basic Latin, Latin-1, general punctuation, superscripts, arrows, math, Greek letters used in copy.
U="U+0020-007E,U+00A0-00FF,U+2010-2027,U+2030-203A,U+2070-2079,U+207A-207E,U+2190-2195,U+2212,U+2215,U+221A,U+221E,U+2248,U+2260,U+2264,U+2265,U+220E,U+2022,U+0394,U+03B5,U+03C3,U+03C4,U+03B8,U+03C0,U+25A0,U+25A1,U+25CB,U+25CF"
for style in Regular Italic Bold; do
  pyftsubset "$SRC/IosevkaTermSlab-$style.ttf" --unicodes="$U" --flavor=woff2 \
    --layout-features='kern,liga,calt' --no-hinting --desubroutinize \
    --output-file="$OUT/IosevkaTermSlab-$style.subset.woff2"
done
ls -l "$OUT"/*.woff2
