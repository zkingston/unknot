#!/usr/bin/env python3
"""Build src/presets/hard-unknots.json: hard unknot diagrams as orthogonal grid layouts.

Sources (APA 7; full reference list in CREDITS.md)
- Gauss codes: Burton, B. A., Chang, H.-C., Löffler, M., Maria, C., de Mesmay, A., Schleimer, S., Sedgwick, E., &
  Spreer, J. (2024). Hard diagrams of the unknot. Experimental Mathematics, 33(3), 482–500, Appendix A.
  https://doi.org/10.1080/10586458.2022.2161676. They are unsigned (over/under only); spherogram realises each as a
  planar diagram via its DT code. Mirror images are not distinguished, which does not matter for unknots.
- PD codes: Applebaum, T., Blackwell, S., Davies, A., Edlich, T., Juhász, A., Lackenby, M., Tomašev, N., & Zheng, D.
  (2025). The unknotting number, hard unknot diagrams, and reinforcement learning. Experimental Mathematics.
  https://doi.org/10.1080/10586458.2025.2542174 (Figure 17, 42 crossings), and Google DeepMind. (2025). Very hard
  unknots [Data set], CC BY 4.0: 385 hard unknot diagrams.
- Original sources per diagram follow Burton et al. (2024), Table 1; each preset carries them as APA in-text
  citations with DOI links (CITE below).

Every diagram is certified an unknot by SnapPy (fundamental group of the exterior is Z), then drawn with
spherogram's orthogonal layout (Tamassia-style grid embedding); only the drawing is stored, not the PD code.
Changes to the CC BY data: PD codes converted to drawings; diagrams sorted by crossing number.
Run once with SnapPy installed (pip install snappy); commit the JSON. The orthogonal layout is not deterministic
between runs, so a rerun can redraw some presets (and change the determinism goldens).
"""
import csv, json, os, sys, urllib.request
import snappy  # noqa: F401  (spherogram needs it for exterior())
import spherogram
from spherogram.links.orthogonal import OrthogonalLinkDiagram

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'src', 'presets', 'hard-unknots.json')

# cite = the diagram's original source (Burton et al., Table 1) as APA in-text citations with DOI links;
# full references in CREDITS.md
CITE = {
    'KL': ('Kauffman & Lambropoulou, 2011', 'https://doi.org/10.1142/9789814313001_0009'),
    'PZ': ('Petronio & Zanellati, 2016', 'https://doi.org/10.1142/S0218216516500590'),
    'CURTIS': ('Curtis, 2001', 'http://f2.org/maths/kt/unknoteq.html'),
    'HK': ('Henrich & Kauffman, 2014', 'https://doi.org/10.4169/amer.math.monthly.121.05.379'),
    'BURTON': ('Burton et al., 2024', 'https://doi.org/10.1080/10586458.2022.2161676'),
    'TS': ('Tuzun & Sikora, 2018', 'https://doi.org/10.1142/S0218216518400096'),
    'FHW': ('Freedman et al., 1994', 'https://doi.org/10.2307/2946626'),
    'OCHIAI': ('Ochiai, 1990', 'https://www.numdam.org/item/AST_1990__192__7_0/'),
    'DEEPMIND': ('Applebaum et al., 2025', 'https://doi.org/10.1080/10586458.2025.2542174'),
}
cite = lambda *keys: [{'t': CITE[k][0], 'u': CITE[k][1]} for k in keys]
NAMED = [
    ('culprit', 'Culprit', cite('KL'),
     '-1 2 -3 4 -5 6 7 8 -9 10 -4 5 -6 3 -2 -7 -10 1 -8 9'),
    ('monster', 'Monster', cite('PZ', 'CURTIS'),
     '1 -2 3 4 5 -6 7 -8 9 -3 10 -1 2 -10 -4 -9 8 -5 6 -7'),
    ('h', 'H', cite('HK'), '-1 2 -3 4 -5 -6 7 -8 9 1 -2 3 -4 -9 6 -7 8 5'),
    ('j', 'J', cite('HK'), '-1 2 -3 4 5 -6 7 1 -4 3 -2 -5 8 -9 6 -7 9 -8'),
    ('d28', 'D28', cite('BURTON'),
     '1 -4 -3 6 5 -2 -7 8 4 -5 -9 10 2 -1 -11 7 12 -13 -6 3 14 -12 -10 9 13 -14 -8 11 15 -18 -17 20 19 -16 -21 17 22 '
     '-23 -20 21 24 -15 -25 26 16 -19 -27 28 18 -24 -26 27 23 -22 -28 25'),
    ('d43', 'D43', cite('BURTON'),
     '-1 2 -3 -4 -5 6 -7 8 -9 10 -11 12 -13 3 14 -15 16 -17 18 -19 20 1 -21 22 -23 24 -25 7 -26 27 -10 28 -29 13 4 30 '
     '31 -32 17 -33 34 35 -2 -36 -22 -37 -38 25 -8 39 -27 11 -40 29 41 23 37 -31 42 -16 33 -43 19 -20 -35 -14 -30 5 -6 '
     '38 -24 9 -39 26 -12 40 -28 -41 36 21 32 -42 15 -34 43 -18'),
    ('tuzun-sikora', 'Tuzun–Sikora', cite('TS'),
     '1 6 -7 10 11 12 -14 -15 16 -17 -18 -2 3 -4 5 7 -9 -11 17 -21 19 -20 -8 9 -10 18 21 -16 -13 14 20 -1 2 -3 4 -5 -6 '
     '8 -12 13 15 -19'),
    ('freedman-he-wang', 'Freedman–He–Wang', cite('FHW'),
     '3 4 -5 -8 10 12 -13 -15 8 7 -9 -10 15 16 -4 1 17 -20 32 31 -27 -25 22 24 -31 -29 28 27 -24 -23 20 19 -18 -17 '
     '-21 -22 25 26 -30 -32 23 21 -26 -28 29 30 -19 18 2 -3 14 13 -12 -11 6 5 -16 -14 11 9 -7 -6 -1 -2'),
    ('ochiai-2', 'Ochiai II', cite('OCHIAI'),
     '-1 2 -3 -4 5 -6 -7 8 -9 10 -11 -12 13 -14 15 16 4 -17 6 18 -19 11 20 21 -22 -23 24 25 -26 27 -28 29 14 30 -31 '
     '-13 -25 32 -33 -34 -27 28 35 36 -37 22 38 -39 12 -40 -41 42 -16 3 -2 1 -21 -38 23 37 45 -35 34 26 -29 -15 -42 43 '
     '-18 9 -8 7 17 -5 -43 41 -44 31 -30 44 40 19 -10 -20 39 -24 -32 33 -36 -45'),
    ('pz31', 'Petronio–Zanellati 31', cite('PZ'),
     '2 -3 4 -19 16 -21 -22 1 -20 26 -27 -29 21 -16 -17 7 -8 -13 -26 -25 28 27 14 -15 29 30 -31 -28 25 -24 23 31 -30 '
     '22 19 -18 6 9 11 -12 13 -14 15 17 18 5 -9 10 12 20 24 -23 -1 -2 3 -4 -5 -6 -7 8 -10 -11'),
    ('haken', 'Haken’s Gordian unknot', cite('PZ'),
     '-1 -2 3 -4 5 6 7 8 -9 -10 11 12 13 -14 15 -16 17 18 -19 20 -21 -7 22 23 24 -25 -26 -27 28 -29 30 31 -32 -33 -34 '
     '35 36 -37 -38 -39 -40 -41 42 43 -44 45 46 47 -48 -49 -50 38 -51 52 -53 -54 -55 -42 -56 57 58 -46 -59 -60 -61 '
     '-62 63 -64 -65 -66 -67 68 -35 -69 70 -71 -72 73 29 -74 -31 -75 -76 77 69 -36 -68 -78 -79 -80 64 -63 -81 -82 60 '
     '-83 -47 84 -57 -85 -43 55 -86 -87 -52 -88 37 -89 49 -90 -84 -58 -45 44 85 56 -91 -92 39 51 88 -70 -77 -93 33 '
     '-94 74 -30 -73 95 27 -96 -97 98 -23 99 -8 21 100 -101 -18 102 -103 -15 -104 105 -12 106 -107 9 -99 -22 108 -5 '
     '-109 110 2 -111 -112 113 -114 -28 -95 72 115 -116 75 32 94 117 78 -118 66 -119 120 121 83 59 10 107 -122 -123 '
     '124 103 16 125 -110 -3 126 127 -98 -24 54 86 -128 40 92 129 90 48 -121 -130 62 81 -131 104 14 132 -133 -124 '
     '-134 19 101 135 -6 -108 -127 136 137 114 -113 -138 139 -126 4 109 -135 -100 -20 140 123 133 -132 -13 -105 131 '
     '141 79 118 67 89 50 -129 91 41 128 87 53 25 97 -136 -139 111 1 -125 -17 -102 134 -140 122 -106 -11 82 61 130 '
     '-120 119 65 80 -141 -117 34 93 76 116 -115 71 26 96 -137 138 112'),
    ('fake-fhw', 'Fake Freedman–He–Wang', cite('BURTON'),
     '1 -2 -3 -4 5 6 -7 -8 4 9 -10 -5 8 11 2 12 -13 -1 14 7 -6 -15 16 3 -11 -14 15 10 -9 -16 -12 13 17 -18 -19 -20 21 '
     '22 -23 -24 25 19 -22 -26 27 23 -28 -17 18 29 24 30 -31 -21 20 32 -30 -27 26 31 -32 -25 -29 28'),
    ('ochiai-2r', 'Ochiai II reduced', cite('BURTON'),
     '1 -2 -21 -3 4 -6 -5 7 -8 -10 9 -11 12 15 -13 14 3 5 -16 17 -18 19 10 21 22 13 23 -24 20 -12 25 -26 -27 28 -30 31 '
     '29 -32 11 -20 -15 33 2 -1 -31 30 -28 27 -33 -22 -14 34 -17 8 -7 16 6 -4 -34 -35 24 -23 35 18 -19 -9 32 -25 26 '
     '-29'),
    ('ochiai-3', 'Ochiai III', cite('OCHIAI'),
     '1 3 31 30 -45 -44 65 64 62 63 -56 -57 54 55 -3 -4 7 8 -10 -9 24 23 49 48 -67 -65 -42 -43 33 32 -38 -39 40 42 44 '
     '46 -48 -50 17 18 -13 -15 -52 -54 57 58 -61 -62 39 37 -34 -33 -30 -29 -27 26 -25 -24 -22 -19 -18 13 14 11 9 -7 -5 '
     '2 4 28 29 -47 -46 67 66 61 60 -59 -58 52 53 -1 -2 5 6 -12 -11 22 21 51 50 -66 -64 -40 -41 34 35 -36 -37 41 43 45 '
     '47 -49 -51 20 19 -14 -16 -53 -55 56 59 -60 -63 38 36 -35 -32 -31 -28 27 -26 25 -23 -21 -20 -17 15 16 12 10 -8 -6'),
    ('ochiai-4', 'Ochiai IV (Suzuki)', cite('OCHIAI'),
     '1 -2 3 6 7 -8 9 -10 -11 12 8 -9 10 13 -14 -15 16 -17 -19 20 15 -16 17 -21 22 23 -24 14 25 26 27 -28 -29 -30 -13 '
     '-31 32 -33 -23 24 31 -32 33 34 -35 19 -20 -25 30 11 -12 36 37 4 -5 38 39 -40 41 29 -26 -42 43 -44 -45 46 42 -43 '
     '44 -47 -38 -48 49 -50 -51 52 48 -49 50 -37 -6 35 21 -22 -34 -7 -36 51 -52 -39 -53 45 -46 -27 28 54 -55 18 40 -41 '
     '-54 55 -18 53 47 -1 2 -3 -4 5'),
    # PZ78: the PDF text runs the page number into this code ('4714-63'); it reads '47 -63'
    ('pz78', 'Petronio–Zanellati 78', cite('BURTON'),
     '1 2 -3 -4 5 6 -7 -8 9 10 -2 -11 12 13 -14 -5 15 16 17 18 -19 -20 21 22 -6 -23 24 25 -26 -1 27 28 -29 -30 23 14 '
     '-31 -32 11 26 -33 -34 30 7 -22 -15 4 31 -13 -24 34 36 -28 -9 20 -17 37 38 35 39 -40 41 42 -43 -44 45 46 -47 -48 '
     '49 -39 -50 51 -52 -53 54 43 -55 -56 48 57 -58 -38 -61 -59 60 52 -62 -41 56 63 -46 -64 65 58 -66 -49 40 67 -51 '
     '-68 59 69 -70 -45 71 55 -42 -72 53 74 -69 -73 -37 -65 75 47 -63 -71 44 76 -74 -60 68 -77 -35 66 -57 -75 64 70 '
     '-76 -54 72 62 -67 50 77 61 73 -18 19 -10 -27 78 33 -25 -12 32 3 -16 -21 8 29 -36 -78'),
    ('pz120', 'Petronio–Zanellati 120', cite('PZ'),
     '1 2 -120 -117 -116 -115 114 113 -112 -111 -110 32 -31 -30 29 40 -39 -38 37 78 77 -92 91 90 -89 88 87 68 -66 64 '
     '53 -51 48 45 -43 -41 38 34 -33 -37 41 42 -40 -36 35 39 -42 -44 46 49 -52 54 65 -67 69 116 108 99 -101 103 111 50 '
     '-49 -48 51 52 -50 -47 44 43 -45 -46 47 110 102 -100 98 12 11 80 -85 -91 92 86 -80 -79 83 89 -90 -84 79 9 10 -93 '
     '95 -98 100 101 -99 96 -94 97 -96 -95 93 94 -97 109 117 -74 71 -69 67 66 -68 70 -73 72 -70 -71 74 73 -72 81 82 '
     '-83 84 85 -86 75 76 33 -34 -35 36 25 -26 -27 28 -102 -103 -104 105 106 -107 -108 -109 6 -8 -10 -12 -14 17 20 -22 '
     '-24 27 31 -32 -28 24 23 -25 -29 30 26 -23 -21 19 16 -13 -11 -9 -7 5 -82 -88 57 -55 -77 -75 15 -16 -17 14 13 -15 '
     '-18 21 22 -20 -19 18 -76 -78 -56 58 -53 -54 118 -113 -105 104 112 -118 -119 115 107 -106 -114 119 -65 -64 -63 60 '
     '-58 56 55 -57 59 -62 61 -59 -60 63 62 -61 -87 -81 -1 3 -5 7 8 -6 4 -2 120 -4 -3'),
    ('pz138', 'Petronio–Zanellati 138', cite('PZ'),
     '1 -11 -12 -13 -14 34 -46 -58 70 -86 -98 -110 -122 126 -130 -134 138 -111 -99 -87 -75 27 -28 -29 30 -31 -32 -33 '
     '-34 35 -36 -37 38 -138 -137 -136 -135 115 -103 -91 79 -63 -51 -39 -27 23 -19 -15 11 -38 -50 -62 -74 122 -121 '
     '-120 119 -118 -117 -116 -115 114 -113 -112 111 -1 2 -3 8 99 -100 -101 102 103 104 105 106 107 -108 -109 110 73 '
     '61 49 37 12 -16 -20 24 28 40 52 64 80 -92 -104 116 131 132 133 134 50 -49 -48 47 46 45 44 43 42 -41 -40 39 76 88 '
     '100 112 137 -133 -129 125 121 109 97 85 69 -57 -45 33 18 17 16 15 6 -4 -2 3 5 7 19 20 21 22 32 -44 -56 68 84 96 '
     '108 120 124 -128 -132 136 113 101 89 77 51 -52 -53 54 55 56 57 58 59 -60 -61 62 130 129 128 127 117 -105 -93 81 '
     '65 53 41 29 25 -21 -17 13 36 48 60 72 98 -97 -96 95 94 93 92 91 90 -89 -88 87 9 -5 4 -6 -7 -10 75 -76 -77 78 -79 '
     '-80 -81 -82 83 -84 -85 86 -71 -59 -47 -35 14 -18 -22 26 -30 -42 -54 -66 82 -94 -106 118 -123 -124 -125 -126 74 '
     '-73 -72 71 -70 -69 -68 -67 66 -65 -64 63 -78 -90 -102 -114 135 -131 -127 123 -119 -107 -95 -83 67 -55 -43 31 -26 '
     '-25 -24 -23 10 -9 -8'),
]
DEEPMIND42 = ('[[62, 25, 63, 26], [59, 4, 60, 5], [66, 17, 67, 18], [63, 11, 64, 10], [73, 19, 74, 18], [83, 33, 84, 32], '
              '[78, 41, 79, 42], [38, 31, 39, 32], [52, 46, 53, 45], [75, 46, 76, 47], [44, 52, 45, 51], [65, 55, 66, 54], '
              '[79, 48, 80, 49], [74, 53, 75, 54], [40, 57, 41, 58], [70, 61, 71, 62], [60, 69, 61, 70], [49, 76, 50, 77], '
              '[68, 71, 69, 72], [47, 80, 48, 81], [81, 57, 82, 56], [42, 77, 43, 78], [50, 44, 51, 43], [11, 16, 12, 17], '
              '[5, 26, 6, 27], [7, 28, 8, 29], [34, 29, 35, 30], [8, 36, 9, 35], [30, 37, 31, 38], [27, 6, 28, 7], '
              '[36, 10, 37, 9], [15, 12, 16, 13], [23, 15, 24, 14], [3, 21, 4, 20], [67, 23, 68, 22], [72, 21, 73, 22], '
              '[13, 25, 14, 24], [55, 65, 56, 64], [33, 1, 34, 84], [58, 2, 59, 1], [39, 82, 40, 83], [19, 3, 20, 2]]')


def gauss_to_link(code):
    g = [int(x) for x in code.split()]
    pos = {}
    for i, x in enumerate(g, 1):
        pos.setdefault(abs(x), []).append((i, x))
    pairs = {}
    for (i, a), (j, b) in pos.values():
        assert (i + j) % 2 == 1, 'not a planar knot Gauss code (parity)'
        odd, even = (i, j) if i % 2 else (j, i)
        sign_even = a if even == i else b
        pairs[odd] = even if sign_even < 0 else -even
    return spherogram.Link('DT: [(%s)]' % ','.join(str(pairs[o]) for o in range(1, 2 * len(pos), 2)))


def certify(L):
    G = L.exterior().fundamental_group()
    assert G.num_generators() == 1 and not G.relators(), 'not an unknot'


def layout(L):
    """Orthogonal planar drawing (SnapPy's plink layout): per component the grid vertices in order (y up), and per
    crossing the global indices of its under and over segments. Far more even than a Tutte embedding."""
    verts, arrows, crossings = OrthogonalLinkDiagram(L).plink_data()
    start = {a: k for k, (a, _) in enumerate(arrows)}  # arrow leaving each vertex
    seen, comps, seg_of = set(), [], {}
    for k0 in range(len(arrows)):
        if k0 in seen:
            continue
        comp, k = [], k0
        while k not in seen:
            seen.add(k)
            seg_of[k] = (len(comps), len(comp))
            a, b = arrows[k]
            comp.append(verts[a])
            k = start[b]
        comps.append(comp)
    offs = [0]
    for c in comps:
        offs.append(offs[-1] + len(c))
    g = lambda k: offs[seg_of[k][0]] + seg_of[k][1]
    xs = []
    for under, over, *_ in crossings:
        xs += [g(under), g(over)]
    # grid step 10 -> integers; flip y (plink is y-down)
    v = [[c for (x, y) in comp for c in (x // 10, -y // 10)] for comp in comps]
    assert len(crossings) == len(L.crossings)
    return {'v': v, 'x': xs}


def main():
    named = []
    for key, name, who, code in NAMED:
        L = gauss_to_link(code)
        certify(L)
        n = len(code.split()) // 2
        assert len(L.crossings) == n, (key, len(L.crossings), n)
        named.append({'id': key, 'name': name, 'crossings': n, 'cite': who, 'layout': layout(L)})
        print(f'{name:28s} {n:4d} crossings  ok', file=sys.stderr)
    L = spherogram.Link(json.loads(DEEPMIND42))
    certify(L)
    named.append({'id': 'deepmind-42', 'name': 'DeepMind 42', 'crossings': 42,
                  'cite': cite('DEEPMIND'), 'layout': layout(L)})
    url = 'https://storage.googleapis.com/gdm-unknotting/very_hard_unknots.csv'
    rows = list(csv.DictReader(urllib.request.urlopen(url).read().decode().splitlines()))
    very = []
    for r in rows:
        pd = json.loads(r['pd_code'])
        L = spherogram.Link(pd)
        certify(L)
        very.append({'n': len(pd), 'layout': layout(L)})
    very.sort(key=lambda r: r['n'])  # stable: CSV order within a crossing number
    print(f'{len(very)} DeepMind very hard unknots ok', file=sys.stderr)
    json.dump({'named': named, 'veryHard': {'cite': cite('DEEPMIND'), 'items': very}},
              open(OUT, 'w'), separators=(',', ':'))


if __name__ == '__main__':
    main()
