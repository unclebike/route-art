# route-art tests

Browser tests for `window.RWGPSConv` (parseDXF / parseSVG / order / exportRoutes), run in headless
Chromium through Playwright against fixtures whose reference geometry comes from independent
Python code (ezdxf for DXF, svgelements for SVG).

## Run

```zsh
cd /home/claude/route-art
python3 -I tests/run_tests.py                         # everything (~2-3 min, mostly squiggle)
python3 -I tests/run_tests.py --only arcs,paths.svg   # substring filter on fixture names
python3 -I tests/run_tests.py --no-e2e                # skip exportRoutes / exportGroups checks
python3 -I tests/run_tests.py --fill                  # only the squiggle-fill section (~40 s)
python3 -I tests/run_tests.py --no-fill               # skip it
python3 -I tests/run_tests.py --page tests/_mock/index.html   # self-test the harness against the mock
```

The script serves the site root with `python3 -m http.server` on a free port, opens `index.html`,
waits for `RWGPSConv.ready === true`, and runs every fixture. It exits 1 if any check FAILs. WARN
never fails the run. It also writes `tests/last_results.json`. If the pinned Playwright browser is
missing it falls back to `/opt/pw-browsers/chromium-*/chrome-linux/chrome`.

Requirements: `ezdxf numpy shapely playwright` (+ `svgelements` to regenerate fixtures).

## Regenerate fixtures

```zsh
python3 -I tests/make_fixtures.py
```

This writes `tests/fixtures/<file>` and `<file>.ref.json`. The extension is kept in the ref name so
`squiggle.dxf` and `squiggle.svg` don't collide. Each ref holds dense reference polylines (y-up),
total length, bbox, the tolerance and any extra expectations.

| fixture | covers |
|---|---|
| lines.dxf | 9 LINEs: connected zig-zag + square; layers ZIGZAG/SQUARE |
| arcs.dxf | ARC 300→60 and 170→10 (wrap through 0°), CIRCLE, rotated partial ELLIPSE, ELLIPSE wrapping 2π, full ELLIPSE |
| lwpoly_bulge.dxf | LWPOLYLINE ± bulges, closed LWPOLYLINE with closing-segment bulge, closed 2D POLYLINE with VERTEX bulges |
| splines.dxf | clamped cubic, clamped non-uniform knots, closed periodic (unclamped knots), rational quadratic quarter circle, rational cubic, quadratic |
| splines_fit.dxf | fit-points-only SPLINE. **Advisory**: interpolation isn't uniquely defined, so this is WARN-only with tol 1e-2 |
| blocks.dxf | block base point ≠ origin, mirror (xscale −1), non-uniform scale + rotation, nested block |
| ocs.dxf | ARC / CIRCLE / LWPOLYLINE with extrusion (0,0,−1) |
| units_mm.dxf | `$INSUNITS=4` → `units: 'mm'` |
| r12.dxf | AutoCAD R12 structure (POLYLINE/VERTEX, rotated INSERT) |
| squiggle.dxf | the real file (CRLF line endings, 3-level nested blocks, 649 SPLINEs, 88 single-point LWPOLYLINEs) |
| shapes.svg | rect (with and without rx/ry), circle, ellipse, line, polyline, polygon |
| transforms.svg | nested translate/rotate/non-uniform scale/matrix, `rotate(a cx cy) skewX`, viewBox with negative min |
| paths.svg | one path, 3 subpaths, relative m c s q t a h v H z, `m` after `z`, arc flag combinations, implicit lineto |
| paths_compact.svg | svgo-style compact numbers and packed arc flags |
| ignored.svg | defs/clipPath/mask/symbol/marker/pattern ignored, `<text>` warning, exactly 1 path |
| squiggle.svg | squiggle as straight-segment paths inside `translate(100,50) scale(0.5)` |
| bike_outline.dxf | the 4 closed SPLINEs at the end of Block_0 of bike_squiggle_fill.dxf: "bike" word outline, i-dot, b and e counters. Even-odd: 2 shapes (word with 2 holes, i-dot) |
| bike_outline.svg | same rings as ONE compound `<path fill-rule="evenodd">` in `translate(-470,-105) scale(1.2)` |
| bike_outline_nonzero.svg | same rings as 4 separate `<path>`s (nonzero). Per-element fill then union, so the counters are filled in (2 shapes, 0 holes). Recorded only (INFO) |
| fill_shapes.svg | square, annulus (evenodd), dumbbell with a 1.5 x spacing neck, bar narrower than spacing (no fill). Spacing h/40 = 5 |

Fill fixtures carry a `fill` block in their ref (shapely reference: shapes with rings, areas, holes, expected
loop counts, `height`, `spacing_divs`). `bike_outline.dxf` is committed. To re-extract it from the 22 MB original
(not committed), run `BIKE_DXF=/path/to/bike_squiggle_fill.dxf python3 -I tests/make_fixtures.py --fill-only`.
`--fill-only` regenerates only the fill fixtures.

## Checks

- **parse**: symmetric max distance between the output and the reference, with both sides densified at
  tol/4 and measured point-to-segment. The limit is `tol_rel × bbox diagonal` (1e-3). Total length must be
  within 0.5%. Closed paths get their closing segment added. Also checks units, path count,
  warnings and that every coordinate is finite. Single-point entities are optional in the output.
- **order**: curves ≤ input paths, the curves cover the parsed geometry (max distance both ways, length
  within 0.5%), `jumps[i]` equals the gap between curve i's end and curve i+1's start, and the jump sum
  is < 4000 for squiggle.dxf (< 2000 for squiggle.svg, which is drawn at half scale).
- **e2e / maxKm / e2e-bv / rot90**: exportRoutes. For squiggle: totalKm within 2% of 127,500, bbox ==
  corners within 1e-4°, ≤ 6000 points per route, 20–25 routes (WARN), KML/GPX parse with lon,lat in the
  box, route.km matches the haversine length of the KML, and every route ≤ maxKm with maxKm=3000.
  **geo** compares the routes in metres against the spec mapping (rotate about the bbox centre, then
  map linearly in Web Mercator) applied to the parsed drawing. This catches mirroring, lat/lon swaps
  and projection errors.
- **order** accepts loop-splice spurs. A connector inside a curve is OK only if it is retraced (segment A->B and
  later B->A in the same curve). Spurs are left out of the geometry and length comparisons and reported as INFO. Any
  other connector inside a curve FAILs as invented geometry.
- **shapes** (buildShapes): shape count, area (≤ 1%), holes and sym-diff vs shapely (polygonize + even-odd, or
  union for the nonzero variant).
- **fill/N, fillW/N** (squiggleFill per shape, spacing = height/N, seed 1; fillW = wobble 0.22): loops closed,
  vertices inside, vertex clearance ≥ 0.3 x spacing, segments inside (no boundary crossing), segment clearance
  (WARN), simple, pairwise disjoint, coverage of the shape eroded by 1 x spacing (p95 ≤ 0.75, max ≤ 2.0 x spacing),
  loop counts, determinism and timing (bike h/120 < 2 s, WARN).
- **prep** (prepare, h/40): group counts for outline/fill/both x shape/join, ring-only geometry for outline, every
  ring vertex on an output curve, total length ≥ perimeters + squiggleFill lengths, fill clearance, each group
  inside one shape, join vs per-shape geometry (spurs excluded, 0.5%).
- **fill-e2e** (exportGroups on bike_outline.*): KML/GPX, totals, maxPoints, unique names, every group exported,
  geo placement, and no route mixing shapes (KML vertices tested against the mapped shapes).
- **ui**: loads bike_outline.svg via `#file`, picks Squiggle fill (`input[name=mode]`) and `#grouping` = shape,
  expects ≥ 2 routes in `#routes` and no page errors, then clicks `#shuffle`.
- A phase slower than 5 s gives a WARN. Uncaught page errors FAIL. Console errors are printed.

The squiggle stress fixtures (a 4.6 MB real-world DXF) aren't committed. To include them, run `SQUIGGLE_DXF=/path/to/squiggle.dxf python3 -I tests/make_fixtures.py`.
