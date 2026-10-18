# Route Art

Turn a DXF or SVG drawing into Ride with GPS routes (KML or GPX). Static site on Cloudflare Pages: everything runs in the browser, nothing is uploaded.

## How it works

1. **Read** the drawing.
   - DXF: LINE, LWPOLYLINE/POLYLINE (with arc bulges), ARC, CIRCLE, ELLIPSE, SPLINE (NURBS, rational), nested block INSERTs. Text, hatches and dimensions are ignored and listed as warnings.
   - SVG: path (all commands, including arcs), line, polyline, polygon, rect (rounded), circle, ellipse. Geometry is read in JS; the browser only supplies transforms and computed styles.
   - **Clip paths** cut the lines exactly to the clip shapes (clip-rule respected, objectBoundingBox units supported). **Masks** are applied as solid shapes: light fills show, dark fills hide, in paint order. Gradients and partial transparency are treated as on/off, and image masks are ignored. Clipping uses Clipper (vendor/clipper.js).
   - Text and `<use>` clones are skipped with a warning; convert text to outlines first.
2. **Join** every piece into one sequence that keeps the straight connectors between pieces short. The order is greedy nearest-neighbour, then 2-opt; it runs in a Web Worker.
   - **Draw** mode: *Outline* rides the drawing's own lines. *Squiggle fill* fills every closed shape with one unbroken squiggle. *Fill + outline* does both. Closed outlines are combined into shapes first: DXF loops even-odd, so letter counters become holes, and SVG uses each element's fill-rule.
   - **Squiggle fill**: the tool grows a random spanning tree on a 2×spacing grid inside the shape (keeping ≥ 0.45×spacing clearance from the outline, checked exactly along every segment). The walk around that tree visits every grid node once and closes on itself, so each shape gets a single loop with no crossings and no connectors. It tries grid angles every 7.5° (two offsets each) and keeps the one needing the fewest lines. Lines split by narrow necks are bridged with two short rungs. Corners are rounded (Chaikin), with an optional wobble. Density, wobble and the random pattern (Shuffle) are adjustable.
   - **Shapes**: either join everything into one route, or make one route per shape. In both cases, closed loops are spliced into the neighbouring line with a short out-and-back spur, not chained with long connectors.
3. **Place** the drawing's bounding box (after optional rotation) between two lat/lon corners, mapped linearly in Web Mercator. That way it looks undistorted on the RWGPS map. Drag the handles or paste coordinates.
4. **Smooth** with Douglas–Peucker in metres (auto: width ÷ 20,000, clamped to 1–100 m).
5. **Split** into the fewest routes that fit the max points / max km limits. A dynamic programme places the cuts where they remove the longest straight connectors.
6. **Download** one KML/GPX per route, or all of them as a zip, then upload at ridewithgps.com/upload and choose *Save as a Route*.

## Layout

```
index.html, styles.css, favicon.svg, _headers
js/dxf.js      DXF parser and curve sampling
js/svg.js      SVG reader (DOM geometry)
js/core.js     ordering (greedy + 2-opt + loop splicing), placement, simplification, splitting, KML/GPX
js/shapes.js   closed outlines → shapes (outer + holes) with Clipper
js/fill.js     squiggle fill (spanning-tree Hamiltonian cycle)
js/plan.js     builds the route groups for the chosen draw mode
js/worker.js   runs ordering off the main thread
js/main.js     UI and MapLibre map (OpenFreeMap tiles)
vendor/        maplibre-gl 4.7.1, jszip 3.10.1, clipper-lib 6.4.2 (pinned, self-hosted)
tests/         fixtures + Playwright harness: python3 -I tests/run_tests.py (see tests/README.md)
```

`window.RWGPSConv` exposes `parseDXF`, `parseSVG`, `order`, `exportRoutes`, `exportGroups`, `buildShapes`, `squiggleFill` and `prepare` for the test harness.

## Deploy

Every push to `main` deploys through `.github/workflows/deploy.yml` (wrangler `pages deploy`, project `route-art`). It needs the repo secrets `CLOUDFLARE_API_TOKEN` and `CLOUDFLARE_ACCOUNT_ID`.

Run locally: `python3 -m http.server -d .` and open http://localhost:8000.
