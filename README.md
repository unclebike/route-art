# Route Art

Turn a DXF or SVG drawing into Ride with GPS routes (KML or GPX). Static site on Cloudflare Pages: everything runs in the browser, nothing is uploaded.

## How it works

1. **Read** the drawing.
   - DXF: LINE, LWPOLYLINE/POLYLINE (with arc bulges), ARC, CIRCLE, ELLIPSE, SPLINE (NURBS, rational), nested block INSERTs. Text, hatches and dimensions are ignored and listed as warnings.
   - SVG: path, line, polyline, polygon, rect, circle, ellipse, with every transform applied by the browser's own geometry engine. Clip paths, masks, `<use>` and text are not applied.
2. **Join** every piece into one sequence that keeps the straight connectors between pieces short. The order is greedy nearest-neighbour, then 2-opt; it runs in a Web Worker.
3. **Place** the drawing's bounding box (after optional rotation) between two lat/lon corners, mapped linearly in Web Mercator. That way it looks undistorted on the RWGPS map. Drag the handles or paste coordinates.
4. **Smooth** with Douglas–Peucker in metres (auto: width ÷ 20,000, clamped to 1–100 m).
5. **Split** into the fewest routes that fit the max points / max km limits. A dynamic programme places the cuts where they remove the longest straight connectors.
6. **Download** one KML/GPX per route, or all of them as a zip, then upload at ridewithgps.com/upload and choose *Save as a Route*.

## Layout

```
index.html, styles.css, favicon.svg, _headers
js/dxf.js      DXF parser and curve sampling
js/svg.js      SVG reader (DOM geometry)
js/core.js     ordering, placement, simplification, splitting, KML/GPX
js/worker.js   runs ordering off the main thread
js/main.js     UI and MapLibre map (OpenFreeMap tiles)
vendor/        maplibre-gl 4.7.1, jszip 3.10.1 (pinned, self-hosted)
tests/         fixtures + Playwright harness (see tests/README.md)
```

`window.RWGPSConv` exposes `parseDXF`, `parseSVG`, `order` and `exportRoutes` for the test harness.

## Deploy

Every push to `main` deploys through `.github/workflows/deploy.yml` (wrangler `pages deploy`, project `route-art`). It needs the repo secrets `CLOUDFLARE_API_TOKEN` and `CLOUDFLARE_ACCOUNT_ID`.

Run locally: `python3 -m http.server -d .` and open http://localhost:8000.
