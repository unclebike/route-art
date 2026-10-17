#!/usr/bin/env python3
"""Browser test harness for the route-art DXF/SVG -> RWGPS converter.

Usage (from anywhere):
    python3 -I /home/claude/route-art/tests/run_tests.py [options]

Options:
    --root DIR        site root to serve (default: parent of this tests/ dir)
    --page PATH       page to open, relative to root (default: index.html;
                      tests/_mock/index.html runs the harness against the mock)
    --only a,b        only fixtures whose file name contains one of these substrings
    --no-e2e          skip exportRoutes end-to-end checks
    --port N          http port (default: random free port)
    --headed          show the browser
    --json FILE       also write all check results as JSON (default tests/last_results.json)

Exit code: 0 if no FAIL, 1 otherwise (WARN does not fail the run).
"""
import argparse
import glob
import json
import math
import os
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import shapely
from shapely import STRtree

HERE = os.path.dirname(os.path.abspath(__file__))
FIX = os.path.join(HERE, "fixtures")
CHROME_FALLBACKS = sorted(glob.glob("/opt/pw-browsers/chromium-*/chrome-linux/chrome"), reverse=True)

# ---------------------------------------------------------------- end-to-end constants
SQ_TL = [48.18022585379953, -101.29474523981379]
SQ_BR = [40.131393368979495, -88.23403637956659]
SQ_TOTAL_KM = 127500.0
BV_TL = [44.20, -77.45]
BV_BR = [44.12, -77.30]
SLOW_MS = 5000.0

R_EARTH_M = 6378137.0          # Web Mercator sphere
R_MEAN_KM = 6371.0088          # haversine


# ======================================================================= result bookkeeping
class Results:
    def __init__(self):
        self.rows = []

    def add(self, fixture, phase, check, status, value=None, threshold=None, detail=""):
        self.rows.append(dict(fixture=fixture, phase=phase, check=check, status=status,
                              value=value, threshold=threshold, detail=detail))
        mark = {"PASS": "  ok ", "FAIL": "FAIL ", "WARN": "warn ", "INFO": "     "}[status]
        v = "" if value is None else f" = {fmt(value)}"
        t = "" if threshold is None else f"  (limit {fmt(threshold)})"
        d = f"  -- {detail}" if detail else ""
        print(f"    {mark}{phase:6s} {check}{v}{t}{d}", flush=True)

    def check(self, fixture, phase, check, ok, value=None, threshold=None, detail="", soft=False):
        self.add(fixture, phase, check, "PASS" if ok else ("WARN" if soft else "FAIL"),
                 value, threshold, detail)
        return ok


def fmt(v):
    if isinstance(v, float):
        if v == 0:
            return "0"
        if abs(v) >= 1e5 or abs(v) < 1e-3:
            return f"{v:.3e}"
        return f"{v:.4f}".rstrip("0").rstrip(".")
    return str(v)


# ======================================================================= geometry helpers
def as_np(pl):
    a = np.asarray(pl, dtype=float)
    return a.reshape(-1, 2) if a.size else np.zeros((0, 2))


def close_if_needed(pts, closed):
    if closed and len(pts) >= 2 and not np.allclose(pts[0], pts[-1], rtol=0, atol=1e-12):
        return np.vstack([pts, pts[:1]])
    return pts


def poly_length(a):
    return float(np.sum(np.hypot(*np.diff(a, axis=0).T))) if len(a) >= 2 else 0.0


def densify(polys, spacing):
    """Sample every polyline at <= spacing (all original vertices kept)."""
    out = []
    for a in polys:
        if len(a) == 1:
            out.append(a)
            continue
        d = np.diff(a, axis=0)
        L = np.hypot(d[:, 0], d[:, 1])
        k = np.maximum(1, np.ceil(L / spacing).astype(int))
        idx = np.repeat(np.arange(len(d)), k)
        frac = (np.arange(k.sum()) - np.repeat(np.cumsum(k) - k, k)) / np.repeat(k, k)
        pts = a[idx] + d[idx] * frac[:, None]
        out.append(np.vstack([pts, a[-1:]]))
    return np.vstack(out) if out else np.zeros((0, 2))


def segments_tree(polys, extra_points=None):
    segs = []
    for a in polys:
        if len(a) >= 2:
            segs.append(np.stack([a[:-1], a[1:]], axis=1))
    geoms = []
    if segs:
        S = np.concatenate(segs)
        keep = np.any(S[:, 0] != S[:, 1], axis=1)
        S = S[keep] if keep.any() else S
        geoms.append(shapely.linestrings(S))
    if extra_points is not None and len(extra_points):
        geoms.append(shapely.points(np.asarray(extra_points, dtype=float)))
    if not geoms:
        return None, None
    g = np.concatenate(geoms)
    return STRtree(g), g


def nearest_dist(tree, pts, scale=None):
    """max/where of distance from pts to tree geometries. scale: per-point multiplier."""
    if tree is None or len(pts) == 0:
        return float("inf"), None
    P = shapely.points(pts)
    _, d = tree.query_nearest(P, return_distance=True, all_matches=False)
    if scale is not None:
        d = d * scale
    i = int(np.argmax(d))
    return float(d[i]), pts[i].tolist()


def hausdorff(ref_polys, ref_points, out_polys, tol):
    """Symmetric max distance using dense sampling of both sides (spacing tol/4)."""
    total = sum(poly_length(a) for a in ref_polys) + sum(poly_length(a) for a in out_polys)
    spacing = max(tol / 4.0, total / 1.5e6)
    out_pts_only = [a for a in out_polys if len(a) == 1]
    out_lines = [a for a in out_polys if len(a) >= 2]
    t_ref, _ = segments_tree(ref_polys, ref_points)
    t_out, _ = segments_tree(out_lines, np.vstack(out_pts_only) if out_pts_only else None)
    rs = densify(ref_polys, spacing)
    os_ = densify(out_polys, spacing)
    d_ro, at_ro = nearest_dist(t_out, rs)
    d_or, at_or = nearest_dist(t_ref, os_)
    return d_ro, at_ro, d_or, at_or


# ======================================================================= geo helpers
def merc(lat, lon):
    lat = np.radians(np.clip(lat, -85.05112878, 85.05112878))
    return np.stack([R_EARTH_M * np.radians(lon), R_EARTH_M * np.log(np.tan(np.pi / 4 + lat / 2))], -1)


def haversine_km(latlon):
    a = np.radians(np.asarray(latlon, float))
    if len(a) < 2:
        return 0.0
    dlat = np.diff(a[:, 0])
    dlon = np.diff(a[:, 1])
    h = np.sin(dlat / 2) ** 2 + np.cos(a[:-1, 0]) * np.cos(a[1:, 0]) * np.sin(dlon / 2) ** 2
    return float(np.sum(2 * R_MEAN_KM * np.arcsin(np.sqrt(np.minimum(1, h)))))


def map_drawing(polys, tl, br, rot_deg):
    """Spec transform: rotate about bbox centre, then map rotated bbox linearly in Web Mercator
    so top-left -> tl and bottom-right -> br. Returns list of (N,2) [lat,lon] arrays."""
    allp = np.vstack(polys)
    mn, mx = allp.min(0), allp.max(0)
    c = (mn + mx) / 2
    th = math.radians(rot_deg)
    R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    rp = [(a - c) @ R.T + c for a in polys]
    allr = np.vstack(rp)
    mn, mx = allr.min(0), allr.max(0)
    TL = merc(np.array(tl[0]), np.array(tl[1]))
    BR = merc(np.array(br[0]), np.array(br[1]))
    out = []
    for a in rp:
        X = TL[0] + (a[:, 0] - mn[0]) / (mx[0] - mn[0]) * (BR[0] - TL[0])
        Y = BR[1] + (a[:, 1] - mn[1]) / (mx[1] - mn[1]) * (TL[1] - BR[1])
        lon = np.degrees(X / R_EARTH_M)
        lat = np.degrees(2 * np.arctan(np.exp(Y / R_EARTH_M)) - np.pi / 2)
        out.append(np.stack([lat, lon], -1))
    return out


def geo_hausdorff(exp_latlon_polys, got_latlon_polys):
    """Symmetric max distance in metres (computed in Mercator, scaled by cos(lat) per point)."""
    em = [merc(a[:, 0], a[:, 1]) for a in exp_latlon_polys]
    gm = [merc(a[:, 0], a[:, 1]) for a in got_latlon_polys if len(a)]
    lat0 = np.vstack(exp_latlon_polys)[:, 0]
    k = float(np.cos(np.radians(lat0.max())))  # conservative-ish local scale
    span = np.ptp(np.vstack(em), axis=0)
    spacing = max(math.hypot(*span) * 2e-4, 1.0)
    t_g, _ = segments_tree(gm)
    t_e, _ = segments_tree(em)
    es = densify(em, spacing)
    gs = np.vstack(gm)   # vertices only (connectors are straight joins, not geometry)

    def scale_of(P):
        lat = np.degrees(2 * np.arctan(np.exp(P[:, 1] / R_EARTH_M)) - np.pi / 2)
        return np.cos(np.radians(lat))
    d_eg, _ = nearest_dist(t_g, es, scale_of(es))
    d_ge, _ = nearest_dist(t_e, gs, scale_of(gs))
    diag_m = math.hypot(*span) * k
    return d_eg, d_ge, diag_m


# ======================================================================= KML / GPX parsing
def strip_ns(tag):
    return tag.rsplit("}", 1)[-1]


def parse_kml(text):
    root = ET.fromstring(text.encode("utf-8") if isinstance(text, str) else text)
    lines = []
    for el in root.iter():
        if strip_ns(el.tag) == "LineString":
            for c in el:
                if strip_ns(c.tag) == "coordinates":
                    toks = (c.text or "").split()
                    pts = []
                    for t in toks:
                        f = t.split(",")
                        pts.append((float(f[1]), float(f[0])))   # (lat, lon) from lon,lat[,alt]
                    lines.append(np.array(pts).reshape(-1, 2))
    return lines


def parse_gpx(text):
    root = ET.fromstring(text.encode("utf-8") if isinstance(text, str) else text)
    pts = []
    kinds = set()
    for el in root.iter():
        t = strip_ns(el.tag)
        if t in ("trkpt", "rtept"):
            kinds.add(t)
            pts.append((float(el.attrib["lat"]), float(el.attrib["lon"])))
    return np.array(pts).reshape(-1, 2), kinds


# ======================================================================= browser plumbing
JS_PARSE = """
async ({kind, text}) => {
  const C = window.RWGPSConv;
  const t0 = performance.now();
  let r = kind === 'dxf' ? C.parseDXF(text) : C.parseSVG(text);
  const isPromise = !!(r && typeof r.then === 'function');
  if (isPromise) r = await r;
  const ms = performance.now() - t0;
  window.__parsed = r;
  const paths = (r.paths || []).map(p => ({
      layer: p.layer, closed: !!p.closed,
      pts: Array.from(p.pts || [], q => [Number(q[0]), Number(q[1])])}));
  return {ms, isPromise, paths, layers: r.layers ?? null, units: r.units === undefined ? '__undefined__' : r.units,
          warnings: (r.warnings || []).map(String)};
}
"""

JS_ORDER = """
async () => {
  const C = window.RWGPSConv;
  const t0 = performance.now();
  let o = C.order(window.__parsed.paths);
  const isPromise = !!(o && typeof o.then === 'function');
  if (isPromise) o = await o;
  const ms = performance.now() - t0;
  window.__ordered = o;
  return {ms, isPromise,
          curves: (o.curves || []).map(c => Array.from(c, q => [Number(q[0]), Number(q[1])])),
          jumps: Array.from(o.jumps || [], Number)};
}
"""

JS_EXPORT = """
async (opts) => {
  const C = window.RWGPSConv;
  const t0 = performance.now();
  let e = C.exportRoutes(window.__ordered, opts);
  const isPromise = !!(e && typeof e.then === 'function');
  if (isPromise) e = await e;
  const ms = performance.now() - t0;
  return {ms, isPromise, totalKm: e.totalKm, bbox: e.bbox,
          routes: (e.routes || []).map(r => ({name: r.name, points: r.points, km: r.km,
                   maxConnectorKm: r.maxConnectorKm, kml: r.kml, gpx: r.gpx}))};
}
"""


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def start_server(root, port):
    proc = subprocess.Popen([sys.executable, "-I", "-m", "http.server", str(port), "--bind", "127.0.0.1",
                             "--directory", root], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(100):
        try:
            socket.create_connection(("127.0.0.1", port), timeout=0.2).close()
            return proc
        except OSError:
            time.sleep(0.1)
    proc.kill()
    raise RuntimeError("http.server did not start")


def launch(pw, headed):
    try:
        return pw.chromium.launch(headless=not headed)
    except Exception as e:  # pinned browser missing -> use whatever is installed
        for exe in CHROME_FALLBACKS:
            try:
                print(f"  (default chromium failed: {str(e).splitlines()[0]}; using {exe})")
                return pw.chromium.launch(headless=not headed, executable_path=exe)
            except Exception:
                continue
        raise


# ======================================================================= test phases
def load_refs(only):
    refs = []
    for f in sorted(glob.glob(os.path.join(FIX, "*.ref.json"))):
        r = json.load(open(f))
        if only and not any(o in r["fixture"] for o in only):
            continue
        refs.append(r)
    # small fixtures first, squiggles last
    refs.sort(key=lambda r: ("squiggle" in r["fixture"], r["kind"], r["fixture"]))
    return refs


def test_parse(page, ref, res):
    name, kind = ref["fixture"], ref["kind"]
    text = open(os.path.join(FIX, name), encoding="utf-8", errors="replace", newline="").read()  # keep CRLF as-is
    t0 = time.time()
    try:
        out = page.evaluate(JS_PARSE, {"kind": kind, "text": text})
    except Exception as e:
        res.add(name, "parse", "parse() threw", "FAIL", detail=str(e).splitlines()[0][:300])
        return None
    wall = (time.time() - t0) * 1000
    soft = ref.get("advisory", False)
    diag = ref["diag"]
    tol = ref["tol_rel"] * diag
    paths = out["paths"]
    res.add(name, "parse", "time_ms", "INFO", round(out["ms"], 1),
            detail=f"wall incl. transfer {wall:.0f} ms; {len(paths)} paths, "
                   f"{sum(len(p['pts']) for p in paths)} pts; units={out['units']!r}; "
                   f"warnings={out['warnings'][:3]}{'...' if len(out['warnings']) > 3 else ''}")
    if out["ms"] > SLOW_MS:
        res.add(name, "parse", "SLOW", "WARN", round(out["ms"]), SLOW_MS)
    if out["isPromise"]:
        res.add(name, "parse", "returned a Promise (spec: sync object)", "WARN")
    if out["units"] == "__undefined__":
        res.add(name, "parse", "result.units missing (undefined)", "FAIL")
    if not isinstance(out["layers"], list):
        res.add(name, "parse", "result.layers not an array", "FAIL", detail=repr(out["layers"])[:80])

    finite = all(math.isfinite(c) for p in paths for q in p["pts"] for c in q)
    res.check(name, "parse", "all coordinates finite", finite)
    if not finite or not paths:
        res.check(name, "parse", "non-empty output", bool(paths))
        return None

    out_polys = [close_if_needed(as_np(p["pts"]), p["closed"]) for p in paths]
    out_polys = [a for a in out_polys if len(a)]
    ref_polys = [as_np(pl) for pl in ref["polylines"]]
    ref_points = np.asarray(ref.get("points") or [], float).reshape(-1, 2)

    d_ro, at_ro, d_or, at_or = hausdorff(ref_polys, ref_points, out_polys, tol)
    res.check(name, "parse", "maxdist ref->out", d_ro <= tol, d_ro, tol,
              f"worst ref pt {np.round(at_ro, 4).tolist() if at_ro else None}; rel {d_ro / diag:.2e}"
              + ("" if d_ro <= tol else " (output is MISSING or displaced geometry here)"), soft=soft)
    res.check(name, "parse", "maxdist out->ref", d_or <= tol, d_or, tol,
              f"worst out pt {np.round(at_or, 4).tolist() if at_or else None}; rel {d_or / diag:.2e}"
              + ("" if d_or <= tol else " (output has EXTRA or displaced geometry here)"), soft=soft)
    L = sum(poly_length(a) for a in out_polys)
    rel = (L - ref["length"]) / ref["length"]
    res.check(name, "parse", "length rel err", abs(rel) <= ref["len_tol"], rel, ref["len_tol"],
              f"out {L:.4f} vs ref {ref['length']:.4f}", soft=soft)

    ex = ref.get("expect", {})
    if "units" in ex:
        if ex["units"] is None:
            res.check(name, "parse", "units", out["units"] is None, repr(out["units"]), "None",
                      "INSUNITS 0/absent should give null", soft=True)
        else:
            res.check(name, "parse", "units", out["units"] == ex["units"], repr(out["units"]), repr(ex["units"]))
    if "path_count" in ex:
        n = sum(1 for p in paths if len(p["pts"]) >= 2)
        res.check(name, "parse", "path count (>=2 pts)", n == ex["path_count"], n, ex["path_count"])
    if "warning_re" in ex:
        import re
        ok = any(re.search(ex["warning_re"], w) for w in out["warnings"])
        res.check(name, "parse", "warning mentions text", ok, out["warnings"][:3], ex["warning_re"])
    if "layers" in ex and isinstance(out["layers"], list):
        res.check(name, "parse", "layers", sorted(out["layers"]) == ex["layers"], sorted(out["layers"]),
                  ex["layers"], soft=True)
    n_degen = sum(1 for p in paths if len(p["pts"]) < 2)
    if n_degen:
        res.add(name, "parse", "single-point paths in output", "INFO", n_degen)
    return dict(out=out, out_polys=out_polys, diag=diag, tol=tol)


def test_order(page, ref, parsed, res):
    name = ref["fixture"]
    try:
        o = page.evaluate(JS_ORDER)
    except Exception as e:
        res.add(name, "order", "order() threw", "FAIL", detail=str(e).splitlines()[0][:300])
        return None
    res.add(name, "order", "time_ms", "INFO", round(o["ms"], 1),
            detail=f"{len(o['curves'])} curves, {len(o['jumps'])} jumps")
    if o["isPromise"]:
        res.add(name, "order", "returned a Promise (spec: synchronous)", "WARN")
    if o["ms"] > SLOW_MS:
        res.add(name, "order", "SLOW", "WARN", round(o["ms"]), SLOW_MS)
    n_in = len(parsed["out"]["paths"])
    curves = [as_np(c) for c in o["curves"]]
    res.check(name, "order", "curves <= input paths", len(curves) <= n_in, len(curves), n_in)
    res.check(name, "order", "jumps length == curves-1", len(o["jumps"]) == max(0, len(curves) - 1),
              len(o["jumps"]), max(0, len(curves) - 1), soft=True)
    finite = all(np.isfinite(c).all() for c in curves) and all(math.isfinite(j) for j in o["jumps"])
    res.check(name, "order", "all finite", finite)
    if not curves or not finite:
        return o
    tol = parsed["tol"]
    in_polys = [a for a in parsed["out_polys"] if len(a) >= 2]
    curves_l = [c for c in curves if len(c) >= 2]
    d_io, at_io, d_oi, at_oi = hausdorff(in_polys, None, curves_l, tol)
    res.check(name, "order", "maxdist parsed->curves", d_io <= tol, d_io, tol,
              f"worst {np.round(at_io, 4).tolist() if at_io else None}" + ("" if d_io <= tol else " (geometry LOST by order())"))
    res.check(name, "order", "maxdist curves->parsed", d_oi <= tol, d_oi, tol,
              f"worst {np.round(at_oi, 4).tolist() if at_oi else None}" + ("" if d_oi <= tol else " (order() INVENTED geometry - connectors inside curves?)"))
    Lin = sum(poly_length(a) for a in in_polys)
    Lout = sum(poly_length(a) for a in curves_l)
    rel = (Lout - Lin) / Lin if Lin else 0
    res.check(name, "order", "curve length vs parsed", abs(rel) <= 0.005, rel, 0.005,
              "loops may be rotated; length must be preserved")
    # jumps vs actual gaps
    if len(o["jumps"]) == len(curves) - 1 and len(curves) > 1:
        gaps = np.array([np.hypot(*(curves[i + 1][0] - curves[i][-1])) for i in range(len(curves) - 1)])
        dev = float(np.max(np.abs(gaps - np.array(o["jumps"]))))
        res.check(name, "order", "jumps == |end_i -> start_i+1|", dev <= 1e-6 * parsed["diag"] + 1e-9, dev,
                  1e-6 * parsed["diag"], soft=True)
    js = float(sum(o["jumps"]))
    lim = ref.get("expect", {}).get("jump_sum_max")
    if lim:
        res.check(name, "order", "sum(jumps)", js < lim, js, lim)
    else:
        res.add(name, "order", "sum(jumps)", "INFO", js)
    return o


def check_export(page, name, res, opts, *, parsed=None, expect_routes=None, expect_total_km=None,
                 km_tol=0.02, routes_range=None, tag="e2e"):
    try:
        e = page.evaluate(JS_EXPORT, opts)
    except Exception as ex:
        res.add(name, tag, "exportRoutes() threw", "FAIL", detail=str(ex).splitlines()[0][:300])
        return None
    res.add(name, tag, "time_ms", "INFO", round(e["ms"], 1),
            detail=f"opts tl={opts['tl']} br={opts['br']} rot={opts['rotationDeg']} tolM={opts['toleranceM']} "
                   f"maxPts={opts['maxPoints']} maxKm={opts['maxKm']}")
    if e["ms"] > SLOW_MS:
        res.add(name, tag, "SLOW", "WARN", round(e["ms"]), SLOW_MS)
    if e["isPromise"]:
        res.add(name, tag, "returned a Promise (spec: sync object)", "WARN")
    routes = e["routes"]
    tl, br = opts["tl"], opts["br"]
    lat_lo, lat_hi = min(tl[0], br[0]), max(tl[0], br[0])
    lon_lo, lon_hi = min(tl[1], br[1]), max(tl[1], br[1])
    eps = 1e-6

    res.add(name, tag, "routes", "INFO", len(routes),
            detail="km: " + ", ".join(f"{r['km']:.0f}" for r in routes[:30]) + (" ..." if len(routes) > 30 else ""))
    if expect_routes is not None:
        res.check(name, tag, "route count", len(routes) == expect_routes, len(routes), expect_routes)
    if routes_range:
        res.check(name, tag, "route count in expected range", routes_range[0] <= len(routes) <= routes_range[1],
                  len(routes), f"{routes_range[0]}..{routes_range[1]}", soft=True)
    if not routes:
        res.add(name, tag, "no routes", "FAIL")
        return e

    tk = e["totalKm"]
    if expect_total_km:
        rel = (tk - expect_total_km) / expect_total_km
        res.check(name, tag, "totalKm", abs(rel) <= km_tol, tk, f"{expect_total_km} +/-{km_tol:.0%}",
                  f"rel {rel:+.3%}")
    else:
        res.add(name, tag, "totalKm", "INFO", tk)
    s = sum(r["km"] for r in routes)
    res.check(name, tag, "sum(route.km) == totalKm", abs(s - tk) <= 1e-3 * max(tk, 1e-9) + 1e-9, s, tk,
              f"diff {tk - s:.1f} km (e.g. totalKm counting the hops between consecutive routes)", soft=True)

    bb = e["bbox"] or {}
    want = dict(west=tl[1], north=tl[0], east=br[1], south=br[0])
    dev = max(abs(float(bb.get(k, float("nan"))) - v) for k, v in want.items()) if bb else float("inf")
    res.check(name, tag, "bbox == corners (deg)", dev <= 1e-4, dev, 1e-4, f"got {bb}")

    max_pts = opts["maxPoints"]
    all_lines = []
    worst_pts = 0
    kml_bad, gpx_bad = [], []
    km_dev_worst = (0.0, None)
    worst_km = 0.0
    for i, r in enumerate(routes):
        try:
            lines = parse_kml(r["kml"])
            if not lines or max(len(l) for l in lines) < 2:
                raise ValueError("no LineString with >=2 coordinates")
            pts = np.vstack(lines)
            if not (np.isfinite(pts).all() and (pts[:, 1] >= lon_lo - eps).all() and (pts[:, 1] <= lon_hi + eps).all()
                    and (pts[:, 0] >= lat_lo - eps).all() and (pts[:, 0] <= lat_hi + eps).all()):
                raise ValueError(f"coords out of range: lat {pts[:, 0].min():.5f}..{pts[:, 0].max():.5f} "
                                 f"lon {pts[:, 1].min():.5f}..{pts[:, 1].max():.5f} (lon,lat order swapped?)")
            all_lines += lines
            n = sum(len(l) for l in lines)
            worst_pts = max(worst_pts, n, int(r["points"] or 0))
            if n != r["points"]:
                res.add(name, tag, f"route {i} reported points != KML coords", "WARN", r["points"], n)
            km = sum(haversine_km(l) for l in lines)
            worst_km = max(worst_km, km)
            d = abs(km - r["km"]) / max(r["km"], 1e-9)
            if d > km_dev_worst[0]:
                km_dev_worst = (d, f"route {i}: reported {r['km']:.3f} km vs haversine(KML) {km:.3f} km")
            if opts.get("maxKm"):
                if km > opts["maxKm"] * 1.01:
                    res.add(name, tag, f"route {i} exceeds maxKm", "FAIL", km, opts["maxKm"])
        except Exception as ex:
            kml_bad.append(f"route {i}: {ex}")
        try:
            g, kinds = parse_gpx(r["gpx"])
            if len(g) < 2:
                raise ValueError("fewer than 2 trkpt/rtept")
            if not ((g[:, 1] >= lon_lo - eps).all() and (g[:, 1] <= lon_hi + eps).all()
                    and (g[:, 0] >= lat_lo - eps).all() and (g[:, 0] <= lat_hi + eps).all()):
                raise ValueError("gpx coords out of range")
            if len(g) > max_pts:
                raise ValueError(f"gpx has {len(g)} points > maxPoints")
        except Exception as ex:
            gpx_bad.append(f"route {i}: {ex}")
    res.check(name, tag, "KML valid (XML, LineString>=2, lon,lat in box)", not kml_bad, len(kml_bad), 0,
              "; ".join(kml_bad[:3]))
    res.check(name, tag, "GPX valid (XML, trkpt/rtept lat/lon in box)", not gpx_bad, len(gpx_bad), 0,
              "; ".join(gpx_bad[:3]))
    res.check(name, tag, "max points per route", worst_pts <= max_pts, worst_pts, max_pts)
    res.check(name, tag, "route.km vs haversine(KML)", km_dev_worst[0] <= 0.005, km_dev_worst[0], 0.005,
              km_dev_worst[1] or "", soft=km_dev_worst[0] <= 0.02)
    if opts.get("maxKm"):
        res.check(name, tag, "longest route (haversine KML) <= maxKm", worst_km <= opts["maxKm"] * 1.01,
                  worst_km, opts["maxKm"])
    mc = max((r["maxConnectorKm"] or 0) for r in routes)
    res.add(name, tag, "max connector km", "INFO", mc)

    # geographic fidelity vs. the spec transform applied to the tool's own parsed geometry
    if parsed is not None and all_lines:
        src = [a for a in parsed["out_polys"] if len(a) >= 2]
        rot = opts["rotationDeg"]
        tries = [rot] if rot in (0, 180) else [rot, -rot]
        best = None
        for r_ in tries:
            exp = map_drawing(src, tl, br, r_)
            d_eg, d_ge, diag_m = geo_hausdorff(exp, all_lines)
            if best is None or max(d_eg, d_ge) < max(best[1], best[2]):
                best = (r_, d_eg, d_ge, diag_m)
        r_, d_eg, d_ge, diag_m = best
        thr = max(1.5 * opts["toleranceM"], 1e-3 * diag_m)
        rdesc = "" if len(tries) == 1 else f" (rotation sense matched: {'CCW/math-positive' if r_ == rot else 'CW'} for rotationDeg={rot})"
        res.check(name, tag, "geo: drawing(mapped) -> routes (m)", d_eg <= thr, d_eg, thr,
                  "max distance of expected mapped drawing from KML routes" + rdesc)
        res.check(name, tag, "geo: route vertices -> drawing (m)", d_ge <= thr, d_ge, thr,
                  "catches mirroring / lat-lon swap / wrong projection")
    return e


# ======================================================================= main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.dirname(HERE))
    ap.add_argument("--page", default="index.html")
    ap.add_argument("--only", default="")
    ap.add_argument("--no-e2e", action="store_true")
    ap.add_argument("--port", type=int, default=0)
    ap.add_argument("--headed", action="store_true")
    ap.add_argument("--json", default=os.path.join(HERE, "last_results.json"))
    ap.add_argument("--ready-timeout", type=float, default=30.0)
    args = ap.parse_args()
    only = [s for s in args.only.split(",") if s]

    from playwright.sync_api import sync_playwright

    res = Results()
    port = args.port or free_port()
    server = start_server(args.root, port)
    url = f"http://127.0.0.1:{port}/{args.page}"
    console_errors, page_errors, console_warnings = [], [], []
    t_start = time.time()
    try:
        with sync_playwright() as pw:
            browser = launch(pw, args.headed)
            page = browser.new_page()
            page.on("console", lambda m: (console_errors if m.type == "error" else
                                          console_warnings if m.type == "warning" else []).append(m.text))
            page.on("pageerror", lambda e: page_errors.append(str(e)))
            page.set_default_timeout(300_000)
            print(f"Opening {url}")
            page.goto(url)
            try:
                page.wait_for_function("window.RWGPSConv && window.RWGPSConv.ready === true",
                                       timeout=args.ready_timeout * 1000)
            except Exception:
                res.add("(page)", "load", "window.RWGPSConv.ready", "FAIL",
                        detail=f"not ready after {args.ready_timeout}s; page errors: {page_errors[:3]} "
                               f"console errors: {console_errors[:3]}")
                raise SystemExit(finish(res, console_errors, page_errors, console_warnings, args, t_start))
            api = page.evaluate("Object.keys(window.RWGPSConv)")
            print(f"RWGPSConv keys: {api}")
            for fn in ("parseDXF", "parseSVG", "order", "exportRoutes"):
                res.check("(page)", "load", f"RWGPSConv.{fn} is a function",
                          page.evaluate(f"typeof window.RWGPSConv.{fn} === 'function'"))

            refs = load_refs(only)
            parsed_by_name = {}
            for ref in refs:
                name = ref["fixture"]
                print(f"\n== {name}  ({ref['kind']}{', ADVISORY' if ref.get('advisory') else ''}) {ref['notes'][:110]}")
                parsed = test_parse(page, ref, res)
                if parsed is None:
                    continue
                parsed_by_name[name] = parsed
                o = test_order(page, ref, parsed, res)
                if o is None or args.no_e2e:
                    continue
                base = dict(rotationDeg=0, toleranceM=50, maxPoints=6000, maxKm=0, name="Squiggle")
                if ref.get("expect", {}).get("e2e"):
                    check_export(page, name, res, dict(base, tl=SQ_TL, br=SQ_BR), parsed=parsed,
                                 expect_total_km=SQ_TOTAL_KM, routes_range=(20, 25), tag="e2e")
                    if ref["kind"] == "dxf":
                        check_export(page, name, res, dict(base, tl=SQ_TL, br=SQ_BR, maxKm=3000), parsed=parsed,
                                     tag="maxKm")
                elif name in ("lines.dxf", "shapes.svg", "arcs.dxf"):
                    check_export(page, name, res, dict(base, tl=BV_TL, br=BV_BR, name="Belleville"),
                                 parsed=parsed, expect_routes=1, tag="e2e-bv")
                    if name == "lines.dxf":
                        check_export(page, name, res, dict(base, tl=BV_TL, br=BV_BR, rotationDeg=90,
                                                           name="Belleville rot"),
                                     parsed=parsed, expect_routes=1, tag="rot90")
                # free page memory between fixtures
                page.evaluate("() => { window.__parsed = null; window.__ordered = null; }")
            browser.close()
    finally:
        server.kill()
    return finish(res, console_errors, page_errors, console_warnings, args, t_start)


def finish(res, console_errors, page_errors, console_warnings, args, t_start):
    for e in page_errors:
        res.add("(page)", "js", "uncaught page error", "FAIL", detail=e[:300])
    print("\n== Browser console")
    print(f"  errors: {len(console_errors)}   warnings: {len(console_warnings)}   uncaught: {len(page_errors)}")
    for e in console_errors[:30]:
        print("  [error]", e[:300])
    for e in page_errors[:30]:
        print("  [uncaught]", e[:300])
    for e in console_warnings[:10]:
        print("  [warning]", e[:200])
    if console_errors:
        res.add("(page)", "js", "console errors", "WARN", len(console_errors))

    # ------------------------------------------------------------ summary table
    print("\n== Summary")
    fixtures = []
    for r in res.rows:
        if r["fixture"] not in fixtures:
            fixtures.append(r["fixture"])

    def get(fx, phase, check):
        for r in res.rows:
            if r["fixture"] == fx and r["phase"] == phase and r["check"] == check:
                return r
        return None

    def num(fx, phase, check, f="{:.2e}"):
        r = get(fx, phase, check)
        if not r or r["value"] is None:
            return "-"
        try:
            return f.format(r["value"])
        except Exception:
            return str(r["value"])

    hdr = f"{'fixture':20s} {'result':6s} {'ref->out':>9s} {'out->ref':>9s} {'tol':>8s} {'len err':>9s} " \
          f"{'parse ms':>9s} {'order ms':>9s} {'sum jumps':>10s} {'e2e ms':>8s} {'totalKm':>9s} {'routes':>6s}"
    print(hdr)
    print("-" * len(hdr))
    nfail = nwarn = 0
    for fx in fixtures:
        rows = [r for r in res.rows if r["fixture"] == fx]
        f = sum(r["status"] == "FAIL" for r in rows)
        w = sum(r["status"] == "WARN" for r in rows)
        nfail += f
        nwarn += w
        status = "FAIL" if f else ("WARN" if w else "PASS")
        tolr = get(fx, "parse", "maxdist ref->out")
        print(f"{fx:20s} {status:6s} {num(fx, 'parse', 'maxdist ref->out'):>9s} {num(fx, 'parse', 'maxdist out->ref'):>9s} "
              f"{(fmt(tolr['threshold']) if tolr else '-'):>8s} {num(fx, 'parse', 'length rel err', '{:+.2e}'):>9s} "
              f"{num(fx, 'parse', 'time_ms', '{:.0f}'):>9s} {num(fx, 'order', 'time_ms', '{:.0f}'):>9s} "
              f"{num(fx, 'order', 'sum(jumps)', '{:.1f}'):>10s} "
              f"{(num(fx, 'e2e', 'time_ms', '{:.0f}') if get(fx, 'e2e', 'time_ms') else num(fx, 'e2e-bv', 'time_ms', '{:.0f}')):>8s} "
              f"{num(fx, 'e2e', 'totalKm', '{:.0f}'):>9s} "
              f"{(num(fx, 'e2e', 'routes', '{}') if get(fx, 'e2e', 'routes') else num(fx, 'e2e-bv', 'routes', '{}')):>6s}")
    print("-" * len(hdr))
    fails = [r for r in res.rows if r["status"] == "FAIL"]
    warns = [r for r in res.rows if r["status"] == "WARN"]
    if fails:
        print(f"\nFAILURES ({len(fails)}):")
        for r in fails:
            print(f"  {r['fixture']:20s} {r['phase']:6s} {r['check']}: value={fmt(r['value']) if r['value'] is not None else '-'}"
                  f" limit={fmt(r['threshold']) if r['threshold'] is not None else '-'} {r['detail'][:200]}")
    if warns:
        print(f"\nWARNINGS ({len(warns)}):")
        for r in warns:
            print(f"  {r['fixture']:20s} {r['phase']:6s} {r['check']}: value={fmt(r['value']) if r['value'] is not None else '-'}"
                  f" limit={fmt(r['threshold']) if r['threshold'] is not None else '-'} {r['detail'][:160]}")
    total = sum(r["status"] in ("PASS", "FAIL", "WARN") for r in res.rows)
    print(f"\n{total} checks: {sum(r['status'] == 'PASS' for r in res.rows)} pass, {nwarn} warn, {nfail} FAIL"
          f"   ({time.time() - t_start:.1f} s)")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(dict(rows=res.rows, console_errors=console_errors, page_errors=page_errors), f, indent=1,
                      default=str)
    return 1 if nfail else 0


if __name__ == "__main__":
    sys.exit(main())
