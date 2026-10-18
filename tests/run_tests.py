#!/usr/bin/env python3
"""Browser test harness for the route-art DXF/SVG -> RWGPS converter.

Usage (from anywhere):
    python3 -I /home/claude/route-art/tests/run_tests.py [options]

Options:
    --root DIR        site root to serve (default: parent of this tests/ dir)
    --page PATH       page to open, relative to root (default: index.html;
                      tests/_mock/index.html runs the harness against the mock)
    --only a,b        only fixtures whose file name contains one of these substrings
    --no-e2e          skip exportRoutes / exportGroups end-to-end checks
    --fill            only the squiggle-fill section (fill fixtures + buildShapes/squiggleFill/prepare/
                      exportGroups checks + UI smoke). The default run includes it too.
    --no-fill         skip the squiggle-fill section
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


def split_spurs(curves, tol):
    """order() may splice a closed loop into a neighbouring curve with an out-and-back spur
    (segment A->B, later B->A, same vertices). Remove such retraced segment pairs (longer than tol)
    and split the curves there. Returns (pieces, spur_len, n_spurs)."""
    pieces, spur_len, n = [], 0.0, 0
    split_spurs.max_len = 0.0
    for c in curves:
        c = as_np(c)
        if len(c) < 3:
            pieces.append(c)
            continue
        a, b = c[:-1], c[1:]
        fwd = {}
        for i, (p, q) in enumerate(zip(map(tuple, a), map(tuple, b))):
            fwd.setdefault((p, q), []).append(i)
        L = np.hypot(*(b - a).T)
        spur = np.zeros(len(a), bool)
        for (p, q), idx in fwd.items():
            if p == q:
                continue
            tw = fwd.get((q, p))
            if tw:
                for i in idx:
                    if L[i] > tol:
                        spur[i] = True
        if spur.any():
            spur_len += float(L[spur].sum())
            split_spurs.max_len = max(split_spurs.max_len, float(L[spur].max()))
            n += int(spur.sum())
            start = 0
            for i in np.flatnonzero(spur):
                if i + 1 - start >= 2:
                    pieces.append(c[start:i + 1])
                start = i + 1
            if len(c) - start >= 2:
                pieces.append(c[start:])
        else:
            pieces.append(c)
    return pieces, spur_len, n // 2


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


JS_EXPORT_GROUPS = """
async (opts) => {
  const C = window.RWGPSConv;
  const t0 = performance.now();
  let e = C.exportGroups(window.__prepared, opts);
  const isPromise = !!(e && typeof e.then === 'function');
  if (isPromise) e = await e;
  const ms = performance.now() - t0;
  return {ms, isPromise, totalKm: e.totalKm, bbox: e.bbox,
          routes: (e.routes || []).map(r => ({name: r.name, points: r.points, km: r.km, group: r.group ?? null,
                   index: r.index ?? null, maxConnectorKm: r.maxConnectorKm, kml: r.kml, gpx: r.gpx}))};
}
"""

# ---- squiggle fill API
JS_SHAPES = """
async () => {
  const C = window.RWGPSConv;
  const t0 = performance.now();
  let b = C.buildShapes(window.__parsed.paths);
  const isPromise = !!(b && typeof b.then === 'function');
  if (isPromise) b = await b;
  const ms = performance.now() - t0;
  window.__built = b;
  return {ms, isPromise,
          shapes: (b.shapes || []).map(s => ({rings: (s.rings || []).map(r => Array.from(r, q => [Number(q[0]), Number(q[1])]))})),
          open: (b.open || []).map(p => ({layer: p.layer ?? null, n: (p.pts || []).length}))};
}
"""

JS_FILL = """
async ({i, spacing, seed, wobble, det}) => {
  const C = window.RWGPSConv;
  const rings = window.__built.shapes[i].rings;
  const o = {spacing, seed};
  if (wobble !== null && wobble !== undefined) o.wobble = wobble;
  const run = async (opts) => { let r = C.squiggleFill(rings, opts); if (r && typeof r.then === 'function') r = await r; return r; };
  const t0 = performance.now();
  const loops = await run(o);
  const ms = performance.now() - t0;
  let same = null, diff = null, ms2 = null;
  if (det) {
    const key = JSON.stringify(loops);
    const t1 = performance.now();
    const b = await run({...o});
    ms2 = performance.now() - t1;
    same = JSON.stringify(b) === key;
    const c = await run({...o, seed: seed + 1});
    diff = JSON.stringify(c) !== key;
  }
  return {ms, ms2, same, diff, loops: (loops || []).map(l => Array.from(l, q => [Number(q[0]), Number(q[1])]))};
}
"""

JS_PREPARE = """
async (opts) => {
  const C = window.RWGPSConv;
  const t0 = performance.now();
  let p = C.prepare(window.__parsed.paths, opts);
  const isPromise = !!(p && typeof p.then === 'function');
  if (isPromise) p = await p;
  const ms = performance.now() - t0;
  window.__prepared = p;
  return {ms, isPromise, ncurves: (p.curves || []).length,
          groups: (p.groups || []).map(g => ({name: g.name ?? null,
              curves: (g.curves || []).map(c => Array.from(c, q => [Number(q[0]), Number(q[1])])),
              jumps: Array.from(g.jumps || [], Number)}))};
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
    curves_l, spur_len, n_spurs = split_spurs([c for c in curves if len(c) >= 2], tol)
    curves_l = [c for c in curves_l if len(c) >= 2]
    max_spur = split_spurs.max_len
    if n_spurs:
        js_ = float(sum(o["jumps"]))
        res.add(name, "order", "loop splice spurs (retraced connectors inside curves)", "INFO", n_spurs,
                detail=f"max spur {max_spur:.3f}; 2 x sum(spur) = {spur_len:.3f}; 2 x spurs + sum(jumps) = "
                       f"{spur_len + js_:.3f}; spurs excluded from the geometry comparisons below")
    d_io, at_io, d_oi, at_oi = hausdorff(in_polys, None, curves_l, tol)
    res.check(name, "order", "maxdist parsed->curves", d_io <= tol, d_io, tol,
              f"worst {np.round(at_io, 4).tolist() if at_io else None}" + ("" if d_io <= tol else " (geometry LOST by order())"))
    res.check(name, "order", "maxdist curves->parsed", d_oi <= tol, d_oi, tol,
              f"worst {np.round(at_oi, 4).tolist() if at_oi else None}" + ("" if d_oi <= tol else " (order() INVENTED geometry - a connector inside a curve that is not a retraced out-and-back spur?)"))
    Lin = sum(poly_length(a) for a in in_polys)
    Lout = sum(poly_length(a) for a in curves_l)
    rel = (Lout - Lin) / Lin if Lin else 0
    res.check(name, "order", "curve length vs parsed", abs(rel) <= 0.005, rel, 0.005,
              "loops may be rotated; length must be preserved (retraced spurs excluded)")
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
                 km_tol=0.02, routes_range=None, tag="e2e", js=JS_EXPORT, fn="exportRoutes"):
    try:
        e = page.evaluate(js, opts)
    except Exception as ex:
        res.add(name, tag, f"{fn}() threw", "FAIL", detail=str(ex).splitlines()[0][:300])
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


# ======================================================================= squiggle fill
FILL_SEED = 1
FILL_WOBBLE = 0.22          # the UI default (prepare() also defaults to it)
FILL_MS_BIKE_120 = 2000.0   # bike at spacing h/120 should fill in < 2 s
FILL_SAMPLES = 2000
FILL_COV_MAX = 2.5   # necks 2–2.5x spacing wide (e.g. where the bike i meets the k) can stay unfilled: no whole grid cell fits
FILL_COV_P95 = 0.75


def tool_polygon(rings):
    from shapely.geometry import Polygon
    rs = [as_np(r) for r in rings if len(r) >= 3]
    if not rs:
        return Polygon()
    return Polygon(rs[0], rs[1:])


def match_shapes(tool_polys, ref_polys):
    """For each ref polygon, the index of the tool polygon with the largest overlap (or None)."""
    out = []
    for rp in ref_polys:
        best, bi = 0.0, None
        for i, tp in enumerate(tool_polys):
            try:
                a = tp.intersection(rp).area if tp.is_valid else shapely.make_valid(tp).intersection(rp).area
            except Exception:
                a = 0.0
            if a > best:
                best, bi = a, i
        out.append(bi)
    return out


def boundary_tree(poly):
    rings = [np.asarray(poly.exterior.coords)] + [np.asarray(r.coords) for r in poly.interiors]
    t, _ = segments_tree(rings)
    return t


def dist_to(tree, pts):
    if tree is None or len(pts) == 0:
        return np.full(len(pts), np.inf)
    _, d = tree.query_nearest(shapely.points(pts), return_distance=True, all_matches=False)
    return d


def check_loops(res, name, tag, sname, poly, loops, spacing, expect_loops, rng, stats):
    """Geometry checks for squiggleFill output of ONE shape (poly = the rings the tool was given)."""
    from shapely.geometry import LineString
    lab = f"[{sname}]"
    arrs = [as_np(l) for l in loops]
    nv = sum(len(a) for a in arrs)
    L = sum(poly_length(a) for a in arrs)
    stats.update(loops=len(arrs), verts=nv, length=L)
    # loop count
    if expect_loops:
        lo, hi = expect_loops
        res.check(name, tag, f"{lab} loop count", lo <= len(arrs) <= hi, len(arrs), f"{lo}..{hi}" if lo != hi else lo)
    else:
        res.add(name, tag, f"{lab} loop count", "INFO", len(arrs), detail=f"{nv} vertices, length {L:.1f}")
    if not arrs:
        stats.update(min_clear=None, min_seg_clear=None, cov_max=None, cov_p95=None, cov_deep=None)
        return
    diag = math.hypot(*(np.ptp(np.vstack([np.asarray(poly.exterior.coords)]), axis=0)))
    finite = all(np.isfinite(a).all() for a in arrs)
    res.check(name, tag, f"{lab} all finite", finite)
    if not finite:
        return
    # closed
    gaps = [float(np.hypot(*(a[0] - a[-1]))) if len(a) else float("inf") for a in arrs]
    short = sum(1 for a in arrs if len(a) < 4)
    res.check(name, tag, f"{lab} loops closed (first == last)", max(gaps) <= 1e-9 * diag and not short,
              max(gaps), 1e-9 * diag, f"{short} loops with < 4 points" if short else "")
    # inside + clearance
    P = np.vstack(arrs)
    inside = shapely.contains_xy(poly, P[:, 0], P[:, 1])
    n_out = int((~inside).sum())
    res.check(name, tag, f"{lab} vertices inside shape", n_out == 0, n_out, 0,
              f"e.g. {np.round(P[~inside][0], 3).tolist()}" if n_out else f"{len(P)} vertices")
    d = dist_to(boundary_tree(poly), P)
    i = int(np.argmin(d))
    mc = float(d[i]) / spacing
    stats["min_clear"] = mc
    below05 = float(np.mean(d < 0.5 * spacing))
    res.check(name, tag, f"{lab} min clearance / spacing", mc >= 0.3 and n_out == 0, mc, 0.3,
              f"at {np.round(P[i], 3).tolist()}; {below05:.2%} of vertices < 0.5 x spacing")
    # segments: must not cross the boundary; clearance of whole segments (a notch tip can poke
    # between two clear vertices)
    lines = [LineString(a) for a in arrs if len(a) >= 2]
    bnd = poly.boundary
    crossing = [k for k, ls in enumerate(lines) if not poly.contains(ls)]
    res.check(name, tag, f"{lab} loop segments inside shape (no boundary crossing)", not crossing, len(crossing), 0,
              f"loops {crossing[:5]}" if crossing else "")
    segd = [bnd.distance(ls) for ls in lines]
    ks = int(np.argmin(segd))
    from shapely.ops import nearest_points
    wp = nearest_points(lines[ks], bnd)
    stats["min_seg_clear"] = segd[ks] / spacing
    res.check(name, tag, f"{lab} min SEGMENT clearance / spacing", segd[ks] >= 0.3 * spacing, segd[ks] / spacing, 0.3,
              f"loop {ks} passes {segd[ks]:.3f} from boundary point {np.round(wp[1].coords[0], 3).tolist()}", soft=True)
    # simple + pairwise disjoint
    bad = [k for k, ls in enumerate(lines) if not ls.is_simple]
    det = ""
    if bad:
        ls = lines[bad[0]]
        u = shapely.unary_union(ls)        # noded: extra nodes = self-intersections
        segs = getattr(u, "geoms", [u])
        ends = np.array([c for s in segs for c in (s.coords[0], s.coords[-1])])
        uq, cnt = np.unique(np.round(ends, 9), axis=0, return_counts=True)
        x = uq[cnt > 2][:3] if (cnt > 2).any() else uq[:0]
        det = f"loop {bad[0]} ({len(arrs[bad[0]])} pts) self-intersects near {np.round(x, 3).tolist()}"
    res.check(name, tag, f"{lab} loops simple (no self-intersections)", not bad, len(bad), 0, det)
    tree = STRtree(lines)
    pairs = tree.query(lines, predicate="intersects")
    pairs = {(int(a), int(b)) for a, b in zip(*pairs) if a < b}
    res.check(name, tag, f"{lab} loops pairwise disjoint", not pairs, len(pairs), 0,
              f"e.g. loops {sorted(pairs)[:3]}" if pairs else "")
    # coverage of the shape eroded by 1 x spacing
    er = poly.buffer(-1.0 * spacing)
    if er.is_empty:
        res.add(name, tag, f"{lab} coverage", "INFO", detail="shape eroded by 1 x spacing is empty")
        stats.update(cov_max=None, cov_p95=None, cov_deep=None)
        return
    x0, y0, x1, y1 = er.bounds
    pts = np.zeros((0, 2))
    tries = 0
    while len(pts) < FILL_SAMPLES and tries < 200:
        c = rng.uniform([x0, y0], [x1, y1], size=(FILL_SAMPLES * 4, 2))
        c = c[shapely.contains_xy(er, c[:, 0], c[:, 1])]
        pts = np.vstack([pts, c])[:FILL_SAMPLES]
        tries += 1
    lt, _ = segments_tree(arrs)
    dd = dist_to(lt, pts) / spacing
    j = int(np.argmax(dd))
    p95 = float(np.percentile(dd, 95))
    stats.update(cov_max=float(dd[j]), cov_p95=p95)
    # The bike's letter strokes are only ~2 cells wide at h/40, so whole 2x-spacing cells
    # can't cover them: that's a density limit, not a defect -> advisory there.
    soft_cov = "bike" in name and "/40" in tag
    res.check(name, tag, f"{lab} coverage p95 dist / spacing", p95 <= FILL_COV_P95, p95, FILL_COV_P95,
              f"{float(np.mean(dd > 1.0)):.2%} of samples > 1 x spacing", soft=soft_cov)
    res.check(name, tag, f"{lab} coverage max dist / spacing", dd[j] <= FILL_COV_MAX, float(dd[j]), FILL_COV_MAX, soft=soft_cov, detail=
              f"p95 {p95:.3f}; worst sample at {np.round(pts[j], 3).tolist()}; {len(pts)} samples in shape eroded "
              f"by 1 x spacing; {float(np.mean(dd > 1.0)):.2%} of samples > 1 x spacing")
    # diagnostic: is the shortfall an edge band or a hole in the interior?
    depth = dist_to(boundary_tree(poly), pts) / spacing
    deep = depth >= 2.5
    stats["cov_deep"] = float(dd[deep].max()) if deep.any() else None
    fill_frac = L * spacing / poly.area if poly.area else 0.0
    stats["len_ratio"] = fill_frac
    res.add(name, tag, f"{lab} coverage diag", "INFO", stats["cov_deep"],
            detail=f"max dist/spacing for samples >= 2.5 x spacing deep ({int(deep.sum())} samples); worst sample "
                   f"depth {depth[j]:.2f} x spacing; fill length x spacing / area = {fill_frac:.2f} (1.0 = evenly filled)")


def classify_curves(curves, polys, spacing, on_tol):
    """Split prepared curves into ring geometry (all vertices on a shape boundary) and the rest.
    Returns (ring_len, fill_len, n_between, n_outside, worst_between)."""
    trees = [boundary_tree(p) for p in polys]
    allp = np.vstack(curves) if curves else np.zeros((0, 2))
    if not len(allp):
        return 0.0, 0.0, 0, 0, None
    db = np.min(np.stack([dist_to(t, allp) for t in trees]), axis=0) if trees else np.full(len(allp), np.inf)
    ins = np.zeros(len(allp), bool)
    for p in polys:
        ins |= shapely.contains_xy(p, allp[:, 0], allp[:, 1])
    ring_len = fill_len = 0.0
    k = 0
    for c in curves:
        n = len(c)
        on = db[k:k + n] <= on_tol
        if on.all():
            ring_len += poly_length(c)
        else:
            fill_len += poly_length(c)
        k += n
    between = (db > on_tol) & (db < 0.3 * spacing)
    outside = (db > on_tol) & ~ins
    wb = allp[np.argmax(between)].tolist() if between.any() else None
    return ring_len, fill_len, int(between.sum()), int(outside.sum()), wb


def group_shape_ids(curves, polys, on_tol):
    """Which shapes the vertices of a group fall in (inside or on the boundary)."""
    P = np.vstack(curves)
    ids = set()
    unassigned = 0
    trees = [boundary_tree(p) for p in polys]
    for i, p in enumerate(polys):
        m = shapely.contains_xy(p, P[:, 0], P[:, 1]) | (dist_to(trees[i], P) <= on_tol)
        if m.any():
            ids.add(i)
    return ids


def map_with_box(polys_for_box, polys, tl, br):
    """map_drawing() of `polys` using the bbox of `polys_for_box` (rotation 0)."""
    allp = np.vstack(polys_for_box)
    mn, mx = allp.min(0), allp.max(0)
    TL = merc(np.array(tl[0]), np.array(tl[1]))
    BR = merc(np.array(br[0]), np.array(br[1]))
    out = []
    for a in polys:
        X = TL[0] + (a[:, 0] - mn[0]) / (mx[0] - mn[0]) * (BR[0] - TL[0])
        Y = BR[1] + (a[:, 1] - mn[1]) / (mx[1] - mn[1]) * (TL[1] - BR[1])
        out.append(np.stack([X, Y], -1))     # Mercator metres
    return out


def test_fill(page, ref, parsed, res, args):
    from shapely.geometry import Polygon
    name = ref["fixture"]
    F = ref["fill"]
    have = page.evaluate("['buildShapes','squiggleFill','prepare','exportGroups']"
                         ".map(k => [k, typeof window.RWGPSConv[k] === 'function'])")
    missing = [k for k, ok in have if not ok]
    if missing:
        res.add(name, "fill", "fill API missing", "FAIL", detail=", ".join(missing))
        return None
    # ------------------------------------------------ buildShapes
    try:
        b = page.evaluate(JS_SHAPES)
    except Exception as e:
        res.add(name, "shapes", "buildShapes() threw", "FAIL", detail=str(e).splitlines()[0][:300])
        return None
    shapes = b["shapes"]
    polys = [tool_polygon(s["rings"]) for s in shapes]
    ref_polys = [Polygon(s["rings"][0], s["rings"][1:]) for s in F["shapes"]]
    soft_rec = F.get("record_only", False)
    res.add(name, "shapes", "time_ms", "INFO", round(b["ms"], 1),
            detail=f"{len(shapes)} shapes, holes {[len(s['rings']) - 1 for s in shapes]}, "
                   f"areas {[round(p.area, 1) for p in polys]}, open {len(b['open'])}")
    if b["isPromise"]:
        res.add(name, "shapes", "returned a Promise (spec: sync)", "WARN")
    if soft_rec:
        def desc(ps):
            return f"{len(ps)} shapes, holes {[len(p.interiors) for p in ps]}, areas {[round(p.area, 1) for p in ps]}"
        alt = [Polygon(s["rings"][0], s["rings"][1:]) for s in F.get("alt", {}).get("shapes", [])]
        def same(a, bb):
            return len(a) == len(bb) and sorted(len(p.interiors) for p in a) == sorted(len(p.interiors) for p in bb) \
                and all(abs(x - y) <= 0.01 * y for x, y in zip(sorted(p.area for p in a), sorted(p.area for p in bb)))
        verdict = ("per-element fill then union (counters filled, no holes)" if same(polys, ref_polys) else
                   "even-odd across elements (counters became holes)" if alt and same(polys, alt) else "neither")
        res.add(name, "shapes", "RECORDED behaviour (nonzero, separate elements)", "INFO", len(shapes),
                detail=f"tool: {desc(polys)}; union-ref: {desc(ref_polys)}; evenodd-ref: {desc(alt)} => matches {verdict}")
    else:
        res.check(name, "shapes", "shape count", len(shapes) == len(ref_polys), len(shapes), len(ref_polys))
    res.check(name, "shapes", "open paths", len(b["open"]) == F.get("open_count", 0), len(b["open"]),
              F.get("open_count", 0), soft=soft_rec)
    valid = [p.is_valid for p in polys]
    res.check(name, "shapes", "shape polygons valid (shapely)", all(valid), sum(not v for v in valid), 0,
              "; ".join(shapely.is_valid_reason(p) for p, v in zip(polys, valid) if not v)[:200], soft=True)
    m = match_shapes(polys, ref_polys)
    names = {}
    for rs, rp, ti in zip(F["shapes"], ref_polys, m):
        if ti is None:
            res.add(name, "shapes", f"[{rs['name']}] no matching tool shape", "WARN" if soft_rec else "FAIL")
            continue
        names[ti] = rs
        tp = polys[ti]
        rel = (tp.area - rp.area) / rp.area
        res.check(name, "shapes", f"[{rs['name']}] area rel err", abs(rel) <= 0.01, rel, 0.01,
                  f"tool {tp.area:.2f} vs shapely {rp.area:.2f} (tool shape #{ti})", soft=soft_rec)
        res.check(name, "shapes", f"[{rs['name']}] holes", len(tp.interiors) == rs["holes"], len(tp.interiors),
                  rs["holes"], soft=soft_rec)
        sd = shapely.symmetric_difference(shapely.make_valid(tp), rp).area / rp.area
        res.check(name, "shapes", f"[{rs['name']}] sym-diff area / area", sd <= 0.01, sd, 0.01, soft=soft_rec)
    if len(set(i for i in m if i is not None)) != len([i for i in m if i is not None]):
        res.add(name, "shapes", "two reference shapes matched the same tool shape", "WARN" if soft_rec else "FAIL")
    if not shapes:
        return None

    H = F["height"]
    rng = np.random.default_rng(12345)
    summary = []
    # ------------------------------------------------ squiggleFill per shape
    runs = [(div, None, True, f"fill/{div}") for div in F["spacing_divs"]]
    runs.append((F["spacing_divs"][0], FILL_WOBBLE, False, f"fillW/{F['spacing_divs'][0]}"))
    for div, wob, det, tag in runs:
        spacing = H / div
        tot_ms = 0.0
        agg = dict(loops=0, verts=0, min_clear=None, min_seg_clear=None, cov_max=None, cov_p95=None, cov_deep=None)
        res.add(name, tag, "spacing", "INFO", spacing,
                detail=f"height {H:.3f} / {div}; seed {FILL_SEED}" + (f"; wobble {wob}" if wob is not None else "; default opts"))
        for i, poly in enumerate(polys):
            rs = names.get(i)
            sname = rs["name"] if rs else f"shape {i}"
            exp = rs.get("loops") if (rs and div == F["spacing_divs"][0] and wob is None) else None
            try:
                o = page.evaluate(JS_FILL, dict(i=i, spacing=spacing, seed=FILL_SEED, wobble=wob, det=det))
            except Exception as e:
                res.add(name, tag, f"[{sname}] squiggleFill() threw", "FAIL", detail=str(e).splitlines()[0][:300])
                continue
            tot_ms += o["ms"]
            st = {}
            check_loops(res, name, tag, sname, poly if poly.is_valid else shapely.make_valid(poly), o["loops"],
                        spacing, exp, rng, st)
            res.add(name, tag, f"[{sname}] time_ms", "INFO", round(o["ms"], 1),
                    detail=(f"repeat {o['ms2']:.1f} ms; " if o["ms2"] is not None else "") +
                           f"{st.get('verts', 0)} vertices")
            if det:
                res.check(name, tag, f"[{sname}] same seed -> identical", bool(o["same"]), o["same"], True)
                cells = poly.area / (2 * spacing) ** 2
                if o["loops"]:
                    res.check(name, tag, f"[{sname}] different seed -> different", bool(o["diff"]), o["diff"], True,
                              f"~{cells:.0f} grid cells", soft=cells < 10)
            agg["loops"] += st.get("loops", 0)
            agg["verts"] += st.get("verts", 0)
            for k, f in (("min_clear", min), ("min_seg_clear", min), ("cov_max", max), ("cov_p95", max),
                         ("cov_deep", max)):
                if st.get(k) is not None:
                    agg[k] = st[k] if agg[k] is None else f(agg[k], st[k])
        res.add(name, tag, "total fill time_ms", "INFO", round(tot_ms, 1), detail=f"{len(polys)} shapes")
        if "bike" in name and div == 120 and wob is None:
            res.check(name, tag, "bike fill time at h/120 (ms)", tot_ms < FILL_MS_BIKE_120, round(tot_ms, 1),
                      FILL_MS_BIKE_120, soft=True)
        elif tot_ms > SLOW_MS:
            res.add(name, tag, "SLOW", "WARN", round(tot_ms), SLOW_MS)
        summary.append(dict(fixture=name, run=tag, spacing=spacing, ms=tot_ms, **agg))

    # ------------------------------------------------ prepare()
    spacing = H / F["spacing_divs"][0]
    n_open = len(b["open"])
    on_tol = 1e-4 * parsed["diag"]
    perim = sum(poly_length(np.asarray(r.coords)) for p in polys for r in [p.exterior, *p.interiors])
    filled = set()
    got = {}
    fill_ref_len = 0.0          # squiggleFill with prepare()'s options (seed, default wobble)
    for i in range(len(polys)):
        try:
            o = page.evaluate(JS_FILL, dict(i=i, spacing=spacing, seed=FILL_SEED, wobble=FILL_WOBBLE, det=False))
            fill_ref_len += sum(poly_length(as_np(l)) for l in o["loops"])
        except Exception:
            pass
    for mode in ("outline", "both", "fill"):
        for grouping in ("shape", "join"):
            tag = "prep"
            lab = f"[{mode}/{grouping}]"
            try:
                p = page.evaluate(JS_PREPARE, dict(mode=mode, grouping=grouping, spacing=spacing, seed=FILL_SEED))
            except Exception as e:
                res.add(name, tag, f"{lab} prepare() threw", "FAIL", detail=str(e).splitlines()[0][:300])
                continue
            if p["isPromise"]:
                res.add(name, tag, f"{lab} returned a Promise (spec: synchronous)", "WARN")
            groups = p["groups"]
            curves_raw = [as_np(c) for g in groups for c in g["curves"] if len(c)]
            curves, spur_len, n_spurs = split_spurs(curves_raw, on_tol)
            got[(mode, grouping)] = (p, curves)
            total = sum(poly_length(c) for c in curves)
            if n_spurs:
                res.add(name, tag, f"{lab} loop splice spurs", "INFO", n_spurs,
                        detail=f"spur length {spur_len:.2f} excluded from the length checks")
            res.add(name, tag, f"{lab} groups", "INFO", len(groups),
                    detail=f"{round(p['ms'], 1)} ms; names {[g['name'] for g in groups][:8]}; "
                           f"curves/group {[len(g['curves']) for g in groups][:8]}; length {total:.1f}")
            res.check(name, tag, f"{lab} top-level curves == concat(groups)", p["ncurves"] == len(curves_raw),
                      p["ncurves"], len(curves_raw), soft=True)
            if grouping == "join":
                res.check(name, tag, f"{lab} group count", len(groups) == 1, len(groups), 1)
            elif mode == "fill":
                pass   # checked below once the fill loops are known
            else:
                want = len(polys) + (1 if n_open else 0)
                res.check(name, tag, f"{lab} group count == shapes(+open)", len(groups) == want, len(groups), want)
            # jumps
            for gi, g in enumerate(groups):
                cs = [as_np(c) for c in g["curves"]]
                if len(g["jumps"]) != max(0, len(cs) - 1):
                    res.add(name, tag, f"{lab} group {gi} jumps length != curves-1", "WARN", len(g["jumps"]),
                            max(0, len(cs) - 1))
                elif len(cs) > 1:
                    gaps = np.array([np.hypot(*(cs[k + 1][0] - cs[k][-1])) for k in range(len(cs) - 1)])
                    dev = float(np.max(np.abs(gaps - np.array(g["jumps"]))))
                    if dev > 1e-6 * parsed["diag"] + 1e-9:
                        res.add(name, tag, f"{lab} group {gi} jumps != gaps", "WARN", dev, 1e-6 * parsed["diag"])
            if not curves:
                res.add(name, tag, f"{lab} no geometry", "FAIL")
                continue
            if mode == "outline" and grouping == "join":
                continue    # join+outline = the raw drawing (covered by the order checks)
            rl, fl, nb, no, wb = classify_curves(curves, polys, spacing, on_tol)
            if mode == "outline":
                res.check(name, tag, f"{lab} only ring geometry (non-ring length)", fl == 0, fl, 0)
                res.check(name, tag, f"{lab} ring length vs shape perimeters", abs(rl - perim) <= 0.005 * perim,
                          (rl - perim) / perim, 0.005)
            else:
                if mode == "fill":
                    res.check(name, tag, f"{lab} no ring geometry (ring length)", rl == 0, rl, 0)
                else:
                    res.add(name, tag, f"{lab} ring-only pieces length vs perimeters", "INFO", (rl - perim) / perim,
                            detail="after removing spurs; informational")
                    # every ring vertex lies on some output curve
                    ct, _ = segments_tree(curves_raw)
                    rv = np.vstack([np.asarray(r.coords) for pp in polys for r in [pp.exterior, *pp.interiors]])
                    dv = dist_to(ct, rv)
                    iw = int(np.argmax(dv))
                    res.check(name, tag, f"{lab} every ring vertex on an output curve", dv[iw] <= on_tol, float(dv[iw]),
                              on_tol, f"worst ring vertex {np.round(rv[iw], 3).tolist()}")
                    tot_raw = sum(poly_length(c) for c in curves_raw)
                    need = perim + fill_ref_len
                    res.check(name, tag, f"{lab} total length >= perimeters + squiggleFill lengths",
                              tot_raw >= need * (1 - 0.005), tot_raw / need, ">= 0.995",
                              f"output {tot_raw:.1f} (incl. spurs) vs perimeters {perim:.1f} + fill {fill_ref_len:.1f}")
                res.add(name, tag, f"{lab} fill length", "INFO", fl)
                res.check(name, tag, f"{lab} vertices outside every shape", no == 0, no, 0)
                res.check(name, tag, f"{lab} fill vertices with clearance < 0.3 x spacing", nb == 0, nb, 0,
                          f"e.g. {np.round(wb, 3).tolist()} (prepare() default wobble applies)" if wb else "")
            if grouping == "shape":
                mixed = []
                for gi, g in enumerate(groups):
                    cs = [c for c in split_spurs([as_np(c) for c in g["curves"] if len(c)], on_tol)[0] if len(c)]
                    if not cs or g["name"] == "lines":
                        continue
                    ids = group_shape_ids(cs, polys, on_tol)
                    if len(ids) != 1:
                        mixed.append(f"group {gi} '{g['name']}' -> shapes {sorted(ids)}")
                    elif mode == "both" and classify_curves(cs, polys, spacing, on_tol)[1] > 0:
                        filled |= ids
                res.check(name, tag, f"{lab} each group within one shape", not mixed, len(mixed), 0,
                          "; ".join(mixed[:3]))
    if ("fill", "shape") in got and ("both", "shape") in got:
        want = len(filled) + (1 if n_open else 0)
        g = got[("fill", "shape")][0]["groups"]
        res.check(name, "prep", "[fill/shape] group count == shapes with fill(+open)", len(g) == want, len(g), want,
                  f"{len(polys)} shapes; shapes with no fill loops get no group")
    if ("both", "shape") in got and ("both", "join") in got:
        A = [c for c in got[("both", "shape")][1] if len(c) >= 2]
        B = [c for c in got[("both", "join")][1] if len(c) >= 2]
        a = sum(poly_length(c) for c in A)
        bb = sum(poly_length(c) for c in B)
        res.check(name, "prep", "[both] join vs shape-grouped length (spurs excluded)",
                  abs(a - bb) <= 0.005 * max(a, 1e-9), (bb - a) / max(a, 1e-9), 0.005)
        gt = 1e-3 * parsed["diag"]
        d1, w1, d2, w2 = hausdorff(A, None, B, gt)
        res.check(name, "prep", "[both] join vs shape-grouped geometry (max dist, spurs excluded)",
                  max(d1, d2) <= gt, max(d1, d2), gt,
                  f"worst {np.round(w1 if d1 >= d2 else w2, 3).tolist() if (w1 or w2) else None}")

    # ------------------------------------------------ exportGroups end-to-end (bike only)
    if "bike_outline" in name and not soft_rec and not args.no_e2e:
        test_fill_e2e(page, name, res, polys, spacing)
    return summary


def test_fill_e2e(page, name, res, polys, spacing):
    tag = "fill-e2e"
    try:
        p = page.evaluate(JS_PREPARE, dict(mode="both", grouping="shape", spacing=spacing, seed=FILL_SEED))
    except Exception as e:
        res.add(name, tag, "prepare() threw", "FAIL", detail=str(e).splitlines()[0][:300])
        return
    opts = dict(tl=SQ_TL, br=SQ_BR, rotationDeg=0, toleranceM=50, maxPoints=6000, maxKm=0, name="Bike")
    e = check_export(page, name, res, opts, tag=tag, js=JS_EXPORT_GROUPS, fn="exportGroups")
    if not e or not e["routes"]:
        return
    routes = e["routes"]
    s = sum(r["km"] for r in routes)
    res.check(name, tag, "sum(route.km) == totalKm (strict)", abs(s - e["totalKm"]) <= 1e-6 * max(s, 1e-9),
              s, e["totalKm"])
    nm = [r["name"] for r in routes]
    res.check(name, tag, "route names unique", len(set(nm)) == len(nm), len(set(nm)), len(nm), "; ".join(nm[:6]))
    idx = [r["index"] for r in routes]
    res.check(name, tag, "route.index == 1..n", idx == list(range(1, len(routes) + 1)), idx[:8], "1..n", soft=True)
    grp = [r["group"] for r in routes]
    res.check(name, tag, "every route has a group", all(g is not None for g in grp), grp[:10], "int", soft=True)
    ng = len(p["groups"])
    res.check(name, tag, "routes >= groups and every group exported",
              set(g for g in grp if g is not None) == set(range(ng)), sorted(set(grp)), f"0..{ng - 1}")
    # placement: map the prepared geometry with the bbox of ALL groups, then compare
    curves = [as_np(c) for g in p["groups"] for c in g["curves"] if len(c) >= 2]
    lines_by_route = [parse_kml(r["kml"]) for r in routes]
    all_lines = [l for ls in lines_by_route for l in ls]
    exp = map_drawing(curves, opts["tl"], opts["br"], 0)
    d_eg, d_ge, diag_m = geo_hausdorff(exp, all_lines)
    thr = max(1.5 * opts["toleranceM"], 1e-3 * diag_m)
    res.check(name, tag, "geo: prepared geometry(mapped) -> routes (m)", d_eg <= thr, d_eg, thr)
    res.check(name, tag, "geo: route vertices -> prepared geometry (m)", d_ge <= thr, d_ge, thr)
    # no route mixes shapes: every KML vertex must lie in ONE mapped shape (buffered by 2 x tolerance)
    rings = [[np.asarray(p_.exterior.coords)] + [np.asarray(r.coords) for r in p_.interiors] for p_ in polys]
    from shapely.geometry import Polygon
    mapped = []
    for rr in rings:
        mr = map_with_box(curves, rr, opts["tl"], opts["br"])
        mapped.append(Polygon(mr[0], mr[1:]))
    k = 1.0 / math.cos(math.radians(max(opts["tl"][0], opts["br"][0])))
    buf = [mp.buffer(2 * opts["toleranceM"] * k) for mp in mapped]
    mixed, stray = [], []
    for ri, ls in enumerate(lines_by_route):
        P = merc(np.vstack(ls)[:, 0], np.vstack(ls)[:, 1])
        hits = np.stack([shapely.contains_xy(bp, P[:, 0], P[:, 1]) for bp in buf])
        best = int(np.argmax(hits.sum(1)))
        nout = int((~hits[best]).sum())
        if nout:
            other = [j for j in range(len(buf)) if j != best and hits[j].any()]
            (mixed if other else stray).append(f"route {ri} '{routes[ri]['name']}': {nout}/{len(P)} vertices outside "
                                               f"shape {best}" + (f", in shapes {other}" if other else ""))
    res.check(name, tag, "no route mixes shapes (KML vertices in one mapped shape)", not mixed, len(mixed), 0,
              "; ".join(mixed[:3]))
    res.check(name, tag, "route vertices inside a mapped shape", not stray, len(stray), 0, "; ".join(stray[:3]))


def test_ui_fill(browser, url, res, ready_timeout):
    """Smoke: load bike_outline.svg through #file, pick Squiggle fill + one route per shape."""
    name = "(ui) bike_outline.svg"
    errors = []
    ctx = browser.new_context()
    page = ctx.new_page()
    page.on("pageerror", lambda e: errors.append(str(e)))
    cons = []
    page.on("console", lambda m: cons.append(m.text) if m.type == "error" else None)
    try:
        page.goto(url)
        page.wait_for_function("window.RWGPSConv && window.RWGPSConv.ready === true", timeout=ready_timeout * 1000)
        t0 = time.time()
        page.set_input_files("#file", os.path.join(FIX, "bike_outline.svg"))
        page.wait_for_function("document.querySelectorAll('#routes li').length >= 1", timeout=60_000)
        res.add(name, "ui", "outline routes after load", "INFO", page.locator("#routes li").count(),
                detail=f"{time.time() - t0:.1f} s")
        # mode control: radio group name=mode, or a <select id=mode>
        if page.locator("#mode").count() and page.evaluate("document.getElementById('mode').tagName") == "SELECT":
            page.select_option("#mode", "fill")
        else:
            page.check("input[name=mode][value=fill]")
        page.wait_for_function("!document.getElementById('busy') || document.getElementById('busy').hidden",
                               timeout=60_000)
        page.select_option("#grouping", "shape")
        t1 = time.time()
        page.wait_for_function("""() => { const b = document.getElementById('busy');
              return (!b || b.hidden) && document.querySelectorAll('#routes li').length >= 2; }""", timeout=60_000)
        page.wait_for_timeout(800)
        n = page.locator("#routes li").count()
        names = page.eval_on_selector_all("#routes li .rn", "els => els.map(e => e.textContent)")
        note = page.evaluate("(document.getElementById('fillNote') || {}).textContent || ''")
        summ = page.evaluate("(document.getElementById('summary') || {}).textContent || ''")
        res.check(name, "ui", "routes listed (fill, one route per shape)", n >= 2, n, ">= 2",
                  f"{time.time() - t1:.1f} s; {names[:4]}; summary '{summ[:80]}'; note '{note[:120]}'")
        page.click("#shuffle")
        page.wait_for_timeout(300)
        page.wait_for_function("""() => { const b = document.getElementById('busy');
              return (!b || b.hidden) && document.querySelectorAll('#routes li').length >= 2; }""", timeout=60_000)
        res.check(name, "ui", "routes still listed after Shuffle", page.locator("#routes li").count() >= 2,
                  page.locator("#routes li").count(), ">= 2")
    except Exception as e:
        res.add(name, "ui", "UI flow failed", "FAIL", detail=str(e).splitlines()[0][:300])
    res.check(name, "ui", "no uncaught page errors", not errors, len(errors), 0, "; ".join(errors[:3])[:300])
    if cons:
        res.add(name, "ui", "console errors", "WARN", len(cons), detail="; ".join(cons[:3])[:300])
    ctx.close()


def print_fill_summary(rows):
    if not rows:
        return
    print("\n== Squiggle fill summary (per fixture / run; clearance and coverage in units of spacing)")
    hdr = f"{'fixture':26s} {'run':10s} {'spacing':>9s} {'loops':>6s} {'verts':>7s} {'min clr':>8s} {'seg clr':>8s} " \
          f"{'cov max':>8s} {'cov p95':>8s} {'cov dp':>8s} {'fill ms':>8s}"
    print(hdr)
    print("-" * len(hdr))
    f = lambda v, s="{:.3f}": "-" if v is None else s.format(v)
    for r in rows:
        print(f"{r['fixture']:26s} {r['run']:10s} {r['spacing']:9.4f} {r['loops']:6d} {r['verts']:7d} "
              f"{f(r['min_clear']):>8s} {f(r['min_seg_clear']):>8s} {f(r['cov_max']):>8s} {f(r['cov_p95']):>8s} "
              f"{f(r['cov_deep']):>8s} {r['ms']:8.1f}")
    print("  min clr/seg clr: min vertex / segment distance to the boundary; cov max/p95: distance from samples in the "
          "shape eroded by 1 x spacing to the nearest loop; cov dp: same for samples >= 2.5 x spacing deep")


# ======================================================================= main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.dirname(HERE))
    ap.add_argument("--page", default="index.html")
    ap.add_argument("--only", default="")
    ap.add_argument("--no-e2e", action="store_true")
    ap.add_argument("--fill", action="store_true", help="only the squiggle-fill section")
    ap.add_argument("--no-fill", action="store_true", help="skip the squiggle-fill section")
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
            if args.fill:
                refs = [r for r in refs if "fill" in r]
            fill_rows = []
            parsed_by_name = {}
            for ref in refs:
                name = ref["fixture"]
                print(f"\n== {name}  ({ref['kind']}{', ADVISORY' if ref.get('advisory') else ''}) {ref['notes'][:110]}")
                parsed = test_parse(page, ref, res)
                if parsed is None:
                    continue
                parsed_by_name[name] = parsed
                o = test_order(page, ref, parsed, res)
                if "fill" in ref and not args.no_fill:
                    print(f"  -- squiggle fill ({ref['fill']['rule']}{', record only' if ref['fill'].get('record_only') else ''})")
                    rows = test_fill(page, ref, parsed, res, args)
                    fill_rows += rows or []
                    page.evaluate("() => { window.__built = null; window.__prepared = null; }")
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
            if not args.no_fill and (not only or any(o in "bike_outline.svg" for o in only)) \
                    and os.path.exists(os.path.join(FIX, "bike_outline.svg")):
                print("\n== UI smoke: squiggle fill")
                test_ui_fill(browser, url, res, args.ready_timeout)
            print_fill_summary(fill_rows)
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

    hdr = f"{'fixture':26s} {'result':6s} {'ref->out':>9s} {'out->ref':>9s} {'tol':>8s} {'len err':>9s} " \
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
        print(f"{fx:26s} {status:6s} {num(fx, 'parse', 'maxdist ref->out'):>9s} {num(fx, 'parse', 'maxdist out->ref'):>9s} "
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
