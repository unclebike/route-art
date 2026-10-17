#!/usr/bin/env python3
"""Generate DXF/SVG test fixtures plus independent reference geometry.

Run:  python3 -I tests/make_fixtures.py        (from /home/claude/route-art)

Every fixture <name>.<ext> gets a sibling <name>.<ext>.ref.json (extension kept so that
squiggle.dxf and squiggle.svg do not collide):
{
  "fixture": "arcs.dxf", "kind": "dxf"|"svg",
  "polylines": [[[x,y],...],...],   # dense reference sampling, y-up, drawing units
  "points": [[x,y],...],            # degenerate (single-point) entities; optional in output
  "length": float, "bbox": [minx,miny,maxx,maxy], "diag": float,
  "tol_rel": 1e-3,                  # Hausdorff threshold = tol_rel * diag
  "len_tol": 0.005,                 # relative total length tolerance
  "advisory": false,                # true => failures reported as WARN, not FAIL
  "expect": {...},                  # extra expectations (units, path_count, warnings...)
  "notes": "..."
}
DXF references are flattened by ezdxf (recursing into INSERTs); SVG references by
svgelements (transforms applied, root viewBox/width/height stripped, y flipped).
"""
import json
import math
import os
import re
import shutil
import sys

import ezdxf
from ezdxf import disassemble
from ezdxf.math import Vec3
import ezdxf.path

HERE = os.path.dirname(os.path.abspath(__file__))
FIX = os.path.join(HERE, "fixtures")
# Optional real-world stress file (not committed): SQUIGGLE_DXF=/path/to/squiggle.dxf
SQUIGGLE_SRC = os.environ.get("SQUIGGLE_DXF", "")

os.makedirs(FIX, exist_ok=True)


# --------------------------------------------------------------------------- helpers
def poly_len(pl):
    return sum(math.dist(pl[i], pl[i + 1]) for i in range(len(pl) - 1))


def bbox_of(polys, pts=()):
    xs = [p[0] for pl in polys for p in pl] + [p[0] for p in pts]
    ys = [p[1] for pl in polys for p in pl] + [p[1] for p in pts]
    return [min(xs), min(ys), max(xs), max(ys)]


def rnd(pl, nd=6):
    return [[round(float(x), nd), round(float(y), nd)] for x, y in pl]


def dedupe(pl, eps=1e-12):
    out = []
    for p in pl:
        if not out or math.dist(out[-1], p) > eps:
            out.append(p)
    return out


def write_ref(name, kind, polys, points=(), expect=None, tol_rel=1e-3, len_tol=0.005,
              advisory=False, notes="", nd=6):
    polys = [rnd(dedupe(pl), nd) for pl in polys]
    polys = [pl for pl in polys if len(pl) >= 2]
    points = rnd(points, nd)
    bb = bbox_of(polys, points)
    ref = {
        "fixture": name,
        "kind": kind,
        "polylines": polys,
        "points": points,
        "length": sum(poly_len(pl) for pl in polys),
        "bbox": bb,
        "diag": math.hypot(bb[2] - bb[0], bb[3] - bb[1]),
        "tol_rel": tol_rel,
        "len_tol": len_tol,
        "advisory": advisory,
        "expect": expect or {},
        "notes": notes,
    }
    with open(os.path.join(FIX, name + ".ref.json"), "w") as f:
        json.dump(ref, f, separators=(",", ":"))
    print(f"  {name:22s} polylines={len(polys):5d} pts={sum(len(p) for p in polys):7d} "
          f"len={ref['length']:.4f} bbox={[round(v, 3) for v in bb]}")
    return ref


# --------------------------------------------------------------------------- DXF reference
def knots_clamped(e):
    k = list(e.knots)
    p = e.dxf.degree
    if not k:
        return True
    return all(abs(v - k[0]) < 1e-12 for v in k[:p + 1]) and all(abs(v - k[-1]) < 1e-12 for v in k[-p - 1:])


def nurbs_sample(e, per_span=400):
    """Independent De Boor NURBS evaluation over the valid domain [u_p, u_(m-p)].
    Needed because ezdxf evaluates unclamped (periodic) knot vectors over [u_0, u_m]."""
    p = e.dxf.degree
    U = list(e.knots)
    P = [(v[0], v[1]) for v in e.control_points]
    W = list(e.weights) if len(e.weights) else [1.0] * len(P)
    n = len(P) - 1
    Ph = [(x * w, y * w, w) for (x, y), w in zip(P, W)]

    def ev(u):
        k = p
        while k < n and not (U[k] <= u < U[k + 1]):
            k += 1
        d = [list(Ph[j + k - p]) for j in range(p + 1)]
        for r in range(1, p + 1):
            for j in range(p, r - 1, -1):
                den = U[j + 1 + k - r] - U[j + k - p]
                a = 0.0 if den == 0 else (u - U[j + k - p]) / den
                d[j] = [(1 - a) * d[j - 1][i] + a * d[j][i] for i in range(3)]
        x, y, w = d[p]
        return (x / w, y / w)

    out = []
    for i in range(p, n + 1):
        a, b = U[i], U[i + 1]
        if b - a <= 0:
            continue
        for s in range(per_span):
            out.append(ev(a + (b - a) * s / per_span))
    out.append(ev(U[n + 1] - 1e-12))
    return out


def flatten_entity(e, dist):
    """Return list of WCS xy polylines for one primitive entity (no INSERTs)."""
    t = e.dxftype()
    if t == "LINE":
        return [[tuple(e.dxf.start)[:2], tuple(e.dxf.end)[:2]]]
    if t in ("ARC", "CIRCLE", "ELLIPSE"):
        # flattening() returns WCS points (OCS handled by ezdxf)
        if t == "ELLIPSE":
            pts = list(e.flattening(dist))
        else:
            pts = list(e.flattening(dist))
        return [[(p.x, p.y) for p in pts]]
    if t == "SPLINE":
        if len(e.control_points) and not knots_clamped(e):
            return [nurbs_sample(e)]
        pts = list(e.flattening(dist, segments=8))
        return [[(p.x, p.y) for p in pts]]
    if t in ("LWPOLYLINE", "POLYLINE"):
        # exact: decompose into LINE/ARC sub-entities (bulges -> true arcs), flatten each in WCS
        subs = list(e.virtual_entities())
        if not subs:  # single vertex (degenerate)
            if t == "LWPOLYLINE":
                ocs = e.ocs()
                return [[tuple(ocs.to_wcs(Vec3(x, y, e.dxf.elevation)))[:2] for x, y in e.get_points("xy")]]
            return [[tuple(v.dxf.location)[:2] for v in e.vertices]]
        out = []
        for s in subs:
            for pl in flatten_entity(s, dist):
                if out and math.dist(out[-1], pl[0]) < 1e-9:
                    out.extend(pl[1:])
                elif out and math.dist(out[-1], pl[-1]) < 1e-9:
                    out.extend(pl[::-1][1:])
                else:
                    out.extend(pl)
        return [out]
    print("    (ignoring entity in reference:", t, ")")
    return []


def dxf_reference(doc, rel=1e-5):
    msp = doc.modelspace()
    prims = list(disassemble.recursive_decompose(msp))
    prims = [e for e in prims if e.dxftype() != "INSERT"]
    # rough extents for choosing flatten distance
    rough = []
    for e in prims:
        rough += flatten_entity(e, 1e9) if e.dxftype() != "SPLINE" else [
            [(v[0], v[1]) for v in e.control_points]] if len(e.control_points) else [
            [(v[0], v[1]) for v in e.fit_points]]
    bb = bbox_of([r for r in rough if r])
    diag = math.hypot(bb[2] - bb[0], bb[3] - bb[1])
    dist = max(diag * rel, 1e-9)
    polys, points = [], []
    for e in prims:
        for pl in flatten_entity(e, dist):
            pl = dedupe(pl)
            if len(pl) >= 2:
                polys.append(pl)
            elif len(pl) == 1:
                points.append(pl[0])
    return polys, points


def new_doc(units=0, ver="R2010"):
    doc = ezdxf.new(ver)
    doc.header["$INSUNITS"] = units
    doc.units = units
    return doc


def save_dxf(doc, name, **refkw):
    path = os.path.join(FIX, name)
    doc.saveas(path)
    doc2 = ezdxf.readfile(path)          # reference from what was actually written
    polys, points = dxf_reference(doc2)
    return write_ref(name, "dxf", polys, points, **refkw)


def make_dxf():
    print("DXF fixtures")
    # a) lines
    doc = new_doc()
    msp = doc.modelspace()
    zz = [(0, 0), (10, 10), (20, 0), (30, 10), (40, 0), (50, 12)]
    for a, b in zip(zz, zz[1:]):
        msp.add_line(a, b, dxfattribs={"layer": "ZIGZAG"})
    sq = [(60, 0), (75, 0), (75, 15), (60, 15)]
    for i in range(4):
        msp.add_line(sq[i], sq[(i + 1) % 4], dxfattribs={"layer": "SQUARE"})
    save_dxf(doc, "lines.dxf", expect={"units": None, "layers": ["SQUARE", "ZIGZAG"]},
             notes="5 LINEs forming a connected zig-zag + 4 LINEs forming a square")

    # b) arcs
    doc = new_doc()
    msp = doc.modelspace()
    msp.add_arc((0, 0), 10, 300, 60)            # crosses 0 deg (CCW from 300 to 60)
    msp.add_arc((30, 0), 6, 45, 270)
    msp.add_arc((30, 30), 4, 170, 10)           # also wraps through 0
    msp.add_circle((60, 0), 8)
    e = msp.add_ellipse((100, 0), major_axis=(20, 10), ratio=0.4, start_param=0.5, end_param=4.0)
    msp.add_ellipse((100, 40), major_axis=(-6, 15), ratio=0.6, start_param=5.5, end_param=1.2)  # wraps 2pi
    msp.add_ellipse((150, 0), major_axis=(0, 12), ratio=0.5)  # full ellipse, vertical major axis
    save_dxf(doc, "arcs.dxf", expect={"units": None},
             notes="ARC 300->60 (wraps 0), ARC 45->270, ARC 170->10, CIRCLE, ELLIPSE rotated partial, "
                   "ELLIPSE param 5.5->1.2 (wraps 2pi), full ELLIPSE")

    # c) lwpolyline / polyline bulges
    doc = new_doc()
    msp = doc.modelspace()
    msp.add_lwpolyline([(0, 0, 0.5), (10, 0, -0.7), (20, 0, 0), (20, 10, 1.0), (30, 10, 0)],
                       format="xyb")
    msp.add_lwpolyline([(40, 0, 0), (50, 0, 0.3), (50, 10, 0), (40, 10, -0.6)],
                       format="xyb", close=True)  # bulge on closing segment (last vertex)
    pl = msp.add_polyline2d([(0, 30), (10, 30), (10, 40), (0, 45)])
    for v, b in zip(pl.vertices, [0.4, 0.0, -1.0, 0.25]):
        v.dxf.bulge = b
    pl.close(True)                               # closing segment 0,45 -> 0,30 has bulge 0.25
    save_dxf(doc, "lwpoly_bulge.dxf", expect={"units": None},
             notes="open LWPOLYLINE +/- bulges incl. semicircle (b=1), closed LWPOLYLINE with bulge "
                   "on closing segment, closed 2D POLYLINE with VERTEX bulges incl. b=-1 and closing bulge")

    # d) splines
    doc = new_doc()
    msp = doc.modelspace()
    cps = [(0, 0), (10, 20), (20, -10), (30, 15), (40, 0), (50, 25)]
    msp.add_open_spline(cps, degree=3)                       # clamped uniform
    s = msp.add_spline(dxfattribs={"degree": 3})             # clamped NON-uniform knots
    s.control_points = [(0, 40), (5, 55), (15, 60), (25, 45), (35, 60), (50, 50)]
    s.knots = [0, 0, 0, 0, 0.1, 0.6, 1, 1, 1, 1]
    s.dxf.flags = 8
    closed_cps = [(70, 0), (90, 5), (95, 25), (80, 35), (65, 20)]
    wrapped = closed_cps + closed_cps[:3]                    # periodic closed: wrapped CPs,
    s = msp.add_spline(dxfattribs={"degree": 3})             # unclamped uniform knots
    s.control_points = wrapped
    s.knots = [float(i) for i in range(len(wrapped) + 4)]
    s.dxf.flags = 1 | 2 | 8                                  # closed | periodic | planar
    r = 1 / math.sqrt(2)
    msp.add_rational_spline([(110, 0), (110, 20), (130, 20)], [1, r, 1], degree=2)  # exact quarter circle
    msp.add_rational_spline([(140, 0), (150, 30), (165, -10), (180, 20)], [1, 3, 0.5, 1], degree=3)
    msp.add_open_spline([(0, -30), (20, -10), (40, -30), (60, -10), (80, -30)], degree=2)  # quadratic
    save_dxf(doc, "splines.dxf", expect={"units": None},
             notes="cubic clamped uniform, cubic clamped non-uniform knots, closed periodic cubic (5 CPs wrapped to 8, unclamped uniform knots 0..11, valid domain [3,8]), "
                   "rational quadratic (exact quarter circle r=20 centre 130,0), rational cubic, quadratic")

    # d2) fit-points-only spline (ezdxf CAN write it; geometry is implementation-defined)
    doc = new_doc()
    msp = doc.modelspace()
    sp = msp.add_spline(fit_points=[(0, 0), (10, 15), (25, 5), (40, 20), (55, 0)])
    assert len(sp.control_points) == 0
    save_dxf(doc, "splines_fit.dxf", expect={"units": None}, tol_rel=1e-2, len_tol=0.02, advisory=True,
             notes="SPLINE with fit points only (no control points). Reference = ezdxf's AutoCAD-like "
                   "interpolation; interpolation is not uniquely defined, so this is ADVISORY "
                   "(tol 1e-2*diag, length 2%)")

    # e) blocks
    doc = new_doc()
    leaf = doc.blocks.new("LEAF", base_point=(5, 5))
    leaf.add_line((5, 5), (15, 5))
    leaf.add_arc((10, 10), 3, 0, 180)
    leaf.add_lwpolyline([(15, 5), (20, 12), (12, 15)], close=True)
    parent = doc.blocks.new("PARENT", base_point=(2, 1))
    parent.add_line((2, 1), (12, 1))
    parent.add_circle((6, 8), 2)
    parent.add_blockref("LEAF", (20, 0), dxfattribs={"rotation": 30, "xscale": 0.5, "yscale": 0.5})
    msp = doc.modelspace()
    msp.add_blockref("LEAF", (0, 0))
    msp.add_blockref("LEAF", (40, 0), dxfattribs={"xscale": -1, "yscale": 1})                  # mirror
    msp.add_blockref("LEAF", (80, 20), dxfattribs={"xscale": 2, "yscale": 1.5, "rotation": 45})  # non-uniform
    msp.add_blockref("PARENT", (0, 60), dxfattribs={"xscale": 1.2, "yscale": 1.2, "rotation": -20})
    save_dxf(doc, "blocks.dxf", expect={"units": None},
             notes="LEAF (base 5,5) inserted 3x: identity, xscale=-1 (mirror), xscale 2/yscale 1.5 rot 45 "
                   "(arc becomes ellipse); PARENT (base 2,1) rot -20 scale 1.2 containing nested LEAF")

    # f) OCS
    doc = new_doc()
    msp = doc.modelspace()
    msp.add_arc((10, 5), 4, 0, 90, dxfattribs={"extrusion": (0, 0, -1)})
    msp.add_circle((30, 5), 3, dxfattribs={"extrusion": (0, 0, -1)})
    msp.add_lwpolyline([(0, 20, 0.5), (10, 20, 0), (10, 30, -0.3), (0, 30, 0)], format="xyb",
                       dxfattribs={"extrusion": (0, 0, -1)})
    msp.add_line((0, 0), (5, 5))  # a normal WCS reference entity
    save_dxf(doc, "ocs.dxf", expect={"units": None},
             notes="ARC, CIRCLE, LWPOLYLINE with extrusion (0,0,-1): OCS x is mirrored (WCS x = -OCS x), "
                   "arc direction reverses")

    # g) units mm
    doc = new_doc(units=4)
    msp = doc.modelspace()
    msp.add_lwpolyline([(0, 0), (100, 0), (100, 50), (0, 50)], close=True)
    msp.add_circle((50, 25), 20)
    save_dxf(doc, "units_mm.dxf", expect={"units": "mm"}, notes="$INSUNITS=4 -> units 'mm'")

    # h) R12 (old structure: no subclass markers, POLYLINE not LWPOLYLINE)
    doc = ezdxf.new("R12")
    doc.header["$INSUNITS"] = 0
    blk = doc.blocks.new("R12BLK", base_point=(0, 0))
    blk.add_line((0, 0), (5, 0))
    blk.add_arc((0, 0), 5, 0, 120)
    msp = doc.modelspace()
    msp.add_line((0, 0), (20, 10))
    msp.add_circle((30, 0), 5)
    msp.add_arc((50, 0), 6, 200, 20)
    p = msp.add_polyline2d([(0, 20), (10, 20), (20, 30)])
    for v, b in zip(p.vertices, [0.5, -0.5, 0]):
        v.dxf.bulge = b
    msp.add_blockref("R12BLK", (60, 20), dxfattribs={"rotation": 90})
    save_dxf(doc, "r12.dxf", expect={"units": None},
             notes="AutoCAD R12 DXF (AC1009): LINE, CIRCLE, ARC, POLYLINE w/ bulges, INSERT rotated 90")

    # squiggle (real world, optional)
    if not SQUIGGLE_SRC or not os.path.exists(SQUIGGLE_SRC):
        print("skipping squiggle fixtures (set SQUIGGLE_DXF to include them)")
        return None
    dst = os.path.join(FIX, "squiggle.dxf")
    shutil.copyfile(SQUIGGLE_SRC, dst)
    doc = ezdxf.readfile(dst)
    polys, points = dxf_reference(doc, rel=1e-5)
    ref = write_ref("squiggle.dxf", "dxf", polys, points, nd=4,
                    expect={"units": None, "jump_sum_max": 4000, "e2e": True},
                    notes="real-world file: 1 INSERT -> 8 -> 11 nested blocks; 649 SPLINEs + 88 single-point "
                          "LWPOLYLINEs (stored in 'points', optional in output)")
    return ref


# --------------------------------------------------------------------------- SVG reference
def svg_reference(svg_text, samples_rel=1e-4):
    """Independent SVG flattening with svgelements; root viewBox/size stripped => root user units."""
    import io
    from svgelements import SVG, Path, Shape, Move, Line, Close, Arc, CubicBezier, QuadraticBezier

    stripped = re.sub(r'(<svg\b[^>]*?)\s(viewBox|width|height)="[^"]*"', r"\1", svg_text, count=1)
    while re.search(r'<svg\b[^>]*\s(viewBox|width|height)="', stripped):
        stripped = re.sub(r'(<svg\b[^>]*?)\s(viewBox|width|height)="[^"]*"', r"\1", stripped, count=1)
    svg = SVG.parse(io.StringIO(stripped), reify=True)
    shapes = [e for e in svg.elements() if isinstance(e, Shape)]
    # rough bbox to choose sampling density
    bbs = [s.bbox() for s in shapes if s.bbox() is not None]
    diag = math.hypot(max(b[2] for b in bbs) - min(b[0] for b in bbs),
                      max(b[3] for b in bbs) - min(b[1] for b in bbs))
    step = diag * samples_rel
    polys = []
    for s in shapes:
        p = Path(s)
        p.reify()
        for sub in p.as_subpaths():
            cur = []
            for seg in sub:
                if isinstance(seg, Move):
                    if len(cur) >= 2:
                        polys.append(cur)
                    cur = [(seg.end.x, seg.end.y)]
                elif isinstance(seg, (Line, Close)):
                    if seg.start is not None and not cur:
                        cur = [(seg.start.x, seg.start.y)]
                    cur.append((seg.end.x, seg.end.y))
                else:
                    if not cur:
                        cur = [(seg.start.x, seg.start.y)]
                    L = seg.length(error=1e-9)
                    n = max(16, min(20000, int(math.ceil(L / step))))
                    for i in range(1, n + 1):
                        q = seg.point(i / n)
                        cur.append((q.x, q.y))
            if len(cur) >= 2:
                polys.append(cur)
    # flip y -> y-up
    return [[(x, -y) for x, y in pl] for pl in polys]


SVG_NS = 'xmlns="http://www.w3.org/2000/svg"'


def save_svg(name, text, ref_text=None, **refkw):
    with open(os.path.join(FIX, name), "w") as f:
        f.write(text)
    polys = svg_reference(ref_text if ref_text is not None else text)
    return write_ref(name, "svg", polys, **refkw)


def make_svg(squiggle_ref):
    print("SVG fixtures")
    shapes = f"""<?xml version="1.0" encoding="UTF-8"?>
<svg {SVG_NS} width="400" height="300" viewBox="0 0 400 300">
  <g fill="none" stroke="black">
    <rect x="10" y="10" width="80" height="50"/>
    <rect x="110" y="10" width="80" height="50" rx="10" ry="15"/>
    <rect x="210" y="10" width="80" height="50" rx="12"/>
    <circle cx="50" cy="150" r="30"/>
    <ellipse cx="150" cy="150" rx="40" ry="20"/>
    <line x1="210" y1="120" x2="290" y2="180"/>
    <polyline points="10,220 40,260 70,220 100,260"/>
    <polygon points="150,210 190,280 110,280"/>
  </g>
</svg>
"""
    save_svg("shapes.svg", shapes, expect={"path_count": 8},
             notes="rect, rect rx+ry, rect rx only (ry=rx), circle, ellipse, line, polyline, polygon")

    transforms = f"""<?xml version="1.0" encoding="UTF-8"?>
<svg {SVG_NS} width="500" height="400" viewBox="-50 -30 500 400">
  <g transform="translate(100,50)" fill="none" stroke="black">
    <g transform="rotate(30)">
      <rect x="0" y="0" width="40" height="20"/>
      <g transform="scale(2,0.5)">
        <circle cx="20" cy="40" r="10"/>
      </g>
    </g>
    <g transform="matrix(1,0.3,-0.2,1,150,0)">
      <path d="M0,0 L50,0 L50,50 Z"/>
    </g>
    <polyline transform="rotate(-15 20 20) skewX(10)" points="0,100 30,130 60,100"/>
  </g>
  <ellipse transform="translate(300 200) rotate(45)" cx="0" cy="0" rx="40" ry="15" fill="none" stroke="black"/>
  <line transform="translate(-40,-20) scale(1.5)" x1="0" y1="0" x2="20" y2="10" stroke="black"/>
</svg>
"""
    save_svg("transforms.svg", transforms, expect={"path_count": 6},
             notes="nested translate/rotate/scale(non-uniform on circle)/matrix(with skew), element-level "
                   "rotate(a cx cy) skewX, viewBox with negative min (ignored; root user units)")

    paths = f"""<?xml version="1.0" encoding="UTF-8"?>
<svg {SVG_NS} width="500" height="300" viewBox="0 0 500 300">
  <path fill="none" stroke="black" d="m 20,20 c 20,-20 40,20 60,0 s 40,20 60,0 q 20,-30 40,0 t 40,0 h 30 v 40 H 20 z
    m 0,80 a 30,20 0 1,1 60,0 a 30,20 0 0,0 60,0 l 20,20 10,0 z
    M 300,50 A 40 40 0 0 1 380 50 A 40 25 15 1 0 300 90 V 150 h -40 L 250,100 260,80 Z"/>
</svg>
"""
    save_svg("paths.svg", paths, expect={"path_count": 3},
             notes="one <path> with 3 subpaths: relative m/c/s/q/t/h/v/H/z; 'm' after 'z' is relative to the "
                   "subpath start (20,20) -> (20,100); arcs with large-arc/sweep flag combos, rotated arc, "
                   "implicit repeated lineto")

    compact = f"""<?xml version="1.0" encoding="UTF-8"?>
<svg {SVG_NS} viewBox="0 0 200 100">
  <path fill="none" stroke="black" d="M10-5l20.5.5-.5 10e0c5,5 10-5 15,0a10 10 0 1020 0M100,50h1e1v-1E1z"/>
</svg>
"""
    save_svg("paths_compact.svg", compact, expect={"path_count": 2},
             notes="compact number syntax: '10-5', '.5.5', '-.5', exponent '10e0'/'1E1', arc flags packed "
                   "'0 1020 0' (flags 1,0 then x=20). Optimisers (svgo) emit this.")

    ignored = f"""<?xml version="1.0" encoding="UTF-8"?>
<svg {SVG_NS} width="200" height="200" viewBox="0 0 200 200">
  <defs>
    <path id="p1" d="M0,0 L200,200"/>
    <linearGradient id="lg"><stop offset="0" stop-color="red"/></linearGradient>
  </defs>
  <clipPath id="c"><rect x="0" y="0" width="150" height="150"/></clipPath>
  <mask id="m"><circle cx="100" cy="100" r="80" fill="white"/></mask>
  <symbol id="s"><circle cx="10" cy="10" r="5"/></symbol>
  <marker id="mk" markerWidth="4" markerHeight="4"><path d="M0,0 L4,2 L0,4 z"/></marker>
  <pattern id="pt" width="10" height="10"><rect x="0" y="0" width="5" height="5"/></pattern>
  <text x="20" y="180">Hello route</text>
  <path d="M 10 10 L 90 40 L 30 80" fill="none" stroke="black" clip-path="url(#c)" marker-end="url(#mk)"/>
</svg>
"""
    visible_only = f"""<svg {SVG_NS}><path d="M 10 10 L 90 40 L 30 80"/></svg>"""
    save_svg("ignored.svg", ignored, ref_text=visible_only,
             expect={"path_count": 1, "warning_re": "(?i)text"},
             notes="defs/clipPath/mask/symbol/marker/pattern content must be ignored; <text> ignored with a "
                   "warning; exactly one path expected")

    if squiggle_ref is None:
        return
    # squiggle.svg : DXF squiggle flattened (coarser) -> SVG polylines (y-down) in a transformed group
    doc = ezdxf.readfile(os.path.join(FIX, "squiggle.dxf"))
    polys, _ = dxf_reference(doc, rel=2e-5)
    T = (100.0, 50.0, 0.5)   # translate(100,50) scale(0.5)
    ds, refpolys = [], []
    for pl in polys:
        pl = dedupe([(round(x, 3), round(-y, 3)) for x, y in pl])   # SVG y-down coords as written
        if len(pl) < 2:
            continue
        ds.append("M" + " L".join(f"{x:g},{y:g}" for x, y in pl))
        refpolys.append([(T[0] + T[2] * x, -(T[1] + T[2] * y)) for x, y in pl])
    body = "\n".join(f'    <path d="{d}"/>' for d in ds)
    text = (f'<?xml version="1.0" encoding="UTF-8"?>\n<svg {SVG_NS} width="1000" height="900" '
            f'viewBox="0 0 1000 900">\n  <g transform="translate(100,50) scale(0.5)" fill="none" '
            f'stroke="black" stroke-width="2">\n{body}\n  </g>\n</svg>\n')
    with open(os.path.join(FIX, "squiggle.svg"), "w") as f:
        f.write(text)
    write_ref("squiggle.svg", "svg", refpolys, nd=4,
              expect={"path_count": len(refpolys), "jump_sum_max": 2000, "e2e": True},
              notes="squiggle.dxf flattened, written as straight-segment paths (y negated) inside "
                    "translate(100,50) scale(0.5); reference computed analytically from the written "
                    "numbers. Jump threshold halved (scale 0.5).")


if __name__ == "__main__":
    print("ezdxf", ezdxf.__version__)
    sq = make_dxf()
    make_svg(sq)
    print("done ->", FIX)
