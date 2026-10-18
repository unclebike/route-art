// Closed outlines → filled shapes (outer ring + holes), using Clipper.
// DXF: all closed loops are combined even-odd, so letter counters become holes.
// SVG: each element uses its own fill-rule, then elements are unioned.

const closedEnough = (pts, eps) => pts.length >= 4 && Math.hypot(pts[0][0] - pts[pts.length - 1][0], pts[0][1] - pts[pts.length - 1][1]) <= eps;

export function buildShapes(paths) {
  const CL = window.ClipperLib;
  const all = paths.flatMap((p) => p.pts);
  if (!CL || !all.length) return { shapes: [], open: paths.slice() };
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const [x, y] of all) { if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y; }
  const diag = Math.hypot(x1 - x0, y1 - y0) || 1;
  const eps = diag * 1e-6;
  const s = 1e9 / diag;
  const toC = (pts) => pts.map(([x, y]) => ({ X: Math.round((x - x0) * s), Y: Math.round((y - y0) * s) }));
  const fromC = (path) => path.map((p) => [p.X / s + x0, p.Y / s + y0]);

  const closed = [], open = [];
  for (const p of paths) (p.closed || closedEnough(p.pts, eps) ? closed : open).push(p);
  if (!closed.length) return { shapes: [], open };

  // group by element: SVG elements carry `elem` + `rule`; DXF has neither (one even-odd group)
  const groups = new Map();
  for (const p of closed) {
    const key = p.elem ?? 'dxf';
    if (!groups.has(key)) groups.set(key, { rule: p.rule || 'evenodd', polys: [] });
    groups.get(key).polys.push(toC(p.pts));
  }
  let region = new CL.Paths();
  for (const g of groups.values()) {
    const c = new CL.Clipper();
    c.AddPaths(g.polys, CL.PolyType.ptSubject, true);
    const norm = new CL.Paths();
    c.Execute(CL.ClipType.ctUnion, norm, g.rule === 'evenodd' ? CL.PolyFillType.pftEvenOdd : CL.PolyFillType.pftNonZero, CL.PolyFillType.pftNonZero);
    const u = new CL.Clipper();
    u.AddPaths(region, CL.PolyType.ptSubject, true);
    u.AddPaths(norm, CL.PolyType.ptClip, true);
    const next = new CL.Paths();
    u.Execute(CL.ClipType.ctUnion, next, CL.PolyFillType.pftNonZero, CL.PolyFillType.pftNonZero);
    region = next;
  }
  // tree: each outer with its direct holes is one shape; islands inside holes are new shapes
  const c = new CL.Clipper();
  c.AddPaths(region, CL.PolyType.ptSubject, true);
  const tree = new CL.PolyTree();
  c.Execute(CL.ClipType.ctUnion, tree, CL.PolyFillType.pftNonZero, CL.PolyFillType.pftNonZero);
  const shapes = [];
  const minArea = (diag * 1e-4) ** 2;
  const visit = (node) => {
    for (const ch of node.Childs()) {
      if (!ch.IsHole()) {
        const outer = fromC(ch.Contour());
        if (Math.abs(area(outer)) < minArea) { for (const h of ch.Childs()) visit(h); continue; }
        const holes = ch.Childs().map((h) => fromC(h.Contour()));
        shapes.push({ rings: [outer, ...holes].map(closeRing) });
        for (const h of ch.Childs()) visit(h);
      }
    }
  };
  visit(tree);
  // stable reading order: top-to-bottom then left-to-right by bbox centre, bands of ~1/8 height
  const band = (y1 - y0) / 8 || 1;
  shapes.sort((a, b) => {
    const ca = centre(a.rings[0]), cb = centre(b.rings[0]);
    const ra = Math.floor((y1 - ca[1]) / band), rb = Math.floor((y1 - cb[1]) / band);
    return ra - rb || ca[0] - cb[0];
  });
  return { shapes, open };
}

function area(r) { let a = 0; for (let i = 0; i < r.length; i++) { const [x1, y1] = r[i], [x2, y2] = r[(i + 1) % r.length]; a += x1 * y2 - x2 * y1; } return a / 2; }
function centre(r) { let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity; for (const [x, y] of r) { if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y; } return [(x0 + x1) / 2, (y0 + y1) / 2]; }
function closeRing(r) { const a = r[0], b = r[r.length - 1]; return a[0] === b[0] && a[1] === b[1] ? r : r.concat([a.slice()]); }
