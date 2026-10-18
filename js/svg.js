// SVG → polylines in root user units, y flipped up.
// Geometry is read in JS (path data, shapes, arcs, Béziers flattened to a
// chord tolerance); the browser is only asked for transforms and computed
// styles. Clip paths cut the lines exactly; masks are applied as solid
// shapes (light fill shows, dark fill hides).

const SKIP = new Set(['defs', 'clipPath', 'mask', 'symbol', 'marker', 'pattern', 'title', 'desc', 'metadata', 'style', 'script', 'linearGradient', 'radialGradient', 'filter', 'foreignObject', 'image', 'switch']);
const SHAPES = new Set(['path', 'line', 'polyline', 'polygon', 'rect', 'circle', 'ellipse']);

// ---------- affine helpers ({a,b,c,d,e,f}) ----------
const M = (a, b, c, d, e, f) => ({ a, b, c, d, e, f });
const ID = M(1, 0, 0, 1, 0, 0);
const mul = (m, n) => M(m.a * n.a + m.c * n.b, m.b * n.a + m.d * n.b, m.a * n.c + m.c * n.d, m.b * n.c + m.d * n.d, m.a * n.e + m.c * n.f + m.e, m.b * n.e + m.d * n.f + m.f);
const ap = (m, x, y) => [m.a * x + m.c * y + m.e, m.b * x + m.d * y + m.f];
const plain = (m) => M(m.a, m.b, m.c, m.d, m.e, m.f);
const scaleOf = (m) => Math.sqrt(Math.abs(m.a * m.d - m.b * m.c)) || 1;
function ownTransform(el) {
  try { const t = el.transform && el.transform.baseVal.consolidate(); return t ? plain(t.matrix) : ID; } catch { return ID; }
}

// ---------- path data ----------
const NUM = /[+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?/y;
// Arc flags are single characters and may be glued to what follows
// ("a1 1 0 015 5"), so the tokenizer tracks which arc parameter it is on.
function tokenizePath(d) {
  const out = [];
  let i = 0, cmd = '', idx = 0;
  const n = d.length;
  while (i < n) {
    const ch = d[i];
    if (/[MmLlHhVvCcSsQqTtAaZz]/.test(ch)) { out.push(ch); cmd = ch; idx = 0; i++; continue; }
    if (/[\s,]/.test(ch)) { i++; continue; }
    if ((cmd === 'A' || cmd === 'a') && (idx % 7 === 3 || idx % 7 === 4)) {
      if (ch === '0' || ch === '1') { out.push(ch === '1' ? 1 : 0); idx++; i++; continue; }
    }
    NUM.lastIndex = i;
    const m = NUM.exec(d);
    if (!m) { i++; continue; }
    out.push(parseFloat(m[0]));
    idx++;
    i = NUM.lastIndex;
  }
  return out;
}

function flattenCubic(out, p0, p1, p2, p3, tol, depth = 0) {
  const dx = p3[0] - p0[0], dy = p3[1] - p0[1];
  const L = Math.hypot(dx, dy);
  const dist = (p) => (L > 1e-15 ? Math.abs(dx * (p[1] - p0[1]) - dy * (p[0] - p0[0])) / L : Math.hypot(p[0] - p0[0], p[1] - p0[1]));
  if (depth > 16 || Math.max(dist(p1), dist(p2)) * 0.75 <= tol) { out.push(p3); return; }
  const m = (a, b) => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
  const p01 = m(p0, p1), p12 = m(p1, p2), p23 = m(p2, p3), p012 = m(p01, p12), p123 = m(p12, p23), mid = m(p012, p123);
  flattenCubic(out, p0, p01, p012, mid, tol, depth + 1);
  flattenCubic(out, mid, p123, p23, p3, tol, depth + 1);
}

function flattenArc(out, p0, rx, ry, phiDeg, large, sweep, p1, tol) {
  // SVG implementation notes F.6.5 / F.6.6
  if (p0[0] === p1[0] && p0[1] === p1[1]) return;
  rx = Math.abs(rx); ry = Math.abs(ry);
  if (!rx || !ry) { out.push(p1); return; }
  const phi = phiDeg * Math.PI / 180, cph = Math.cos(phi), sph = Math.sin(phi);
  const dx2 = (p0[0] - p1[0]) / 2, dy2 = (p0[1] - p1[1]) / 2;
  const x1 = cph * dx2 + sph * dy2, y1 = -sph * dx2 + cph * dy2;
  const lam = (x1 * x1) / (rx * rx) + (y1 * y1) / (ry * ry);
  if (lam > 1) { const s = Math.sqrt(lam); rx *= s; ry *= s; }
  const num = rx * rx * ry * ry - rx * rx * y1 * y1 - ry * ry * x1 * x1;
  const den = rx * rx * y1 * y1 + ry * ry * x1 * x1;
  let co = Math.sqrt(Math.max(0, num / den));
  if (large === sweep) co = -co;
  const cxp = co * (rx * y1) / ry, cyp = co * -(ry * x1) / rx;
  const cx = cph * cxp - sph * cyp + (p0[0] + p1[0]) / 2, cy = sph * cxp + cph * cyp + (p0[1] + p1[1]) / 2;
  const ang = (ux, uy, vx, vy) => { const a = Math.atan2(ux * vy - uy * vx, ux * vx + uy * vy); return a; };
  const t1 = ang(1, 0, (x1 - cxp) / rx, (y1 - cyp) / ry);
  let dt = ang((x1 - cxp) / rx, (y1 - cyp) / ry, (-x1 - cxp) / rx, (-y1 - cyp) / ry);
  if (!sweep && dt > 0) dt -= 2 * Math.PI; else if (sweep && dt < 0) dt += 2 * Math.PI;
  const r = Math.max(rx, ry);
  const step = r > tol ? Math.min(Math.PI / 8, 2 * Math.acos(Math.max(-1, 1 - tol / r))) : Math.PI / 8;
  const n = Math.max(2, Math.ceil(Math.abs(dt) / step));
  for (let i = 1; i < n; i++) {
    const t = t1 + dt * i / n, x = rx * Math.cos(t), y = ry * Math.sin(t);
    out.push([cph * x - sph * y + cx, sph * x + cph * y + cy]);
  }
  out.push(p1);
}

// returns [{pts, closed}]
export function pathToPolys(d, tol) {
  const tok = tokenizePath(d || '');
  const subs = [];
  let cur = null, cmd = null, k = 0;
  let pos = [0, 0], start = [0, 0], lastC = null, lastQ = null;
  const begin = (p) => { cur = { pts: [p], closed: false }; subs.push(cur); start = p; };
  const need = (n) => { for (let j = 0; j < n; j++) if (typeof tok[k + j] !== 'number') return false; return true; };
  while (k < tok.length) {
    if (typeof tok[k] === 'string') { cmd = tok[k++]; if (cmd === 'Z' || cmd === 'z') { if (cur) { cur.pts.push(start); cur.closed = true; } pos = start; cur = null; lastC = lastQ = null; continue; } }
    else if (!cmd) { k++; continue; }
    const rel = cmd === cmd.toLowerCase();
    const C = cmd.toUpperCase();
    const ox = rel ? pos[0] : 0, oy = rel ? pos[1] : 0;
    if (!cur && C !== 'M') begin(pos.slice());
    let ctrlC = null, ctrlQ = null;
    if (C === 'M') {
      if (!need(2)) break;
      const p = [tok[k] + ox, tok[k + 1] + oy]; k += 2;
      begin(p); pos = p;
      cmd = rel ? 'l' : 'L';
    } else if (C === 'L') {
      if (!need(2)) break;
      pos = [tok[k] + ox, tok[k + 1] + oy]; k += 2; cur.pts.push(pos);
    } else if (C === 'H') {
      if (!need(1)) break;
      pos = [tok[k] + ox, pos[1]]; k += 1; cur.pts.push(pos);
    } else if (C === 'V') {
      if (!need(1)) break;
      pos = [pos[0], tok[k] + oy]; k += 1; cur.pts.push(pos);
    } else if (C === 'C' || C === 'S') {
      let p1, p2, p3;
      if (C === 'C') { if (!need(6)) break; p1 = [tok[k] + ox, tok[k + 1] + oy]; p2 = [tok[k + 2] + ox, tok[k + 3] + oy]; p3 = [tok[k + 4] + ox, tok[k + 5] + oy]; k += 6; }
      else { if (!need(4)) break; p1 = lastC ? [2 * pos[0] - lastC[0], 2 * pos[1] - lastC[1]] : pos; p2 = [tok[k] + ox, tok[k + 1] + oy]; p3 = [tok[k + 2] + ox, tok[k + 3] + oy]; k += 4; }
      flattenCubic(cur.pts, pos, p1, p2, p3, tol); ctrlC = p2; pos = p3;
    } else if (C === 'Q' || C === 'T') {
      let q1, p3;
      if (C === 'Q') { if (!need(4)) break; q1 = [tok[k] + ox, tok[k + 1] + oy]; p3 = [tok[k + 2] + ox, tok[k + 3] + oy]; k += 4; }
      else { if (!need(2)) break; q1 = lastQ ? [2 * pos[0] - lastQ[0], 2 * pos[1] - lastQ[1]] : pos; p3 = [tok[k] + ox, tok[k + 1] + oy]; k += 2; }
      const c1 = [pos[0] + 2 / 3 * (q1[0] - pos[0]), pos[1] + 2 / 3 * (q1[1] - pos[1])];
      const c2 = [p3[0] + 2 / 3 * (q1[0] - p3[0]), p3[1] + 2 / 3 * (q1[1] - p3[1])];
      flattenCubic(cur.pts, pos, c1, c2, p3, tol); ctrlQ = q1; pos = p3;
    } else if (C === 'A') {
      if (!need(7)) break;
      const rx = tok[k], ry = tok[k + 1], rot = tok[k + 2], large = tok[k + 3], sweep = tok[k + 4]; k += 5;
      const p = [tok[k] + ox, tok[k + 1] + oy]; k += 2;
      flattenArc(cur.pts, pos, rx, ry, rot, !!large, !!sweep, p, tol); pos = p;
    } else { k++; }
    lastC = ctrlC; lastQ = ctrlQ;
  }
  return subs.filter((s) => s.pts.length >= 2);
}

function ellipsePoly(cx, cy, rx, ry, tol) {
  const r = Math.max(rx, ry);
  const step = r > tol ? Math.min(Math.PI / 8, 2 * Math.acos(Math.max(-1, 1 - tol / r))) : Math.PI / 8;
  const n = Math.max(8, Math.ceil(2 * Math.PI / step));
  const pts = [];
  for (let i = 0; i <= n; i++) { const t = 2 * Math.PI * i / n; pts.push([cx + rx * Math.cos(t), cy + ry * Math.sin(t)]); }
  return pts;
}

const len = (el, name) => { try { return el[name].baseVal.value; } catch { return parseFloat(el.getAttribute(name)) || 0; } };

// local (untransformed) outlines of one shape element
export function shapeToPolys(el, tol) {
  const tag = el.localName;
  if (tag === 'path') return pathToPolys(el.getAttribute('d'), tol);
  if (tag === 'line') return [{ pts: [[len(el, 'x1'), len(el, 'y1')], [len(el, 'x2'), len(el, 'y2')]], closed: false }];
  if (tag === 'polyline' || tag === 'polygon') {
    const v = (el.getAttribute('points') || '').trim().split(/[\s,]+/).map(parseFloat).filter((x) => !Number.isNaN(x));
    const pts = [];
    for (let i = 0; i + 1 < v.length; i += 2) pts.push([v[i], v[i + 1]]);
    if (tag === 'polygon' && pts.length > 2) pts.push(pts[0].slice());
    return pts.length >= 2 ? [{ pts, closed: tag === 'polygon' }] : [];
  }
  if (tag === 'circle') { const r = len(el, 'r'); return r > 0 ? [{ pts: ellipsePoly(len(el, 'cx'), len(el, 'cy'), r, r, tol), closed: true }] : []; }
  if (tag === 'ellipse') {
    let rx = len(el, 'rx'), ry = len(el, 'ry');
    if (!el.hasAttribute('rx')) rx = ry; if (!el.hasAttribute('ry')) ry = rx;
    return rx > 0 && ry > 0 ? [{ pts: ellipsePoly(len(el, 'cx'), len(el, 'cy'), rx, ry, tol), closed: true }] : [];
  }
  if (tag === 'rect') {
    const x = len(el, 'x'), y = len(el, 'y'), w = len(el, 'width'), h = len(el, 'height');
    if (!(w > 0 && h > 0)) return [];
    let rx = el.hasAttribute('rx') ? len(el, 'rx') : null, ry = el.hasAttribute('ry') ? len(el, 'ry') : null;
    if (rx == null) rx = ry ?? 0; if (ry == null) ry = rx;
    rx = Math.min(Math.max(0, rx), w / 2); ry = Math.min(Math.max(0, ry), h / 2);
    if (!rx || !ry) return [{ pts: [[x, y], [x + w, y], [x + w, y + h], [x, y + h], [x, y]], closed: true }];
    const d = `M${x + rx},${y}H${x + w - rx}A${rx},${ry} 0 0 1 ${x + w},${y + ry}V${y + h - ry}A${rx},${ry} 0 0 1 ${x + w - rx},${y + h}H${x + rx}A${rx},${ry} 0 0 1 ${x},${y + h - ry}V${y + ry}A${rx},${ry} 0 0 1 ${x + rx},${y}Z`;
    return pathToPolys(d, tol);
  }
  return [];
}

// ---------- clipping (Clipper, integer space) ----------
function clipperKit(diag) {
  const CL = window.ClipperLib;
  if (!CL) return null;
  const s = 1e9 / (diag || 1);
  const toC = (pts) => pts.map(([x, y]) => ({ X: Math.round(x * s), Y: Math.round(y * s) }));
  const fromC = (path) => path.map((p) => [p.X / s, p.Y / s]);
  const exec = (type, subj, clip, subjFill, clipFill) => {
    const c = new CL.Clipper();
    if (subj.length) c.AddPaths(subj, CL.PolyType.ptSubject, true);
    if (clip.length) c.AddPaths(clip, CL.PolyType.ptClip, true);
    const out = new CL.Paths();
    c.Execute(type, out, subjFill, clipFill);
    return out;
  };
  const NZ = CL.PolyFillType.pftNonZero, EO = CL.PolyFillType.pftEvenOdd;
  return {
    toC, fromC,
    normalize: (polys, evenodd) => exec(CL.ClipType.ctUnion, polys, [], evenodd ? EO : NZ, NZ),
    union: (a, b) => exec(CL.ClipType.ctUnion, a, b, NZ, NZ),
    diff: (a, b) => exec(CL.ClipType.ctDifference, a, b, NZ, NZ),
    intersect: (a, b) => exec(CL.ClipType.ctIntersection, a, b, NZ, NZ),
    clipLines(lines, region) {
      if (!region.length) return [];
      const c = new CL.Clipper();
      c.AddPaths(lines, CL.PolyType.ptSubject, false);
      c.AddPaths(region, CL.PolyType.ptClip, true);
      const tree = new CL.PolyTree();
      c.Execute(CL.ClipType.ctIntersection, tree, NZ, NZ);
      return CL.Clipper.OpenPathsFromPolyTree(tree);
    },
  };
}

const refId = (v) => { const m = /url\(\s*["']?#([^"')\s]+)["']?\s*\)/.exec(v || ''); return m ? m[1] : null; };
function luminance(color) {
  const m = /rgba?\(([^)]+)\)/.exec(color || '');
  if (!m) return null;
  const [r, g, b, a = 1] = m[1].split(/[\s,/]+/).filter(Boolean).map(parseFloat);
  return ((0.2126 * r + 0.7152 * g + 0.0722 * b) / 255) * a;
}

export function parseSVG(text) {
  const doc = new DOMParser().parseFromString(text, 'image/svg+xml');
  if (doc.querySelector('parsererror') || !doc.documentElement || doc.documentElement.localName !== 'svg') throw new Error('This file is not a readable SVG.');
  const host = document.createElement('div');
  host.style.cssText = 'position:fixed;left:-20000px;top:0;width:1000px;height:1000px;visibility:hidden;pointer-events:none;overflow:hidden';
  const root = document.importNode(doc.documentElement, true);
  root.setAttribute('width', '1000');
  root.setAttribute('height', '1000');
  host.appendChild(root);
  document.body.appendChild(host);
  const warnings = new Set();
  const paths = [];
  const layers = new Set();
  try {
    const rootInv = root.getScreenCTM().inverse();
    const toRoot = (el) => plain(rootInv.multiply(el.getScreenCTM()));
    let diag = 1;
    try { const bb = root.getBBox(); diag = Math.hypot(bb.width, bb.height) || 1; } catch { /* empty */ }
    const tolRoot = diag * 2e-5;
    const ck = clipperKit(diag);
    const byId = (id) => { try { return root.querySelector(`#${CSS.escape(id)}`); } catch { return null; } };
    const style = (el) => { try { return getComputedStyle(el); } catch { return null; } };

    const unitBox = (ref) => { try { const b = ref.getBBox(); return M(b.width, 0, 0, b.height, b.x, b.y); } catch { return ID; } };

    // region (Clipper paths, root coords) for a clipPath referenced by `ref`
    const clipRegion = (cp, ref) => {
      let m = toRoot(ref);
      if (cp.getAttribute('clipPathUnits') === 'objectBoundingBox') m = mul(m, unitBox(ref));
      m = mul(m, ownTransform(cp));
      let region = [];
      for (const ch of cp.children) {
        if (ch.localName === 'text') { warnings.add('A clipping mask contains text; convert text to outlines first'); continue; }
        let el = ch, mm = mul(m, ownTransform(ch));
        if (ch.localName === 'use') {
          const t = byId(refId(`url(${ch.getAttribute('href') || ch.getAttribute('xlink:href')})`));
          if (!t) continue;
          el = t; mm = mul(mul(mm, M(1, 0, 0, 1, len(ch, 'x'), len(ch, 'y'))), ownTransform(t));
        }
        if (!SHAPES.has(el.localName)) continue;
        const polys = shapeToPolys(el, tolRoot / scaleOf(mm)).map((p) => ck.toC(p.pts.map(([x, y]) => ap(mm, x, y))));
        const st = style(ch);
        region = ck.union(region, ck.normalize(polys, st && st.clipRule === 'evenodd'));
      }
      return region;
    };
    // masks: walk content in paint order; light fills add, dark fills subtract
    const maskRegion = (mk, ref) => {
      let m = toRoot(ref);
      if (mk.getAttribute('maskContentUnits') === 'objectBoundingBox') m = mul(m, unitBox(ref));
      let region = [];
      const walk = (el, mm) => {
        for (const ch of el.children) {
          const tag = ch.localName;
          const m2 = mul(mm, ownTransform(ch));
          if (tag === 'g') { walk(ch, m2); continue; }
          if (tag === 'image') { warnings.add('A mask uses an image, which can’t be applied; that part was ignored'); continue; }
          if (tag === 'text') { warnings.add('A mask contains text; convert text to outlines first'); continue; }
          if (!SHAPES.has(tag)) continue;
          const st = style(ch);
          if (!st || st.fill === 'none') continue;
          let lum = luminance(st.fill);
          if (lum == null) { lum = 1; warnings.add('Gradient masks are treated as fully visible'); }
          lum *= (parseFloat(st.fillOpacity) || 1) * (parseFloat(st.opacity) || 1);
          if (lum > 0.02 && lum < 0.98) warnings.add('Partly transparent mask areas are treated as on/off');
          const polys = shapeToPolys(ch, tolRoot / scaleOf(m2)).map((p) => ck.toC(p.pts.map(([x, y]) => ap(m2, x, y))));
          const shape = ck.normalize(polys, st.fillRule === 'evenodd');
          region = lum >= 0.5 ? ck.union(region, shape) : ck.diff(region, shape);
        }
      };
      walk(mk, m);
      return region;
    };
    const regionsFor = (el) => {
      const out = [];
      const st = style(el);
      const cpId = refId(el.getAttribute('clip-path')) || refId(st && st.clipPath);
      const mkId = refId(el.getAttribute('mask')) || refId(st && (st.mask || st.maskImage));
      if (!ck && (cpId || mkId)) { warnings.add('Clipping library failed to load; masks were not applied'); return out; }
      if (cpId) { const cp = byId(cpId); if (cp && cp.localName === 'clipPath') out.push(clipRegion(cp, el)); }
      else if (st && st.clipPath && st.clipPath !== 'none' && !cpId) warnings.add('CSS clip shapes (inset/circle) aren’t supported; ignored');
      if (mkId) { const mk = byId(mkId); if (mk && mk.localName === 'mask') out.push(maskRegion(mk, el)); }
      return out;
    };

    let elemNo = 0;
    const emit = (polys, regions, layer, rule) => {
      const elem = elemNo++;
      let ls = polys.filter((p) => p.pts.length >= 2);
      if (regions.length) {
        // clipped pieces are open lines
        let cl = ls.map((p) => ck.toC(p.pts));
        for (const r of regions) cl = ck.clipLines(cl, r);
        ls = cl.map((c) => ({ pts: ck.fromC(c), closed: false }));
      }
      for (const p of ls) {
        if (p.pts.length < 2) continue;
        paths.push({ layer, pts: p.pts.map(([x, y]) => [x, -y]), closed: !!p.closed, rule, elem });
        layers.add(layer);
      }
    };

    const visit = (el, layer, regions) => {
      for (const ch of el.children) {
        const tag = ch.localName;
        if (SKIP.has(tag)) continue;
        if (tag === 'text') { warnings.add('Ignored text (convert text to outlines first)'); continue; }
        if (tag === 'use') { warnings.add('Ignored <use> clones (expand them first)'); continue; }
        const st = style(ch);
        if (st && st.display === 'none') continue;
        let lay = layer;
        if (tag === 'g' && !layer) lay = ch.getAttribute('inkscape:label') || ch.getAttribute('id') || null;
        const regs = regions.concat(regionsFor(ch));
        if (SHAPES.has(tag)) {
          const m = toRoot(ch);
          const polys = shapeToPolys(ch, tolRoot / scaleOf(m)).map((p) => ({ pts: p.pts.map(([x, y]) => ap(m, x, y)), closed: p.closed }));
          emit(polys, regs, lay || 'Drawing', st && st.fillRule === 'evenodd' ? 'evenodd' : 'nonzero');
        } else if (tag === 'svg' || tag === 'g' || tag === 'a') visit(ch, lay, regs);
      }
    };
    visit(root, null, regionsFor(root));
  } finally {
    host.remove();
  }
  return { paths, layers: [...layers], units: null, warnings: [...warnings] };
}
