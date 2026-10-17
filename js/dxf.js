// DXF → polylines in drawing units (y up).
// Handles LINE, LWPOLYLINE/POLYLINE (with bulges), ARC, CIRCLE, ELLIPSE,
// SPLINE (NURBS, rational, fit-point fallback) and nested INSERTs (blocks with
// base point, scale, rotation, column/row arrays, OCS mirror via extrusion).

const UNITS = { 1: 'in', 2: 'ft', 3: 'mi', 4: 'mm', 5: 'cm', 6: 'm', 7: 'km', 8: 'µin', 9: 'mil', 10: 'yd', 14: 'dm' };
const SKIP_WARN = { TEXT: 'text', MTEXT: 'text', ATTRIB: 'text', ATTDEF: 'text', HATCH: 'hatch fills', DIMENSION: 'dimensions', IMAGE: 'images', SOLID: 'solid fills', MESH: 'meshes', '3DFACE': '3D faces', REGION: 'regions', '3DSOLID': '3D solids', LEADER: 'leaders', MLEADER: 'leaders' };

function tokenize(text) {
  const lines = text.split(/\r\n|\r|\n/);
  const out = [];
  for (let i = 0; i + 1 < lines.length; i += 2) {
    const code = parseInt(lines[i].trim(), 10);
    if (Number.isNaN(code)) { i -= 1; continue; } // resync on junk
    out.push([code, lines[i + 1].trim()]);
  }
  return out;
}

// Group tokens into entities: {type, g: [[code, value], ...]}
function readEntities(tok, start, stopTypes) {
  const ents = [];
  let i = start;
  while (i < tok.length) {
    const [c, v] = tok[i];
    if (c !== 0) { i++; continue; }
    if (stopTypes.includes(v)) return { ents, next: i };
    const e = { type: v, g: [] };
    i++;
    while (i < tok.length && tok[i][0] !== 0) { e.g.push(tok[i]); i++; }
    ents.push(e);
  }
  return { ents, next: i };
}

const num = (e, code, def = 0) => { for (const [c, v] of e.g) if (c === code) return parseFloat(v); return def; };
const str = (e, code, def = '') => { for (const [c, v] of e.g) if (c === code) return v; return def; };
const nums = (e, code) => e.g.filter(([c]) => c === code).map(([, v]) => parseFloat(v));

// Attach VERTEX/SEQEND to POLYLINE (and ATTRIB/SEQEND to INSERT)
function nestSequences(ents) {
  const out = [];
  for (let i = 0; i < ents.length; i++) {
    const e = ents[i];
    if (e.type === 'POLYLINE' || (e.type === 'INSERT' && num(e, 66) === 1)) {
      e.vertices = [];
      i++;
      while (i < ents.length && ents[i].type !== 'SEQEND') { if (ents[i].type === 'VERTEX') e.vertices.push(ents[i]); i++; }
    }
    out.push(e);
  }
  return out;
}

// --- 2D affine: [a, b, c, d, e, f] : x' = a x + c y + e ; y' = b x + d y + f
const I = [1, 0, 0, 1, 0, 0];
const mul = (m, n) => [
  m[0] * n[0] + m[2] * n[1], m[1] * n[0] + m[3] * n[1],
  m[0] * n[2] + m[2] * n[3], m[1] * n[2] + m[3] * n[3],
  m[0] * n[4] + m[2] * n[5] + m[4], m[1] * n[4] + m[3] * n[5] + m[5],
];
const tr = (x, y) => [1, 0, 0, 1, x, y];
const rot = (deg) => { const r = deg * Math.PI / 180, c = Math.cos(r), s = Math.sin(r); return [c, s, -s, c, 0, 0]; };
const scl = (sx, sy) => [sx, 0, 0, sy, 0, 0];
const MIRROR_X = [-1, 0, 0, 1, 0, 0];
const apply = (m, x, y) => [m[0] * x + m[2] * y + m[4], m[1] * x + m[3] * y + m[5]];
const ocs = (e) => (num(e, 230, 1) < 0 ? MIRROR_X : I); // arbitrary-axis for (0,0,-1) is x → -x

// --- curve sampling helpers (tol = max chord deviation, drawing units)
function arcPts(cx, cy, r, a0, a1, tol) {
  // a0..a1 radians CCW, a1 > a0
  const sweep = a1 - a0;
  const maxStep = r > tol ? 2 * Math.acos(Math.max(-1, 1 - tol / r)) : Math.PI / 2;
  const n = Math.max(2, Math.ceil(sweep / Math.min(maxStep, Math.PI / 8)));
  const pts = [];
  for (let i = 0; i <= n; i++) { const a = a0 + sweep * i / n; pts.push([cx + r * Math.cos(a), cy + r * Math.sin(a)]); }
  return pts;
}

function bulgeSeg(p0, p1, b, tol, out) {
  // append points strictly after p0 up to and including p1
  if (!b || Math.abs(b) < 1e-12) { out.push(p1); return; }
  const theta = 4 * Math.atan(b); // included angle, signed (CCW positive)
  const dx = p1[0] - p0[0], dy = p1[1] - p0[1], chord = Math.hypot(dx, dy);
  if (chord < 1e-15) { out.push(p1); return; }
  const r = chord / (2 * Math.sin(Math.abs(theta) / 2));
  // centre sits on the chord's left normal, signed offset (chord/2)·(1−b²)/(2b)
  const mx = (p0[0] + p1[0]) / 2, my = (p0[1] + p1[1]) / 2;
  const h = (chord / 2) * (1 - b * b) / (2 * b);
  const nx = -dy / chord, ny = dx / chord;
  const cx = mx + nx * h, cy = my + ny * h;
  const a0 = Math.atan2(p0[1] - cy, p0[0] - cx);
  const maxStep = r > tol ? 2 * Math.acos(Math.max(-1, 1 - tol / r)) : Math.PI / 2;
  const n = Math.max(2, Math.ceil(Math.abs(theta) / Math.min(maxStep, Math.PI / 8)));
  for (let i = 1; i < n; i++) { const a = a0 + theta * i / n; out.push([cx + r * Math.cos(a), cy + r * Math.sin(a)]); }
  out.push(p1);
}

function polyWithBulges(verts, bulges, closed, tol) {
  if (verts.length === 0) return [];
  const out = [verts[0]];
  const n = verts.length;
  const last = closed ? n : n - 1;
  for (let i = 0; i < last; i++) bulgeSeg(verts[i], verts[(i + 1) % n], bulges[i] || 0, tol, out);
  return out;
}

// NURBS evaluation (de Boor, rational aware)
function findSpan(knots, p, nCtrl, t) {
  const hi = nCtrl; // valid spans p .. nCtrl-1
  if (t >= knots[hi]) { let k = hi - 1; while (k > p && knots[k] === knots[k + 1]) k--; return k; }
  let lo = p, h = hi;
  while (h - lo > 1) { const mid = (lo + h) >> 1; if (t < knots[mid]) h = mid; else lo = mid; }
  return lo;
}
function deBoor(knots, ctrlW, p, t, k) {
  // ctrlW: [[x*w, y*w, w]]
  const d = [];
  for (let j = 0; j <= p; j++) d.push(ctrlW[j + k - p].slice());
  for (let r = 1; r <= p; r++) {
    for (let j = p; j >= r; j--) {
      const i = j + k - p;
      const den = knots[i + p - r + 1] - knots[i];
      const a = den === 0 ? 0 : (t - knots[i]) / den;
      for (let q = 0; q < 3; q++) d[j][q] = (1 - a) * d[j - 1][q] + a * d[j][q];
    }
  }
  return [d[p][0] / d[p][2], d[p][1] / d[p][2]];
}
function splinePts(e, tol, warn) {
  const p = Math.round(num(e, 71, 3));
  let knots = nums(e, 40);
  const ws = nums(e, 41);
  const xs = nums(e, 10), ys = nums(e, 20);
  const ctrl = xs.map((x, i) => [x, ys[i] ?? 0]);
  if (ctrl.length < 2) {
    const fx = nums(e, 11), fy = nums(e, 21);
    const fit = fx.map((x, i) => [x, fy[i] ?? 0]);
    if (fit.length >= 2) { warn('Some splines only had fit points and were approximated'); return catmull(fit, tol, (num(e, 70) & 1) === 1); }
    return [];
  }
  const n = ctrl.length;
  if (knots.length !== n + p + 1) {
    // build clamped uniform knots
    knots = [];
    for (let i = 0; i <= p; i++) knots.push(0);
    for (let i = 1; i < n - p; i++) knots.push(i);
    for (let i = 0; i <= p; i++) knots.push(Math.max(1, n - p));
  }
  const ctrlW = ctrl.map((c, i) => { const w = ws.length === n ? ws[i] : 1; return [c[0] * w, c[1] * w, w]; });
  const t0 = knots[p], t1 = knots[n];
  const pts = [];
  const ev = (t) => deBoor(knots, ctrlW, p, t, findSpan(knots, p, n, t));
  const distToChord = (a, b, m) => {
    const dx = b[0] - a[0], dy = b[1] - a[1], L = Math.hypot(dx, dy);
    if (L < 1e-15) return Math.hypot(m[0] - a[0], m[1] - a[1]);
    return Math.abs(dx * (m[1] - a[1]) - dy * (m[0] - a[0])) / L;
  };
  const rec = (ta, pa, tb, pb, depth) => {
    const tm = (ta + tb) / 2, pm = ev(tm);
    // also probe quarter points to avoid missing S-bends
    const q1 = ev((ta + tm) / 2), q3 = ev((tm + tb) / 2);
    const err = Math.max(distToChord(pa, pb, pm), distToChord(pa, pb, q1), distToChord(pa, pb, q3));
    if (depth > 18 || err <= tol) { pts.push(pb); return; }
    rec(ta, pa, tm, pm, depth + 1);
    rec(tm, pm, tb, pb, depth + 1);
  };
  // iterate over distinct knot spans
  const breaks = [];
  for (let i = p; i <= n; i++) if (breaks.length === 0 || knots[i] > breaks[breaks.length - 1]) breaks.push(knots[i]);
  if (breaks.length < 2) return [];
  let prev = ev(t0);
  pts.push(prev);
  for (let b = 0; b + 1 < breaks.length; b++) {
    const ta = breaks[b], tb = breaks[b + 1];
    // split each span in 2 up front so symmetric bends are seen
    const tm = (ta + tb) / 2, pm = ev(tm), pb = ev(tb);
    rec(ta, prev, tm, pm, 0);
    rec(tm, pm, tb, pb, 0);
    prev = pb;
  }
  if (t1 !== breaks[breaks.length - 1]) pts.push(ev(t1));
  return pts;
}
function catmull(fit, tol, closed) {
  const P = closed ? [fit[fit.length - 1], ...fit, fit[0], fit[1]] : [fit[0], ...fit, fit[fit.length - 1]];
  const out = [fit[0]];
  for (let i = 1; i + 2 < P.length; i++) {
    const [p0, p1, p2, p3] = [P[i - 1], P[i], P[i + 1], P[i + 2]];
    const L = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
    const n = Math.max(4, Math.min(64, Math.ceil(Math.sqrt(L / Math.max(tol, 1e-12)))));
    for (let s = 1; s <= n; s++) {
      const t = s / n, t2 = t * t, t3 = t2 * t;
      out.push([0, 1].map((q) => 0.5 * (2 * p1[q] + (-p0[q] + p2[q]) * t + (2 * p0[q] - 5 * p1[q] + 4 * p2[q] - p3[q]) * t2 + (-p0[q] + 3 * p1[q] - 3 * p2[q] + p3[q]) * t3)));
    }
  }
  return out;
}

// --- main
export function parseDXF(text) {
  const tok = tokenize(text);
  const warnings = new Set();
  const warn = (w) => warnings.add(w);
  let units = null;
  const blocks = new Map();
  let entities = [];

  for (let i = 0; i < tok.length; i++) {
    if (tok[i][0] === 0 && tok[i][1] === 'SECTION' && tok[i + 1] && tok[i + 1][0] === 2) {
      const name = tok[i + 1][1];
      if (name === 'HEADER') {
        for (let j = i + 2; j < tok.length && !(tok[j][0] === 0 && tok[j][1] === 'ENDSEC'); j++) {
          if (tok[j][0] === 9 && tok[j][1] === '$INSUNITS') {
            const v = parseInt(tok[j + 1][1], 10); units = UNITS[v] || null;
          }
        }
      } else if (name === 'BLOCKS') {
        let j = i + 2;
        while (j < tok.length && !(tok[j][0] === 0 && tok[j][1] === 'ENDSEC')) {
          if (tok[j][0] === 0 && tok[j][1] === 'BLOCK') {
            const head = { type: 'BLOCK', g: [] };
            j++;
            while (j < tok.length && tok[j][0] !== 0) { head.g.push(tok[j]); j++; }
            const r = readEntities(tok, j, ['ENDBLK']);
            blocks.set(str(head, 2), { base: [num(head, 10), num(head, 20)], ents: nestSequences(r.ents) });
            j = r.next + 1;
          } else j++;
        }
      } else if (name === 'ENTITIES') {
        const r = readEntities(tok, i + 2, ['ENDSEC']);
        entities = nestSequences(r.ents);
      }
    }
  }

  // Pass 1: bounds from defining points (through inserts) to pick sampling tolerance.
  let bx0 = Infinity, by0 = Infinity, bx1 = -Infinity, by1 = -Infinity;
  const grow = (m, x, y) => { const [X, Y] = apply(m, x, y); if (X < bx0) bx0 = X; if (X > bx1) bx1 = X; if (Y < by0) by0 = Y; if (Y > by1) by1 = Y; };
  const walk = (ents, m, depth, fn, layer) => {
    if (depth > 12) { warn('Blocks nested too deeply were skipped'); return; }
    for (const e of ents) {
      const lay = (layer && str(e, 8, '0') === '0') ? layer : str(e, 8, '0');
      if (e.type === 'INSERT') {
        const b = blocks.get(str(e, 2));
        if (!b) { warn('Some blocks were missing and skipped'); continue; }
        const sx = num(e, 41, 1), sy = num(e, 42, 1), r = num(e, 50, 0);
        const cols = Math.max(1, num(e, 70, 1)), rows = Math.max(1, num(e, 71, 1));
        const cs = num(e, 44, 0), rs = num(e, 45, 0);
        for (let ci = 0; ci < cols; ci++) for (let ri = 0; ri < rows; ri++) {
          let t = mul(tr(num(e, 10), num(e, 20)), rot(r));
          t = mul(t, tr(ci * cs, ri * rs));
          t = mul(t, scl(sx, sy));
          t = mul(t, tr(-b.base[0], -b.base[1]));
          t = mul(ocs(e), t);
          walk(b.ents, mul(m, t), depth + 1, fn, lay);
        }
      } else fn(e, m, lay);
    }
  };
  walk(entities, I, 0, (e, m) => {
    const o = mul(m, ocs(e));
    const xs = nums(e, 10), ys = nums(e, 20);
    xs.forEach((x, k) => grow(o, x, ys[k] ?? 0));
    const r = num(e, 40, 0);
    if ((e.type === 'CIRCLE' || e.type === 'ARC') && xs.length) { grow(o, xs[0] + r, ys[0] + r); grow(o, xs[0] - r, ys[0] - r); }
    if (e.type === 'ELLIPSE' && xs.length) { const mx = num(e, 11), my = num(e, 21), L = Math.hypot(mx, my); grow(o, xs[0] + L, ys[0] + L); grow(o, xs[0] - L, ys[0] - L); }
    if (e.vertices) e.vertices.forEach((v) => grow(o, num(v, 10), num(v, 20)));
    if (e.type === 'LINE') grow(o, num(e, 11), num(e, 21));
  }, null);
  const diag = Number.isFinite(bx0) ? Math.hypot(bx1 - bx0, by1 - by0) || 1 : 1;
  const tolDraw = diag * 2e-5; // chord tolerance in world units

  // Pass 2: sample.
  const paths = [];
  const layers = new Set();
  const push = (pts, m, layer, closed) => {
    if (pts.length < 2) return;
    const w = pts.map(([x, y]) => apply(m, x, y));
    paths.push({ layer, pts: w, closed: !!closed });
    layers.add(layer);
  };
  const scaleOf = (m) => Math.sqrt(Math.abs(m[0] * m[3] - m[1] * m[2])) || 1;
  walk(entities, I, 0, (e, m, layer) => {
    const o = mul(m, ocs(e));
    const tol = tolDraw / scaleOf(o);
    switch (e.type) {
      case 'LINE': push([[num(e, 10), num(e, 20)], [num(e, 11), num(e, 21)]], o, layer); break;
      case 'LWPOLYLINE': {
        const verts = [], bulges = [];
        for (const [c, v] of e.g) {
          if (c === 10) { verts.push([parseFloat(v), 0]); bulges.push(0); }
          else if (c === 20 && verts.length) verts[verts.length - 1][1] = parseFloat(v);
          else if (c === 42 && verts.length) bulges[bulges.length - 1] = parseFloat(v);
        }
        const closed = (num(e, 70, 0) & 1) === 1;
        push(polyWithBulges(verts, bulges, closed, tol), o, layer, closed);
        break;
      }
      case 'POLYLINE': {
        const f = num(e, 70, 0);
        if (f & 16 || f & 64) { warn('Ignored meshes'); break; }
        const vs = (e.vertices || []).filter((v) => !(num(v, 70, 0) & 16));
        const verts = vs.map((v) => [num(v, 10), num(v, 20)]);
        const bulges = vs.map((v) => num(v, 42, 0));
        const closed = (f & 1) === 1;
        push(polyWithBulges(verts, bulges, closed, tol), o, layer, closed);
        break;
      }
      case 'ARC': {
        let a0 = num(e, 50) * Math.PI / 180, a1 = num(e, 51) * Math.PI / 180;
        while (a1 <= a0) a1 += 2 * Math.PI;
        push(arcPts(num(e, 10), num(e, 20), num(e, 40), a0, a1, tol), o, layer);
        break;
      }
      case 'CIRCLE': push(arcPts(num(e, 10), num(e, 20), num(e, 40), 0, 2 * Math.PI, tol), o, layer, true); break;
      case 'ELLIPSE': {
        const cx = num(e, 10), cy = num(e, 20), mx = num(e, 11), my = num(e, 21), ratio = num(e, 40, 1);
        let t0 = num(e, 41, 0), t1 = num(e, 42, 2 * Math.PI);
        while (t1 <= t0) t1 += 2 * Math.PI;
        const R = Math.hypot(mx, my), rmin = R * ratio;
        const step = rmin > 0 ? Math.min(Math.PI / 16, 2 * Math.acos(Math.max(-1, 1 - tol / R))) : Math.PI / 16;
        const n = Math.max(4, Math.ceil((t1 - t0) / step));
        const pts = [];
        for (let i = 0; i <= n; i++) {
          const t = t0 + (t1 - t0) * i / n, c = Math.cos(t), s = Math.sin(t);
          pts.push([cx + c * mx - s * ratio * my, cy + c * my + s * ratio * mx]);
        }
        push(pts, o, layer, Math.abs(t1 - t0 - 2 * Math.PI) < 1e-9);
        break;
      }
      case 'SPLINE': {
        const pts = splinePts(e, tol, warn);
        push(pts, o, layer, (num(e, 70, 0) & 1) === 1);
        break;
      }
      default:
        if (SKIP_WARN[e.type]) warn(`Ignored ${SKIP_WARN[e.type]}`);
    }
  }, null);

  return { paths, layers: [...layers], units, warnings: [...warnings] };
}
