// Ordering, placement (Web Mercator), simplification, splitting and KML/GPX export.

const R = 6378137;
export const merc = (lon, lat) => [R * lon * Math.PI / 180, R * Math.log(Math.tan(Math.PI / 4 + lat * Math.PI / 360))];
export const unmerc = (X, Y) => [X / R * 180 / Math.PI, (2 * Math.atan(Math.exp(Y / R)) - Math.PI / 2) * 180 / Math.PI];
export function haversineKm(lon1, lat1, lon2, lat2) {
  const r = Math.PI / 180, dLat = (lat2 - lat1) * r, dLon = (lon2 - lon1) * r;
  const a = Math.sin(dLat / 2) ** 2 + Math.cos(lat1 * r) * Math.cos(lat2 * r) * Math.sin(dLon / 2) ** 2;
  return 6371.0088 * 2 * Math.asin(Math.min(1, Math.sqrt(a)));
}
const d2 = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

export function bounds(curves) {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const c of curves) for (const [x, y] of c) { if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y; }
  return { x0, y0, x1, y1, w: x1 - x0, h: y1 - y0 };
}

// ---------- ordering ----------
// Chain all curves into one sequence that keeps straight connectors short:
// greedy nearest-neighbour (open curves may be reversed, closed loops are
// entered at their nearest vertex) followed by 2-opt on the curve sequence.
export function order(paths, { timeBudgetMs = 4000 } = {}) {
  let curves = [];
  for (const p of paths) {
    const pts = [];
    for (const q of (p.pts || p)) if (!pts.length || d2(pts[pts.length - 1], q) > 0) pts.push(q);
    if (pts.length >= 2) curves.push(pts);
  }
  if (!curves.length) return { curves: [], jumps: [] };
  const bb = bounds(curves);
  const diag = Math.hypot(bb.w, bb.h) || 1;
  const eps = diag * 1e-9;
  const closed = curves.map((c) => d2(c[0], c[c.length - 1]) <= eps * 10);

  // spatial grid of anchors (open: both ends; closed: every vertex)
  const anchors = []; // [x, y, curveIdx, vertexIdx]
  curves.forEach((c, i) => {
    if (closed[i]) c.forEach((p, j) => { if (j < c.length - 1) anchors.push([p[0], p[1], i, j]); });
    else { anchors.push([c[0][0], c[0][1], i, 0]); anchors.push([c[c.length - 1][0], c[c.length - 1][1], i, c.length - 1]); }
  });
  const cell = Math.max(diag / Math.sqrt(Math.max(1, anchors.length)) * 2, diag * 1e-6);
  const gx = (x) => Math.floor((x - bb.x0) / cell), gy = (y) => Math.floor((y - bb.y0) / cell);
  const nx = gx(bb.x1) + 1, ny = gy(bb.y1) + 1;
  const grid = new Map();
  anchors.forEach((a, k) => { const key = gx(a[0]) * ny + gy(a[1]); let b = grid.get(key); if (!b) grid.set(key, b = []); b.push(k); });
  const used = new Uint8Array(curves.length);
  let remaining = curves.length;

  const nearest = (x, y) => {
    const cx = gx(x), cy = gy(y);
    let best = -1, bd = Infinity;
    const maxR = Math.max(nx, ny) + 1;
    for (let r = 0; r <= maxR; r++) {
      if (best >= 0 && (r - 1) * cell > bd) break;
      for (let i = cx - r; i <= cx + r; i++) for (let j = cy - r; j <= cy + r; j++) {
        if (Math.max(Math.abs(i - cx), Math.abs(j - cy)) !== r) continue;
        if (i < 0 || j < 0 || i >= nx || j >= ny) continue;
        const b = grid.get(i * ny + j); if (!b) continue;
        for (let t = b.length - 1; t >= 0; t--) {
          const a = anchors[b[t]];
          if (used[a[2]]) { b[t] = b[b.length - 1]; b.pop(); continue; }
          const d = Math.hypot(a[0] - x, a[1] - y);
          if (d < bd) { bd = d; best = b[t]; }
        }
      }
    }
    return best;
  };

  const seq = [];
  let cur = [bb.x0, bb.y1]; // start at top-left
  while (remaining) {
    const k = nearest(cur[0], cur[1]);
    const [, , ci, vj] = anchors[k];
    const c = curves[ci];
    let oriented;
    if (closed[ci]) oriented = c.slice(vj, c.length - 1).concat(c.slice(0, vj + 1));
    else oriented = vj === 0 ? c : c.slice().reverse();
    used[ci] = 1; remaining--;
    seq.push(oriented);
    cur = oriented[oriented.length - 1];
  }

  // 2-opt over the sequence (reverse blocks i+1..k when it shortens connectors)
  const t0 = Date.now();
  const n = seq.length;
  const S = seq.map((c) => c[0]), E = seq.map((c) => c[c.length - 1]);
  for (let pass = 0; pass < 60 && n > 2; pass++) {
    let improved = false;
    for (let i = 0; i < n - 1; i++) {
      const base = d2(S[i + 1], E[i]);
      let bestG = 1e-12 * diag, bestK = -1;
      for (let k = i + 1; k < n; k++) {
        const oldK = k < n - 1 ? d2(S[k + 1], E[k]) : 0;
        const newK = k < n - 1 ? d2(S[k + 1], S[i + 1]) : 0;
        const g = base + oldK - d2(E[k], E[i]) - newK;
        if (g > bestG) { bestG = g; bestK = k; }
      }
      if (bestK >= 0) {
        // reverse seq[i+1..bestK] and each curve in it
        let a = i + 1, b = bestK;
        while (a < b) { [seq[a], seq[b]] = [seq[b], seq[a]]; a++; b--; }
        for (let q = i + 1; q <= bestK; q++) { seq[q] = seq[q].slice().reverse(); S[q] = seq[q][0]; E[q] = seq[q][seq[q].length - 1]; }
        improved = true;
      }
      if (Date.now() - t0 > timeBudgetMs) break;
    }
    if (!improved || Date.now() - t0 > timeBudgetMs) break;
  }

  // closed loops start and end at the same vertex: re-pick that vertex for
  // the final neighbours (2-opt moved them), minimising both connectors
  for (let sweep = 0; sweep < 2; sweep++) {
    for (let i = 0; i < seq.length; i++) {
      const c = seq[i];
      if (d2(c[0], c[c.length - 1]) > eps * 10 || c.length < 4) continue;
      const prev = i > 0 ? seq[i - 1][seq[i - 1].length - 1] : null;
      const next = i + 1 < seq.length ? seq[i + 1][0] : null;
      if (!prev && !next) continue;
      let bj = 0, bd = Infinity;
      for (let j = 0; j < c.length - 1; j++) {
        const v = (prev ? d2(prev, c[j]) : 0) + (next ? d2(c[j], next) : 0);
        if (v < bd) { bd = v; bj = j; }
      }
      if (bj) seq[i] = c.slice(bj, c.length - 1).concat(c.slice(0, bj + 1));
    }
  }

  // Splice closed loops into a neighbouring line instead of chaining them:
  // at the closest pair of points, step across, go round the loop, step
  // back. A short out-and-back spur replaces two long connectors.
  spliceLoops(seq, diag, eps);

  // merge pieces that touch end-to-start
  const out = [seq[0].slice()];
  const jumps = [];
  for (let i = 1; i < seq.length; i++) {
    const g = d2(out[out.length - 1][out[out.length - 1].length - 1], seq[i][0]);
    if (g <= eps * 10) { const last = out[out.length - 1]; for (let j = 1; j < seq[i].length; j++) last.push(seq[i][j]); }
    else { jumps.push(g); out.push(seq[i].slice()); }
  }
  return { curves: out, jumps };
}

// ---------- placement ----------
// Rotate the drawing about its centre (clockwise degrees as seen on the map),
// then map its bounding box linearly in Web Mercator onto tl/br.
export function rotatedBox(curves, rotationDeg = 0) {
  const bb = bounds(curves);
  const cx = (bb.x0 + bb.x1) / 2, cy = (bb.y0 + bb.y1) / 2;
  const t = rotationDeg * Math.PI / 180, c = Math.cos(t), s = Math.sin(t);
  const rotP = (x, y) => [(x - cx) * c + (y - cy) * s, -(x - cx) * s + (y - cy) * c];
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const cv of curves) for (const [x, y] of cv) { const [u, v] = rotP(x, y); if (u < x0) x0 = u; if (u > x1) x1 = u; if (v < y0) y0 = v; if (v > y1) y1 = v; }
  return { rotP, x0, y0, x1, y1, w: x1 - x0, h: y1 - y0 };
}

export function makePlacer(curves, { tl, br, rotationDeg = 0, box = null }) {
  const { rotP, x0, y0, x1, y1 } = box || rotatedBox(curves, rotationDeg);
  const [X1, Y1] = merc(tl[1], tl[0]);
  const [X2, Y2] = merc(br[1], br[0]);
  const sx = (X2 - X1) / ((x1 - x0) || 1), sy = (Y1 - Y2) / ((y1 - y0) || 1);
  const toMerc = (x, y) => { const [u, v] = rotP(x, y); return [X1 + (u - x0) * sx, Y2 + (v - y0) * sy]; };
  return { toMerc, rotatedAspect: (x1 - x0) / ((y1 - y0) || 1) };
}

export function rotatedAspect(curves, rotationDeg) {
  return makePlacer(curves, { tl: [1, 0], br: [0, 1], rotationDeg }).rotatedAspect;
}

// Douglas–Peucker on an array of [X, Y] with per-point tolerance function
function simplify(pts, tol) {
  const n = pts.length;
  if (n <= 2 || tol <= 0) return pts;
  const keep = new Uint8Array(n); keep[0] = keep[n - 1] = 1;
  const stack = [[0, n - 1]];
  while (stack.length) {
    const [a, b] = stack.pop();
    const [ax, ay] = pts[a], [bx, by] = pts[b];
    const dx = bx - ax, dy = by - ay, L = Math.hypot(dx, dy);
    let md = -1, mi = -1;
    for (let i = a + 1; i < b; i++) {
      const d = L > 0 ? Math.abs(dx * (pts[i][1] - ay) - dy * (pts[i][0] - ax)) / L : Math.hypot(pts[i][0] - ax, pts[i][1] - ay);
      if (d > md) { md = d; mi = i; }
    }
    if (md > tol) { keep[mi] = 1; stack.push([a, mi], [mi, b]); }
  }
  const out = [];
  for (let i = 0; i < n; i++) if (keep[i]) out.push(pts[i]);
  return out;
}

export function autoToleranceM(widthKm) {
  return Math.min(100, Math.max(1, Math.round(widthKm * 1000 / 20000)));
}

// ---------- export ----------
export function exportRoutes(ordered, opts) {
  const { tl, br, rotationDeg = 0, maxPoints = 6000, maxKm = 0, name = 'Route' } = opts;
  const curves = ordered.curves;
  if (!curves.length) return { routes: [], totalKm: 0, bbox: null };
  const placer = makePlacer(curves, { tl, br, rotationDeg, box: opts.box || null });
  const widthKm = haversineKm(tl[1], (tl[0] + br[0]) / 2, br[1], (tl[0] + br[0]) / 2);
  const toleranceM = opts.toleranceM > 0 ? opts.toleranceM : autoToleranceM(widthKm);

  // project + simplify each curve; pre-split very long curves
  const cap = Math.max(50, Math.floor(maxPoints / 10));
  const pieces = []; // {ll: [[lon,lat]...], km, gapBefore(km)}
  let west = Infinity, east = -Infinity, south = Infinity, north = -Infinity;
  let prevEnd = null;
  for (const c of curves) {
    const m = c.map(([x, y]) => placer.toMerc(x, y));
    for (const [X, Y] of m) { const [lon, lat] = unmerc(X, Y); if (lon < west) west = lon; if (lon > east) east = lon; if (lat < south) south = lat; if (lat > north) north = lat; }
    const midLat = unmerc(0, m[Math.floor(m.length / 2)][1])[1];
    const s = simplify(m, toleranceM / Math.cos(midLat * Math.PI / 180));
    const ll = s.map(([X, Y]) => unmerc(X, Y));
    for (let st = 0; st < ll.length - 1; st += cap - 1) {
      const part = ll.slice(st, Math.min(ll.length, st + cap));
      let km = 0;
      for (let i = 1; i < part.length; i++) km += haversineKm(part[i - 1][0], part[i - 1][1], part[i][0], part[i][1]);
      const gap = prevEnd ? haversineKm(prevEnd[0], prevEnd[1], part[0][0], part[0][1]) : 0;
      pieces.push({ ll: part, km, gap });
      prevEnd = part[part.length - 1];
    }
  }

  // split into routes: cut at the widest gaps near even point/length targets
  const P = pieces.map((p) => p.ll.length);
  const totalPts = P.reduce((a, b) => a + b, 0);
  const totalKm = pieces.reduce((a, p) => a + p.km + p.gap, 0);
  // Fewest routes the limits allow (greedy fill gives that count), then a
  // dynamic programme picks the cut positions that remove the longest
  // straight connectors.
  const N = pieces.length;
  const cp = new Float64Array(N + 1), ck = new Float64Array(N + 1);
  for (let i = 0; i < N; i++) { cp[i + 1] = cp[i] + P[i]; ck[i + 1] = ck[i] + pieces[i].km + pieces[i].gap; }
  const fits = (i, j) => cp[j] - cp[i] <= maxPoints && (!maxKm || j - i === 1 || ck[j] - ck[i] - pieces[i].gap <= maxKm);
  let K = 0;
  for (let a = 0; a < N; K++) { let j = a + 1; while (j < N && fits(a, j + 1)) j++; a = j; }
  let bounds_ = [0, N];
  if (K > 1) {
    const NEG = -Infinity;
    let prev = new Float64Array(N + 1).fill(NEG); prev[0] = 0;
    const arg = [];
    for (let k = 1; k <= K; k++) {
      const cur = new Float64Array(N + 1).fill(NEG);
      const ak = new Int32Array(N + 1).fill(-1);
      for (let j = 1; j <= N; j++) {
        for (let i = j - 1; i >= 0; i--) {
          if (!fits(i, j)) break;
          if (prev[i] === NEG) continue;
          const v = prev[i] + (i > 0 ? pieces[i].gap : 0);
          if (v > cur[j]) { cur[j] = v; ak[j] = i; }
        }
      }
      arg.push(ak); prev = cur;
    }
    if (prev[N] !== NEG) {
      const b = [N]; let j = N;
      for (let k = K; k >= 1; k--) { j = arg[k - 1][j]; b.push(j); }
      bounds_ = b.reverse();
    }
  }

  const routes = [];
  const nR = bounds_.length - 1;
  const pad = String(nR).length;
  for (let k = 0; k < nR; k++) {
    const coords = [];
    let km = 0, maxGap = 0;
    for (let i = bounds_[k]; i < bounds_[k + 1]; i++) {
      const p = pieces[i];
      if (i > bounds_[k]) { km += p.gap; if (p.gap > maxGap) maxGap = p.gap; }
      km += p.km;
      for (const q of p.ll) {
        const l = coords[coords.length - 1];
        if (!l || Math.abs(l[0] - q[0]) > 1e-7 || Math.abs(l[1] - q[1]) > 1e-7) coords.push(q);
      }
    }
    const rname = nR > 1 ? `${name} ${String(k + 1).padStart(pad, '0')} of ${nR}` : name;
    routes.push({ name: rname, index: k + 1, points: coords.length, km, maxConnectorKm: maxGap, coords, kml: toKML(rname, coords), gpx: toGPX(rname, coords) });
  }
  const routesKm = routes.reduce((a, r) => a + r.km, 0);
  return { routes, totalKm: routesKm, toleranceM, widthKm, bbox: { west, south, east, north } };
}

const esc = (s) => String(s).replace(/[<>&"']/g, (c) => ({ '<': '&lt;', '>': '&gt;', '&': '&amp;', '"': '&quot;', "'": '&apos;' }[c]));
export function toKML(name, coords) {
  return `<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2"><Document><name>${esc(name)}</name>
<Placemark><name>${esc(name)}</name><LineString><tessellate>1</tessellate><coordinates>
${coords.map(([lon, lat]) => `${lon.toFixed(6)},${lat.toFixed(6)}`).join('\n')}
</coordinates></LineString></Placemark></Document></kml>
`;
}
export function toGPX(name, coords) {
  return `<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="route-art" xmlns="http://www.topografix.com/GPX/1/1"><trk><name>${esc(name)}</name><trkseg>
${coords.map(([lon, lat]) => `<trkpt lat="${lat.toFixed(6)}" lon="${lon.toFixed(6)}"/>`).join('\n')}
</trkseg></trk></gpx>
`;
}

// ---------- grouped export ----------
// Every group (a shape, or everything joined) becomes its own route(s); all
// groups share one placement so they stay registered to each other.
export function exportGroups(prepared, opts) {
  const all = prepared.curves || prepared.groups.flatMap((g) => g.curves);
  if (!all.length) return { routes: [], totalKm: 0, bbox: null };
  const box = opts.box || rotatedBox(all, opts.rotationDeg || 0);
  const name = opts.name || 'Route';
  const parts = prepared.groups.map((g) => exportRoutes(g, { ...opts, box, name: 'tmp' }));
  const multi = prepared.groups.length > 1;
  const routes = [];
  let west = Infinity, south = Infinity, east = -Infinity, north = -Infinity;
  parts.forEach((res, gi) => {
    if (res.bbox) { west = Math.min(west, res.bbox.west); east = Math.max(east, res.bbox.east); south = Math.min(south, res.bbox.south); north = Math.max(north, res.bbox.north); }
    const label = multi ? `${name} ${prepared.groups[gi].name || gi + 1}` : name;
    res.routes.forEach((rt, k) => {
      const rname = res.routes.length > 1 ? `${label} (${k + 1} of ${res.routes.length})` : label;
      routes.push({ ...rt, group: gi, name: rname, kml: toKML(rname, rt.coords), gpx: toGPX(rname, rt.coords) });
    });
  });
  routes.forEach((r, i) => { r.index = i + 1; });
  const first = parts.find((p) => p.routes.length) || parts[0];
  return { routes, totalKm: routes.reduce((a, r) => a + r.km, 0), toleranceM: first.toleranceM, widthKm: first.widthKm, bbox: { west, south, east, north } };
}

function spliceLoops(seq, diag, eps) {
  const isClosed = (c) => c.length >= 4 && d2(c[0], c[c.length - 1]) <= eps * 10;
  if (seq.length < 2 || seq.filter(isClosed).length > 300) return;
  const t0 = Date.now();
  for (let guard = 0; guard < 400 && seq.length > 1 && Date.now() - t0 < 3000; guard++) {
    // grid of every vertex: [x, y, curve, index]
    const total = seq.reduce((a, c) => a + c.length, 0);
    const cell = Math.max(diag / Math.sqrt(total) * 2, diag * 1e-6);
    const grid = new Map();
    const key = (x, y) => `${Math.floor(x / cell)},${Math.floor(y / cell)}`;
    seq.forEach((c, ci) => c.forEach((p, vi) => { const k = key(p[0], p[1]); let b = grid.get(k); if (!b) grid.set(k, b = []); b.push(ci, vi); }));
    const nearestOther = (p, self, limit) => {
      const gx = Math.floor(p[0] / cell), gy = Math.floor(p[1] / cell);
      let best = null, bd = limit;
      for (let r = 0; r * cell <= bd + cell; r++) {
        for (let i = gx - r; i <= gx + r; i++) for (let j = gy - r; j <= gy + r; j++) {
          if (Math.max(Math.abs(i - gx), Math.abs(j - gy)) !== r) continue;
          const b = grid.get(`${i},${j}`); if (!b) continue;
          for (let t = 0; t < b.length; t += 2) {
            if (b[t] === self) continue;
            const q = seq[b[t]][b[t + 1]];
            const d = Math.hypot(q[0] - p[0], q[1] - p[1]);
            if (d < bd) { bd = d; best = [b[t], b[t + 1], d]; }
          }
        }
        if (r > 64) break;
      }
      return best;
    };
    let bestMove = null;
    for (let i = 0; i < seq.length; i++) {
      const L = seq[i];
      if (!isClosed(L)) continue;
      const prev = i > 0 ? seq[i - 1][seq[i - 1].length - 1] : null;
      const next = i + 1 < seq.length ? seq[i + 1][0] : null;
      const gain = (prev ? d2(prev, L[0]) : 0) + (next ? d2(L[L.length - 1], next) : 0) - (prev && next ? d2(prev, next) : 0);
      if (gain <= 1e-9 * diag) continue;
      const stride = Math.max(1, Math.floor(L.length / 600));
      let cand = null;
      for (let w = 0; w < L.length - 1; w += stride) {
        const hit = nearestOther(L[w], i, cand ? cand.d : gain / 2);
        if (hit) cand = { w, ci: hit[0], vi: hit[1], d: hit[2] };
      }
      if (cand && 2 * cand.d < gain - 1e-9 * diag && (!bestMove || gain - 2 * cand.d > bestMove.save)) bestMove = { i, ...cand, save: gain - 2 * cand.d };
    }
    if (!bestMove) break;
    const { i, w, ci, vi } = bestMove;
    const L = seq[i];
    const Lrot = L.slice(w, L.length - 1).concat(L.slice(0, w + 1));
    const C = seq[ci];
    seq[ci] = C.slice(0, vi + 1).concat(Lrot, C.slice(vi));
    seq.splice(i, 1);
  }
}
