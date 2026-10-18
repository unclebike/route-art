// Squiggle fill: one unbroken line per connected part of a shape.
//
// 1. Rasterise the shape (outer ring + holes, even-odd) at spacing/4.
// 2. Keep only cells at least `clearance` from the boundary.
// 3. Grow a random spanning tree on a 2×spacing grid of the cells that fit.
// 4. Walk around the tree: that walk visits every node of the spacing grid
//    exactly once and closes on itself (a Hamiltonian cycle), so the fill is
//    a single loop with no crossings and no connectors.
// 5. Round the right-angle corners (Chaikin) for the squiggle look.

export function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6D2B79F5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function ringsBBox(rings) {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const r of rings) for (const [x, y] of r) { if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y; }
  return { x0, y0, x1, y1 };
}

// even-odd scanline fill of all rings into a W×H mask (pixel centres)
function rasterize(rings, ox, oy, r, W, H) {
  const mask = new Uint8Array(W * H);
  const xs = [];
  for (let j = 0; j < H; j++) {
    const y = oy + (j + 0.5) * r;
    xs.length = 0;
    for (const ring of rings) {
      const n = ring.length;
      for (let i = 0; i < n; i++) {
        const [ax, ay] = ring[i], [bx, by] = ring[(i + 1) % n];
        if ((ay <= y && by > y) || (by <= y && ay > y)) xs.push(ax + (y - ay) / (by - ay) * (bx - ax));
      }
    }
    xs.sort((a, b) => a - b);
    for (let k = 0; k + 1 < xs.length; k += 2) {
      const i0 = Math.max(0, Math.ceil((xs[k] - ox) / r - 0.5));
      const i1 = Math.min(W - 1, Math.floor((xs[k + 1] - ox) / r - 0.5));
      for (let i = i0; i <= i1; i++) mask[j * W + i] = 1;
    }
  }
  return mask;
}

// (kept for reference/tests) distance in pixels from each inside pixel to the nearest outside pixel (chamfer 3-4)
function distanceInside(mask, W, H) {
  const BIG = 1e9;
  const d = new Float64Array(W * H);
  for (let i = 0; i < W * H; i++) d[i] = mask[i] ? BIG : 0;
  const at = (i, j) => (i < 0 || j < 0 || i >= W || j >= H ? 0 : d[j * W + i]);
  for (let j = 0; j < H; j++) for (let i = 0; i < W; i++) {
    const k = j * W + i; if (!d[k]) continue;
    d[k] = Math.min(d[k], at(i - 1, j) + 3, at(i, j - 1) + 3, at(i - 1, j - 1) + 4, at(i + 1, j - 1) + 4);
  }
  for (let j = H - 1; j >= 0; j--) for (let i = W - 1; i >= 0; i--) {
    const k = j * W + i; if (!d[k]) continue;
    d[k] = Math.min(d[k], at(i + 1, j) + 3, at(i, j + 1) + 3, at(i + 1, j + 1) + 4, at(i - 1, j + 1) + 4);
  }
  for (let i = 0; i < W * H; i++) d[i] /= 3;
  return d;
}

function chaikinClosed(pts, iterations) {
  let p = pts;
  for (let it = 0; it < iterations; it++) {
    const out = [];
    const n = p.length;
    for (let i = 0; i < n; i++) {
      const a = p[i], b = p[(i + 1) % n];
      out.push([0.75 * a[0] + 0.25 * b[0], 0.75 * a[1] + 0.25 * b[1]]);
      out.push([0.25 * a[0] + 0.75 * b[0], 0.25 * a[1] + 0.75 * b[1]]);
    }
    p = out;
  }
  return p;
}

/**
 * @param {Array<Array<[number,number]>>} rings outer ring first, then holes (even-odd)
 * @param {{spacing:number, seed?:number, clearance?:number, smooth?:number}} opts
 * @returns {Array<Array<[number,number]>>} closed loops (first point repeated at the end)
 */
export function squiggleFill(rings, opts = {}) {
  const { angle = 'auto' } = opts;
  if (angle !== 'auto') return fillAt(rings, { ...opts, angle: +angle || 0 });
  // try a few grid angles and keep the one that needs the fewest lines and covers the most
  // (angles every 7.5°, two grid offsets); stop early once one line covers it well
  let best = null;
  for (let a = 0; a < 90; a += 7.5) {
    for (const phase of [0, 0.5]) {
      const res = fillAt(rings, { ...opts, angle: a, phase });
      const score = res.loops.length * 1e9 - res.nodes;
      if (res.loops.length && (!best || score < best.score)) best = { score, res };
    }
  }
  return best ? best.res.loops : [];
}

export function squiggleFillInfo(rings, opts = {}) {
  return fillAt(rings, { ...opts, angle: +opts.angle || 0 });
}

function fillAt(rings0, { spacing, seed = 1, clearance = 0.45, smooth = 3, wobble = 0, angle = 0, phase = 0 } = {}) {
  // work in a frame rotated by -angle so the grid runs at +angle on the page
  const t = angle * Math.PI / 180, ct = Math.cos(t), st = Math.sin(t);
  const bb0 = ringsBBox(rings0);
  const pcx = (bb0.x0 + bb0.x1) / 2, pcy = (bb0.y0 + bb0.y1) / 2;
  const toLocal = ([x, y]) => [(x - pcx) * ct + (y - pcy) * st, -(x - pcx) * st + (y - pcy) * ct];
  const toPage = ([u, v]) => [pcx + u * ct - v * st, pcy + u * st + v * ct];
  const rings = angle ? rings0.map((r) => r.map(toLocal)) : rings0;
  const s = spacing;
  if (!(s > 0) || !rings.length) return { loops: [], nodes: 0 };
  const bb = ringsBBox(rings);
  const cell = 2 * s;
  const nCx = Math.floor((bb.x1 - bb.x0) / cell), nCy = Math.floor((bb.y1 - bb.y0) / cell);
  if (nCx < 1 || nCy < 1) return { loops: [], nodes: 0 };
  // centre the coarse grid inside the bbox
  const gx0 = bb.x0 + ((bb.x1 - bb.x0) - nCx * cell) / 2 - phase * s;
  const gy0 = bb.y0 + ((bb.y1 - bb.y0) - nCy * cell) / 2 - phase * s;
  const r = s / 4;
  const ox = bb.x0 - r, oy = bb.y0 - r;
  const W = Math.ceil((bb.x1 - bb.x0) / r) + 3, H = Math.ceil((bb.y1 - bb.y0) / r) + 3;
  if (W * H > 40e6) throw new Error('Fill is too dense for this shape; lower the density.');
  const mask = rasterize(rings, ox, oy, r, W, H);
  const insideAt = (x, y) => {
    const i = Math.floor((x - ox) / r), j = Math.floor((y - oy) / r);
    return i >= 0 && j >= 0 && i < W && j < H && mask[j * W + i] === 1;
  };

  // exact clearance: outline segments bucketed in a grid, segment-to-segment distances
  const G = 2 * s;
  const segs = [];
  for (const ring of rings) for (let i = 0; i < ring.length; i++) {
    const a = ring[i], b = ring[(i + 1) % ring.length];
    if (a[0] !== b[0] || a[1] !== b[1]) segs.push([a[0], a[1], b[0], b[1]]);
  }
  const egrid = new Map();
  const gk = (i, j) => i * 1048576 + j;
  segs.forEach(([ax, ay, bx, by], k) => {
    for (let i = Math.floor(Math.min(ax, bx) / G); i <= Math.floor(Math.max(ax, bx) / G); i++)
      for (let j = Math.floor(Math.min(ay, by) / G); j <= Math.floor(Math.max(ay, by) / G); j++) {
        const key = gk(i, j); let b = egrid.get(key); if (!b) egrid.set(key, b = []); b.push(k);
      }
  });
  const stamp = new Int32Array(segs.length); let stampNo = 0;
  const ptSeg = (px, py, ax, ay, bx, by) => {
    const dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy;
    const t = L2 ? Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / L2)) : 0;
    return Math.hypot(px - ax - t * dx, py - ay - t * dy);
  };
  const cross = (ax, ay, bx, by, cx, cy) => (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
  const segSeg = (p, q, e) => {
    const d1 = cross(p[0], p[1], q[0], q[1], e[0], e[1]), d2 = cross(p[0], p[1], q[0], q[1], e[2], e[3]);
    const d3 = cross(e[0], e[1], e[2], e[3], p[0], p[1]), d4 = cross(e[0], e[1], e[2], e[3], q[0], q[1]);
    if (((d1 > 0) !== (d2 > 0)) && ((d3 > 0) !== (d4 > 0))) return 0;
    return Math.min(ptSeg(p[0], p[1], e[0], e[1], e[2], e[3]), ptSeg(q[0], q[1], e[0], e[1], e[2], e[3]),
      ptSeg(e[0], e[1], p[0], p[1], q[0], q[1]), ptSeg(e[2], e[3], p[0], p[1], q[0], q[1]));
  };
  // is the segment p→q at least minD from every outline segment?
  const clearSeg = (p, q, minD) => {
    stampNo++;
    for (let i = Math.floor((Math.min(p[0], q[0]) - minD) / G); i <= Math.floor((Math.max(p[0], q[0]) + minD) / G); i++)
      for (let j = Math.floor((Math.min(p[1], q[1]) - minD) / G); j <= Math.floor((Math.max(p[1], q[1]) + minD) / G); j++) {
        const b = egrid.get(gk(i, j)); if (!b) continue;
        for (const k of b) { if (stamp[k] === stampNo) continue; stamp[k] = stampNo; if (segSeg(p, q, segs[k]) < minD) return false; }
      }
    return true;
  };
  const pointRoom = (p, maxR) => {
    stampNo++;
    let best = maxR;
    for (let i = Math.floor((p[0] - maxR) / G); i <= Math.floor((p[0] + maxR) / G); i++)
      for (let j = Math.floor((p[1] - maxR) / G); j <= Math.floor((p[1] + maxR) / G); j++) {
        const b = egrid.get(gk(i, j)); if (!b) continue;
        for (const k of b) { if (stamp[k] === stampNo) continue; stamp[k] = stampNo; const e = segs[k]; best = Math.min(best, ptSeg(p[0], p[1], e[0], e[1], e[2], e[3])); }
      }
    return best;
  };
  const C = clearance * s;
  const node = (i, j) => [gx0 + (i + 0.5) * s, gy0 + (j + 0.5) * s];

  // coarse cells that fit: inside, and all four sides of the cell's node square clear
  const inside = new Uint8Array(nCx * nCy);
  for (let cy = 0; cy < nCy; cy++) for (let cx = 0; cx < nCx; cx++) {
    const a = node(2 * cx, 2 * cy), b = node(2 * cx + 1, 2 * cy), c = node(2 * cx + 1, 2 * cy + 1), d = node(2 * cx, 2 * cy + 1);
    if (insideAt(a[0], a[1]) && clearSeg(a, b, C) && clearSeg(b, c, C) && clearSeg(c, d, C) && clearSeg(d, a, C)) inside[cy * nCx + cx] = 1;
  }

  // random spanning forest (Kruskal over shuffled edges); an edge needs its corridor clear too
  const rand = mulberry32(seed);
  const edges = [];
  for (let cy = 0; cy < nCy; cy++) for (let cx = 0; cx < nCx; cx++) {
    const k = cy * nCx + cx; if (!inside[k]) continue;
    if (cx + 1 < nCx && inside[k + 1] && clearSeg(node(2 * cx + 1, 2 * cy), node(2 * cx + 2, 2 * cy), C) && clearSeg(node(2 * cx + 1, 2 * cy + 1), node(2 * cx + 2, 2 * cy + 1), C)) edges.push([k, k + 1]);
    if (cy + 1 < nCy && inside[k + nCx] && clearSeg(node(2 * cx, 2 * cy + 1), node(2 * cx, 2 * cy + 2), C) && clearSeg(node(2 * cx + 1, 2 * cy + 1), node(2 * cx + 1, 2 * cy + 2), C)) edges.push([k, k + nCx]);
  }
  for (let i = edges.length - 1; i > 0; i--) { const j = Math.floor(rand() * (i + 1)); [edges[i], edges[j]] = [edges[j], edges[i]]; }
  const parent = new Int32Array(nCx * nCy).map((_, i) => i);
  const find = (x) => { while (parent[x] !== x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; };
  const E = new Uint8Array(nCx * nCy), N = new Uint8Array(nCx * nCy); // tree edge to east / north
  for (const [a, b] of edges) {
    const ra = find(a), rb = find(b);
    if (ra === rb) continue;
    parent[ra] = rb;
    if (b === a + 1) E[a] = 1; else N[a] = 1;
  }

  // fine graph: each fine node gets exactly two links
  const FW = 2 * nCx, FH = 2 * nCy;
  const links = new Int32Array(FW * FH * 2).fill(-1);
  const id = (i, j) => j * FW + i;
  const link = (u, v) => {
    links[u * 2 + (links[u * 2] < 0 ? 0 : 1)] = v;
    links[v * 2 + (links[v * 2] < 0 ? 0 : 1)] = u;
  };
  for (let cy = 0; cy < nCy; cy++) for (let cx = 0; cx < nCx; cx++) {
    const k = cy * nCx + cx; if (!inside[k]) continue;
    const a = id(2 * cx, 2 * cy), b = id(2 * cx + 1, 2 * cy), c = id(2 * cx + 1, 2 * cy + 1), d = id(2 * cx, 2 * cy + 1);
    const S = cy > 0 && N[k - nCx], Wt = cx > 0 && E[k - 1];
    if (!S) link(a, b);
    if (!E[k]) link(b, c); else { link(b, id(2 * cx + 2, 2 * cy)); link(c, id(2 * cx + 2, 2 * cy + 1)); }
    if (!N[k]) link(d, c); else { link(d, id(2 * cx, 2 * cy + 2)); link(c, id(2 * cx + 1, 2 * cy + 2)); }
    if (!Wt) link(a, d);
  }

  // walk each cycle (as grid node indices)
  const seen = new Uint8Array(FW * FH);
  let cycles = [];
  for (let start = 0; start < FW * FH; start++) {
    if (seen[start] || links[start * 2] < 0) continue;
    const cyc = [];
    let prev = -1, cur = start;
    while (cur >= 0 && !seen[cur]) {
      seen[cur] = 1;
      cyc.push(cur);
      const n0 = links[cur * 2], n1 = links[cur * 2 + 1];
      const next = n0 !== prev ? n0 : n1;
      prev = cur; cur = next;
    }
    if (cyc.length >= 4) cycles.push(cyc);
  }

  // bridge separate cycles across narrow necks: where two cycles run side by
  // side (parallel unit edges 1–3 grid steps apart, clear of the boundary),
  // swap those two edges for two short rungs. The result is one cycle.
  const bridgeOk = (u, v) => clearSeg(node(u % FW, Math.floor(u / FW)), node(v % FW, Math.floor(v / FW)), 0.4 * s);
  for (let guard = 0; cycles.length > 1 && guard < 10000; guard++) {
    const owner = new Int32Array(FW * FH).fill(-1), pos = new Int32Array(FW * FH);
    cycles.forEach((c, ci) => c.forEach((n, k) => { owner[n] = ci; pos[n] = k; }));
    let merged = false;
    for (let ai = 0; ai < cycles.length && !merged; ai++) {
      const A = cycles[ai];
      for (let k = 0; k < A.length && !merged; k++) {
        const u1 = A[k], u2 = A[(k + 1) % A.length];
        const i1 = u1 % FW, j1 = Math.floor(u1 / FW), i2 = u2 % FW, j2 = Math.floor(u2 / FW);
        const dx = i2 - i1, dy = j2 - j1;
        for (const sgn of [1, -1]) {
          for (let step = 1; step <= 3 && !merged; step++) {
            const ox_ = -dy * sgn * step, oy_ = dx * sgn * step; // perpendicular offset
            const a1 = i1 + ox_, b1 = j1 + oy_, a2 = i2 + ox_, b2 = j2 + oy_;
            if (a1 < 0 || b1 < 0 || a2 < 0 || b2 < 0 || a1 >= FW || a2 >= FW || b1 >= FH || b2 >= FH) break;
            const v1 = b1 * FW + a1, v2 = b2 * FW + a2;
            const bi = owner[v1];
            if (bi < 0) continue;
            if (bi === ai) break; // our own line is in the way
            if (owner[v2] !== bi) break;
            let B = cycles[bi];
            const p1 = pos[v1], p2 = pos[v2];
            const fwd = (p1 + 1) % B.length === p2, back = (p2 + 1) % B.length === p1;
            if (!fwd && !back) break;
            if (!bridgeOk(u1, v1) || !bridgeOk(u2, v2)) break;
            // A rotated to run u2 … u1; B arranged to run v1 … v2 the long way
            const Arot = A.slice(k + 1).concat(A.slice(0, k + 1));
            let Bp;
            if (fwd) { const rot = B.slice(p2).concat(B.slice(0, p2)); Bp = rot.reverse(); }
            else { Bp = B.slice(p1).concat(B.slice(0, p1)); }
            const joined = Arot.concat(Bp);
            cycles = cycles.filter((_, ci) => ci !== ai && ci !== bi);
            cycles.push(joined);
            merged = true;
          }
          if (merged) break;
        }
      }
    }
    if (!merged) break;
  }

  const loops = [];
  let nodes = 0;
  for (const cyc of cycles) {
    const ring = cyc.map((n) => node(n % FW, Math.floor(n / FW)));
    nodes += ring.length;
    // nudge each node a little (organic wobble), then round the corners;
    // every grid node is kept so corners are cut at most spacing/4 wide
    // near the outline the nudge shrinks so the line keeps ≥ 0.35×spacing clearance
    if (wobble > 0) for (const p of ring) {
      const room = pointRoom(p, 2 * s) - 0.4 * s;
      const amp = Math.max(0, Math.min(wobble * s, room / Math.SQRT2));
      p[0] += (rand() - 0.5) * 2 * amp; p[1] += (rand() - 0.5) * 2 * amp;
    }
    const sm = smooth > 0 ? chaikinClosed(ring, smooth) : ring;
    const out = angle ? sm.map(toPage) : sm;
    out.push(out[0].slice());
    loops.push(out);
  }
  return { loops, nodes };
}
