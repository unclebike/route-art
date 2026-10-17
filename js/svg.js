// SVG → polylines in root user units, y flipped up. Uses the browser's own
// geometry engine (getPointAtLength + CTMs), so every path command, arc and
// transform is handled exactly as the browser renders it.

const SKIP = new Set(['defs', 'clipPath', 'mask', 'symbol', 'marker', 'pattern', 'title', 'desc', 'metadata', 'style', 'script', 'linearGradient', 'radialGradient', 'filter', 'foreignObject', 'image']);
const GEOM = new Set(['path', 'line', 'polyline', 'polygon', 'rect', 'circle', 'ellipse']);

export function parseSVG(text) {
  const doc = new DOMParser().parseFromString(text, 'image/svg+xml');
  const err = doc.querySelector('parsererror');
  if (err || !doc.documentElement || doc.documentElement.localName !== 'svg') throw new Error('This file is not a readable SVG.');
  const host = document.createElement('div');
  host.style.cssText = 'position:fixed;left:-20000px;top:0;width:1000px;height:1000px;visibility:hidden;pointer-events:none;overflow:hidden';
  const root = document.importNode(doc.documentElement, true);
  // give the root a concrete size so its CTM is never singular
  root.setAttribute('width', '1000');
  root.setAttribute('height', '1000');
  host.appendChild(root);
  document.body.appendChild(host);
  const warnings = new Set();
  const paths = [];
  const layers = new Set();
  try {
    const rootInv = root.getScreenCTM().inverse();
    const bb = root.getBBox();
    const diagUser = Math.hypot(bb.width, bb.height) || 1;
    const visit = (el, layer) => {
      for (const ch of el.children) {
        const tag = ch.localName;
        if (SKIP.has(tag)) continue;
        if (tag === 'text') { warnings.add('Ignored text (convert text to outlines/paths first)'); continue; }
        if (tag === 'use') { warnings.add('Ignored <use> clones (expand them first)'); continue; }
        if (ch.hasAttribute('clip-path') || ch.hasAttribute('mask')) warnings.add('Clipping masks are not applied: the full shapes are used');
        let lay = layer;
        if (tag === 'g') {
          const id = ch.getAttribute('inkscape:label') || ch.getAttribute('id');
          if (!layer && id) lay = id;
        }
        let disp = 'inline';
        try { disp = getComputedStyle(ch).display; } catch { /* noop */ }
        if (disp === 'none') continue;
        if (GEOM.has(tag)) {
          const m = rootInv.multiply(ch.getScreenCTM());
          const out = sample(ch, tag, m, diagUser);
          for (const pts of out) {
            if (pts.length >= 2) {
              paths.push({ layer: lay || 'Drawing', pts: pts.map(([x, y]) => [x, -y]), closed: false });
              layers.add(lay || 'Drawing');
            }
          }
        } else if (ch.children.length) visit(ch, lay);
      }
    };
    visit(root, null);
  } finally {
    host.remove();
  }
  return { paths, layers: [...layers], units: null, warnings: [...warnings] };
}

function xf(m, x, y) { return [m.a * x + m.c * y + m.e, m.b * x + m.d * y + m.f]; }

function sample(el, tag, m, diagUser) {
  const num = (a) => parseFloat(el.getAttribute(a) || '0');
  if (tag === 'line') return [[xf(m, num('x1'), num('y1')), xf(m, num('x2'), num('y2'))]];
  if (tag === 'polyline' || tag === 'polygon') {
    const v = (el.getAttribute('points') || '').trim().split(/[\s,]+/).map(parseFloat).filter((n) => !Number.isNaN(n));
    const pts = [];
    for (let i = 0; i + 1 < v.length; i += 2) pts.push(xf(m, v[i], v[i + 1]));
    if (tag === 'polygon' && pts.length > 2) pts.push(pts[0]);
    return [pts];
  }
  if (tag === 'rect' && !el.getAttribute('rx') && !el.getAttribute('ry')) {
    const x = num('x'), y = num('y'), w = num('width'), h = num('height');
    if (w <= 0 || h <= 0) return [];
    return [[[x, y], [x + w, y], [x + w, y + h], [x, y + h], [x, y]].map(([a, b]) => xf(m, a, b))];
  }
  // generic: walk the outline with getPointAtLength, splitting at move-tos
  let L = 0;
  try { L = el.getTotalLength(); } catch { return []; }
  if (!(L > 0)) return [];
  // local scale of the element so the step is fine in root units
  const scale = Math.sqrt(Math.abs(m.a * m.d - m.b * m.c)) || 1;
  const step = Math.max(L / 200000, diagUser / 6000 / scale);
  const n = Math.max(8, Math.ceil(L / step));
  const ds = L / n;
  const P = (l) => { const p = el.getPointAtLength(Math.min(L, Math.max(0, l))); return [p.x, p.y]; };
  const out = [];
  let cur = [];
  let prevL = 0, prev = P(0);
  cur.push(prev);
  for (let i = 1; i <= n; i++) {
    const l = i === n ? L : i * ds;
    const p = P(l);
    const gap = Math.hypot(p[0] - prev[0], p[1] - prev[1]);
    if (gap > (l - prevL) * 1.5 + 1e-9 * diagUser) {
      // a move-to lies in (prevL, l]: binary-search the jump
      let lo = prevL, hi = l, plo = prev;
      for (let k = 0; k < 40 && hi - lo > ds * 1e-6; k++) {
        const mid = (lo + hi) / 2, pm = P(mid);
        if (Math.hypot(pm[0] - plo[0], pm[1] - plo[1]) > (mid - lo) * 1.5 + 1e-12) hi = mid; else { lo = mid; plo = pm; }
      }
      cur.push(plo);
      out.push(cur);
      cur = [P(hi)];
    }
    cur.push(p);
    prev = p; prevL = l;
  }
  out.push(cur);
  return out.map((pts) => pts.map(([x, y]) => xf(m, x, y)));
}
