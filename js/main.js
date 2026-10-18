import { parseDXF } from './dxf.js';
import { parseSVG } from './svg.js';
import { order, exportRoutes, exportGroups, makePlacer, rotatedBox, merc, unmerc, haversineKm, autoToleranceM } from './core.js';
import { buildShapes } from './shapes.js';
import { planGroups } from './plan.js';
import { squiggleFill } from './fill.js';

// Test/automation hook
const spacingFor = (paths, density) => {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const p of paths) for (const [x, y] of p.pts) { if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y; }
  return Math.sqrt(Math.max(1e-12, (x1 - x0) * (y1 - y0))) / Math.max(1, density);
};
const prepare = (paths, o = {}) => planGroups(buildShapes(paths), paths, { mode: 'outline', grouping: 'join', seed: 1, wobble: 0.22, ...o, spacing: o.spacing || spacingFor(paths, o.density || 90) });
window.RWGPSConv = { parseDXF, parseSVG, order, exportRoutes, exportGroups, buildShapes, squiggleFill, prepare, ready: true };

const $ = (id) => document.getElementById(id);
const ROUTE_COLORS = ['#e8742c', '#4f7a28', '#2a7f8f', '#8a4f9e', '#b8860b', '#3f63b5', '#b33a3a', '#6b8e23'];
const STORE = 'route-art:v1';
const store = {
  get() { try { return JSON.parse(localStorage.getItem(STORE) || '{}'); } catch { return {}; } },
  set(v) { try { localStorage.setItem(STORE, JSON.stringify({ ...store.get(), ...v })); } catch { /* private mode */ } },
};

const SAMPLE_SVG = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 240 150">
<circle cx="55" cy="100" r="40" fill="none" stroke="#000"/>
<circle cx="185" cy="100" r="40" fill="none" stroke="#000"/>
<polyline points="55,100 100,100 85,45 160,45 100,100" fill="none" stroke="#000"/>
<polyline points="160,45 185,100" fill="none" stroke="#000"/>
<path d="M160,45 L152,22 Q160,14 176,20" fill="none" stroke="#000"/>
<path d="M85,45 L80,32 M68,30 L94,30" fill="none" stroke="#000"/>
<path d="M55,100 L85,45" fill="none" stroke="#000"/>
</svg>`;

const S = {
  seed: 1, frame: null, built: null, planKey: '', spacing: 0,
  fileName: null, kind: null, parsed: null, layerOn: new Set(),
  ordered: null, box: null, boxRot: null, preview: null,
  place: null, // { tl:[lat,lon], br:[lat,lon], rot }
  result: null, sample: false,
};

// ---------- map ----------
const darkMQ = matchMedia('(prefers-color-scheme: dark)');
const styleUrl = () => `https://tiles.openfreemap.org/styles/${darkMQ.matches ? 'dark' : 'positron'}`;
const saved = store.get();
const map = new maplibregl.Map({
  container: 'map', style: styleUrl(),
  center: saved.view?.center || [-77.383, 44.163], zoom: saved.view?.zoom ?? 9.5,
  attributionControl: { compact: true }, dragRotate: false, pitchWithRotate: false,
});
map.addControl(new maplibregl.NavigationControl({ showCompass: false }), 'top-right');
map.addControl(new maplibregl.ScaleControl({ unit: 'metric' }), 'bottom-right');
map.touchZoomRotate.disableRotation();
darkMQ.addEventListener('change', () => map.setStyle(styleUrl()));
map.on('moveend', () => store.set({ view: { center: map.getCenter().toArray(), zoom: map.getZoom() } }));

const emptyFC = { type: 'FeatureCollection', features: [] };
function ensureLayers() {
  if (map.getSource('routes')) return;
  map.addSource('box', { type: 'geojson', data: emptyFC });
  map.addSource('preview', { type: 'geojson', data: emptyFC });
  map.addSource('routes', { type: 'geojson', data: emptyFC });
  map.addLayer({ id: 'box', type: 'line', source: 'box', paint: { 'line-color': '#f2833c', 'line-width': 1.5, 'line-dasharray': [3, 2] } });
  map.addLayer({ id: 'routes', type: 'line', source: 'routes', layout: { 'line-join': 'round', 'line-cap': 'round' }, paint: { 'line-color': ['get', 'color'], 'line-width': ['interpolate', ['linear'], ['zoom'], 4, 1, 12, 2.5] } });
  map.addLayer({ id: 'preview', type: 'line', source: 'preview', layout: { 'line-join': 'round' }, paint: { 'line-color': darkMQ.matches ? '#f5e9d8' : '#22271d', 'line-width': 1, 'line-opacity': 0.85 } });
  drawAll();
}
map.on('style.load', ensureLayers);
const setData = (id, data) => { const s = map.getSource(id); if (s) s.setData(data); };

// handles
function handle(cls, label) {
  const el = document.createElement('div');
  el.className = `handle ${cls}`;
  el.setAttribute('aria-label', label);
  el.title = label;
  return new maplibregl.Marker({ element: el, draggable: true });
}
const hTL = handle('corner', 'Top-left corner'), hBR = handle('corner', 'Bottom-right corner'), hMove = handle('move', 'Move drawing');
let handlesOn = false;

// ---------- placement maths (Web Mercator metres) ----------
const toM = ([lat, lon]) => merc(lon, lat);
const toLL = ([X, Y]) => { const [lon, lat] = unmerc(X, Y); return [lat, lon]; };
const fmtLL = ([lat, lon]) => `${lat.toFixed(5)}, ${lon.toFixed(5)}`;
function parseLL(s) {
  const m = String(s).replace(/[()°]/g, ' ').match(/(-?\d+(?:\.\d+)?)\s*[,\s]\s*(-?\d+(?:\.\d+)?)/);
  if (!m) return null;
  let a = parseFloat(m[1]), b = parseFloat(m[2]);
  if (Math.abs(a) > 90 && Math.abs(b) <= 90) [a, b] = [b, a]; // pasted as lon, lat
  if (Math.abs(a) > 85 || Math.abs(b) > 180) return null;
  return [a, b];
}
function getBox() {
  if (!S.ordered) return null;
  const rot = S.place ? S.place.rot : 0;
  if (!S.box || S.boxRot !== rot) { S.box = rotatedBox(S.frame || S.ordered.curves, rot); S.boxRot = rot; }
  return S.box;
}
const aspect = () => { const b = getBox(); return b ? b.w / (b.h || 1) : 1; };

function fitToView(frac = 0.7) {
  const b = map.getBounds();
  const [Xa, Ya] = merc(b.getWest(), b.getNorth());
  const [Xb, Yb] = merc(b.getEast(), b.getSouth());
  const vw = (Xb - Xa) * frac, vh = (Ya - Yb) * frac;
  const A = aspect();
  let W = vw, H = vw / A;
  if (H > vh) { H = vh; W = vh * A; }
  const cx = (Xa + Xb) / 2, cy = (Ya + Yb) / 2;
  setPlace({ tl: toLL([cx - W / 2, cy + H / 2]), br: toLL([cx + W / 2, cy - H / 2]), rot: S.place?.rot || 0 }, { exportNow: true });
}

function setPlace(p, { exportNow = false, fromInput = false } = {}) {
  S.place = p;
  if (!fromInput) { $('tl').value = fmtLL(p.tl); $('br').value = fmtLL(p.br); $('tl').removeAttribute('aria-invalid'); $('br').removeAttribute('aria-invalid'); }
  $('rot').value = p.rot; $('rotRange').value = p.rot;
  store.set({ place: p });
  drawPlacement();
  if (exportNow) scheduleExport(0); else scheduleExport();
}

// ---------- drawing ----------
let raf = 0;
function drawPlacement() {
  if (raf) return;
  raf = requestAnimationFrame(() => { raf = 0; drawAll(); });
}
function drawAll() {
  if (!map.getSource('routes')) return;
  const p = S.place;
  if (!p || !S.ordered) { setData('box', emptyFC); setData('preview', emptyFC); setData('routes', emptyFC); showHandles(false); return; }
  const [X1, Y1] = toM(p.tl), [X2, Y2] = toM(p.br);
  const ring = [[X1, Y1], [X2, Y1], [X2, Y2], [X1, Y2], [X1, Y1]].map(([X, Y]) => unmerc(X, Y));
  setData('box', { type: 'Feature', geometry: { type: 'LineString', coordinates: ring } });
  const fresh = S.result && S.result.key === exportKey();
  if (fresh) {
    setData('preview', emptyFC);
  } else {
    const placer = makePlacer(null, { tl: p.tl, br: p.br, rotationDeg: p.rot, box: getBox() });
    const coords = [];
    for (const c of S.preview) for (const [x, y] of c) { const [X, Y] = placer.toMerc(x, y); coords.push(unmerc(X, Y)); }
    setData('preview', { type: 'Feature', geometry: { type: 'LineString', coordinates: coords } });
  }
  hTL.setLngLat([p.tl[1], p.tl[0]]);
  hBR.setLngLat([p.br[1], p.br[0]]);
  hMove.setLngLat(unmerc((X1 + X2) / 2, (Y1 + Y2) / 2));
  showHandles(true);
  updateSize();
  if (S.parsed) updateDensityLabel();
}
function showHandles(on) {
  if (on === handlesOn) return;
  handlesOn = on;
  for (const h of [hTL, hBR, hMove]) on ? h.addTo(map) : h.remove();
}
function updateSize() {
  const p = S.place;
  if (!p) { $('size').textContent = ''; return; }
  const midLat = (p.tl[0] + p.br[0]) / 2;
  const wKm = haversineKm(p.tl[1], midLat, p.br[1], midLat);
  const hKm = haversineKm(p.tl[1], p.tl[0], p.tl[1], p.br[0]);
  const [X1, Y1] = toM(p.tl), [X2, Y2] = toM(p.br);
  const boxA = (X2 - X1) / ((Y1 - Y2) || 1), A = aspect();
  const stretch = boxA / A - 1;
  let s = `${fmtKm(wKm)} wide × ${fmtKm(hKm)} tall`;
  if (Math.abs(stretch) > 0.01) s += ` · stretched ${Math.round(Math.abs(stretch) * 100)}% ${stretch > 0 ? 'wider' : 'taller'}`;
  $('size').textContent = s;
}
const fmtKm = (km) => (km >= 100 ? `${Math.round(km).toLocaleString()} km` : km >= 1 ? `${km.toFixed(1)} km` : `${Math.round(km * 1000)} m`);

// handle drags
const MIN = 20; // metres
hTL.on('drag', () => {
  const ll = hTL.getLngLat();
  const [X2, Y2] = toM(S.place.br);
  let [Xa, Ya] = merc(ll.lng, ll.lat);
  let W = Math.max(MIN, X2 - Xa), H = Math.max(MIN, Ya - Y2);
  if ($('keep').checked) { const A = aspect(); W = Math.max(W, H * A); H = W / A; }
  setPlace({ ...S.place, tl: toLL([X2 - W, Y2 + H]) });
});
hBR.on('drag', () => {
  const ll = hBR.getLngLat();
  const [X1, Y1] = toM(S.place.tl);
  let [Xb, Yb] = merc(ll.lng, ll.lat);
  let W = Math.max(MIN, Xb - X1), H = Math.max(MIN, Y1 - Yb);
  if ($('keep').checked) { const A = aspect(); W = Math.max(W, H * A); H = W / A; }
  setPlace({ ...S.place, br: toLL([X1 + W, Y1 - H]) });
});
hMove.on('drag', () => {
  const ll = hMove.getLngLat();
  const [cx, cy] = merc(ll.lng, ll.lat);
  const [X1, Y1] = toM(S.place.tl), [X2, Y2] = toM(S.place.br);
  const W = X2 - X1, H = Y1 - Y2;
  setPlace({ ...S.place, tl: toLL([cx - W / 2, cy + H / 2]), br: toLL([cx + W / 2, cy - H / 2]) });
});
for (const h of [hTL, hBR, hMove]) h.on('dragend', () => { drawAll(); scheduleExport(0); });

// ---------- inputs ----------
function onCornerInput() {
  const tl = parseLL($('tl').value), br = parseLL($('br').value);
  $('tl').setAttribute('aria-invalid', String(!tl));
  $('br').setAttribute('aria-invalid', String(!br));
  if (!tl || !br || !S.ordered) return;
  if (!(tl[0] > br[0]) || !(tl[1] < br[1])) { $('size').textContent = 'Top-left must be north-west of bottom-right.'; return; }
  setPlace({ tl, br, rot: S.place?.rot || 0 }, { fromInput: true });
}
$('tl').addEventListener('input', onCornerInput);
$('br').addEventListener('input', onCornerInput);
$('tl').addEventListener('change', () => { if (S.place) $('tl').value = fmtLL(S.place.tl); });
$('br').addEventListener('change', () => { if (S.place) $('br').value = fmtLL(S.place.br); });

function setRotation(deg) {
  deg = Math.max(-180, Math.min(180, Number(deg) || 0));
  if (!S.place || !S.ordered) { $('rot').value = deg; $('rotRange').value = deg; return; }
  // keep the drawing's scale and centre; the bounding box reshapes around it
  const old = getBox();
  const [X1, Y1] = toM(S.place.tl), [X2, Y2] = toM(S.place.br);
  const sx = (X2 - X1) / old.w, sy = (Y1 - Y2) / old.h;
  S.place = { ...S.place, rot: deg };
  const nb = getBox();
  const cx = (X1 + X2) / 2, cy = (Y1 + Y2) / 2, W = nb.w * sx, H = nb.h * sy;
  setPlace({ tl: toLL([cx - W / 2, cy + H / 2]), br: toLL([cx + W / 2, cy - H / 2]), rot: deg });
}
$('rotRange').addEventListener('input', (e) => setRotation(e.target.value));
$('rot').addEventListener('change', (e) => setRotation(e.target.value));
$('fitView').addEventListener('click', () => S.ordered && fitToView());
$('zoomTo').addEventListener('click', zoomToPlace);
function zoomToPlace() {
  if (!S.place) return;
  map.fitBounds([[S.place.tl[1], S.place.br[0]], [S.place.br[1], S.place.tl[0]]], { padding: 60, duration: 600 });
}
$('keep').addEventListener('change', () => store.set({ keep: $('keep').checked }));
for (const id of ['tol', 'maxPts', 'maxKm', 'fmt', 'rname']) {
  $(id).addEventListener('input', () => { saveSettings(); scheduleExport(); });
}
function saveSettings() { store.set({ tol: $('tol').value, maxPts: $('maxPts').value, maxKm: $('maxKm').value, fmt: $('fmt').value }); }
if (saved.tol != null) $('tol').value = saved.tol;
if (saved.maxPts) $('maxPts').value = saved.maxPts;
if (saved.maxKm != null) $('maxKm').value = saved.maxKm;
if (saved.fmt) $('fmt').value = saved.fmt;
if (saved.keep === false) $('keep').checked = false;

// ---------- export ----------
const exportKey = () => JSON.stringify([S.place, $('tol').value, $('maxPts').value, $('maxKm').value, $('rname').value, S.fileName, [...S.layerOn], S.planKey]);
let expTimer = 0;
function scheduleExport(delay = 350) {
  clearTimeout(expTimer);
  expTimer = setTimeout(runExport, delay);
}
function runExport() {
  if (!S.ordered || !S.place || !S.ordered.curves.length) return;
  const key = exportKey();
  if (S.result && S.result.key === key) return;
  busy('Building routes…');
  setTimeout(() => {
    try {
      const res = exportGroups(S.ordered, {
        tl: S.place.tl, br: S.place.br, rotationDeg: S.place.rot, box: getBox(),
        toleranceM: parseFloat($('tol').value) || 0,
        maxPoints: Math.max(200, parseInt($('maxPts').value, 10) || 6000),
        maxKm: parseFloat($('maxKm').value) || 0,
        name: $('rname').value.trim() || 'Route',
      });
      res.key = key;
      S.result = res;
      renderRoutes();
      drawAll();
    } catch (e) {
      console.error(e);
      $('summary').textContent = `Couldn’t build routes: ${e.message}`;
    } finally { busy(false); }
  }, 16);
}

function renderRoutes() {
  const r = S.result;
  const list = $('routes');
  list.textContent = '';
  if (!r || !r.routes.length) { $('summary').textContent = ''; $('dlAll').disabled = true; setData('routes', emptyFC); return; }
  $('tol').placeholder = `auto (${autoToleranceM(r.widthKm)} m)`;
  const pts = r.routes.reduce((a, x) => a + x.points, 0);
  $('summary').innerHTML = '';
  const line = (label, value) => { const d = document.createElement('div'); d.append(`${label} `); const s = document.createElement('strong'); s.textContent = value; d.append(s); $('summary').append(d); };
  line(r.routes.length === 1 ? 'One route,' : `${r.routes.length} routes,`, `${fmtKm(r.totalKm)} total`);
  line('Points:', `${pts.toLocaleString()} (smoothing ${r.toleranceM} m)`);
  const feats = [];
  r.routes.forEach((rt, i) => {
    const color = ROUTE_COLORS[i % ROUTE_COLORS.length];
    feats.push({ type: 'Feature', properties: { color }, geometry: { type: 'LineString', coordinates: rt.coords } });
    const li = document.createElement('li');
    const sw = document.createElement('span'); sw.className = 'sw'; sw.style.background = color;
    const mid = document.createElement('div');
    const nm = document.createElement('div'); nm.className = 'rn'; nm.textContent = rt.name;
    const meta = document.createElement('div'); meta.className = 'rm';
    meta.textContent = `${fmtKm(rt.km)} · ${rt.points.toLocaleString()} pts`;
    if (rt.maxConnectorKm > Math.max(0.05 * rt.km / Math.max(1, rt.points / 100), 0.5) && rt.maxConnectorKm > 0.02 * r.widthKm) {
      const w = document.createElement('span'); w.className = 'warnc'; w.textContent = ` · longest straight join ${fmtKm(rt.maxConnectorKm)}`; meta.append(w);
    }
    mid.append(nm, meta);
    const b = document.createElement('button'); b.type = 'button'; b.className = 'btn small'; b.textContent = $('fmt').value.toUpperCase();
    b.setAttribute('aria-label', `Download ${rt.name}`);
    b.addEventListener('click', () => downloadRoute(rt));
    li.append(sw, mid, b);
    list.append(li);
  });
  setData('routes', { type: 'FeatureCollection', features: feats });
  $('dlAll').disabled = false;
  $('dlAll').textContent = r.routes.length === 1 ? `Download ${$('fmt').value.toUpperCase()}` : `Download all ${r.routes.length} (.zip)`;
}

const slug = (s) => (s || 'route').toLowerCase().replace(/[^a-z0-9]+/g, '-').replace(/^-|-$/g, '') || 'route';
function fileFor(rt) {
  const fmt = $('fmt').value;
  const base = slug($('rname').value);
  const n = S.result.routes.length;
  const name = n > 1 ? `${base}-${String(rt.index).padStart(String(n).length, '0')}.${fmt}` : `${base}.${fmt}`;
  return { name, data: fmt === 'gpx' ? rt.gpx : rt.kml, mime: fmt === 'gpx' ? 'application/gpx+xml' : 'application/vnd.google-earth.kml+xml' };
}
function save(name, blob) {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  document.body.append(a);
  a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 2000);
}
function downloadRoute(rt) { const f = fileFor(rt); save(f.name, new Blob([f.data], { type: f.mime })); }
$('dlAll').addEventListener('click', async () => {
  const r = S.result; if (!r) return;
  if (r.routes.length === 1) return downloadRoute(r.routes[0]);
  busy('Zipping…');
  try {
    const zip = new JSZip();
    for (const rt of r.routes) { const f = fileFor(rt); zip.file(f.name, f.data); }
    save(`${slug($('rname').value)}-routes.zip`, await zip.generateAsync({ type: 'blob', compression: 'DEFLATE' }));
  } finally { busy(false); }
});
$('fmt').addEventListener('change', () => { if (S.result) renderRoutes(); });

// ---------- loading ----------
function busy(msg) { $('busy').hidden = !msg; if (msg) $('busyText').textContent = msg; }

let worker = null, reqId = 0;
function planAsync(built, rawPaths, opts) {
  try { worker = worker || new Worker(new URL('./worker.js', import.meta.url), { type: 'module' }); } catch { worker = null; }
  const local = () => planGroups(built, rawPaths, opts);
  if (!worker) return Promise.resolve(local());
  const id = ++reqId;
  return new Promise((resolve, reject) => {
    const onMsg = (e) => {
      if (e.data.id !== id) return;
      worker.removeEventListener('message', onMsg);
      e.data.ok ? resolve(e.data.prepared) : reject(new Error(e.data.error));
    };
    worker.addEventListener('message', onMsg);
    worker.addEventListener('error', () => { worker = null; resolve(local()); }, { once: true });
    worker.postMessage({ id, built: { shapes: built.shapes, open: built.open.map((p) => ({ pts: p.pts })) }, rawPaths: rawPaths.map((p) => ({ pts: p.pts })), opts });
  });
}

async function loadText(name, text, { sample = false } = {}) {
  const lower = name.toLowerCase();
  let kind = lower.endsWith('.svg') ? 'svg' : lower.endsWith('.dxf') ? 'dxf' : null;
  if (!kind) kind = /<svg[\s>]/i.test(text.slice(0, 4000)) ? 'svg' : /SECTION/.test(text.slice(0, 2000)) ? 'dxf' : null;
  if (text.startsWith('AutoCAD Binary DXF')) return fail(name, 'This is a binary DXF. Save it as an ASCII DXF and try again.');
  if (!kind) return fail(name, 'That file isn’t a DXF or SVG.');
  busy('Reading drawing…');
  await new Promise((r) => setTimeout(r, 16));
  let parsed;
  try { parsed = kind === 'svg' ? parseSVG(text) : parseDXF(text); }
  catch (e) { busy(false); return fail(name, e.message || 'The file couldn’t be read.'); }
  if (!parsed.paths.length) { busy(false); return fail(name, 'No lines or curves were found in this drawing.'); }
  S.fileName = name; S.kind = kind; S.parsed = parsed; S.sample = sample;
  S.layerOn = new Set(parsed.layers);
  $('rname').value = sample ? 'Sample bike' : name.replace(/\.(dxf|svg)$/i, '');
  renderFile();
  await reorder({ refit: true });
}
function fail(name, msg) {
  busy(false);
  $('filecard').hidden = false;
  $('fname').textContent = name; $('ftag').textContent = 'error';
  $('fstats').textContent = '';
  $('fwarn').innerHTML = '';
  const li = document.createElement('li'); li.textContent = msg; $('fwarn').append(li);
  $('layers').hidden = true;
}

const styleOpts = () => ({
  mode: document.querySelector('input[name="mode"]:checked')?.value || 'outline',
  grouping: $('grouping').value,
  density: +$('density').value,
  wobble: +$('wobble').value / 100,
  seed: S.seed,
});
let planToken = 0;
async function reorder({ refit = false } = {}) {
  const paths = S.parsed.paths.filter((p) => S.layerOn.has(p.layer));
  if (!paths.length) { S.ordered = null; S.result = null; renderRoutes(); drawAll(); return; }
  const token = ++planToken;
  S.frame = paths.map((p) => p.pts);
  S.built = buildShapes(paths);
  const o = styleOpts();
  if (!S.built.shapes.length && o.mode !== 'outline') { document.getElementById('mode-outline').checked = true; o.mode = 'outline'; }
  o.spacing = spacingFor(paths, o.density);
  S.spacing = o.spacing;
  busy(o.mode === 'outline' ? `Joining ${paths.length.toLocaleString()} pieces…` : 'Drawing the squiggle…');
  let prepared;
  try { prepared = await planAsync(S.built, paths, o); }
  catch (e) { busy(false); return fail(S.fileName, e.message); }
  if (token !== planToken) return; // a newer request is on its way
  S.ordered = prepared;
  S.planKey = JSON.stringify(o);
  S.box = null; S.result = null;
  // decimated copy for live dragging
  const total = S.ordered.curves.reduce((a, c) => a + c.length, 0);
  const k = Math.max(1, Math.ceil(total / 30000));
  S.preview = S.ordered.curves.map((c) => c.filter((_, i) => i % k === 0 || i === c.length - 1));
  renderFile();
  renderStyle();
  busy(false);
  if (refit || !S.place) fitToView(); else setPlace(S.place, { exportNow: true });
  if (refit) setTimeout(zoomToPlace, 50);
}

function renderStyle() {
  const shapes = S.built ? S.built.shapes.length : 0;
  $('styleSet').hidden = !S.parsed;
  for (const id of ['mode-fill', 'mode-both']) $(id).disabled = !shapes;
  const mode = styleOpts().mode;
  $('fillOpts').hidden = mode === 'outline';
  updateDensityLabel();
  $('wobbleOut').textContent = `${$('wobble').value}%`;
  const st = S.ordered && S.ordered.stats;
  let note = '';
  if (!shapes) note = 'No closed shapes found, so there’s nothing to fill. Fill needs closed outlines.';
  else if (mode !== 'outline' && st) {
    note = `${shapes} shape${shapes === 1 ? '' : 's'}, ${st.fillLoops} fill line${st.fillLoops === 1 ? '' : 's'}.`;
    if (st.splitShapes) note += ` ${st.splitShapes} shape${st.splitShapes === 1 ? ' has' : 's have'} parts too narrow to join at this density, so ${st.splitShapes === 1 ? 'it gets' : 'they get'} more than one line. Raise the density to fix.`;
  }
  $('fillNote').textContent = note;
  $('fillNote').hidden = !note;
}
function updateDensityLabel() {
  let txt = `${$('density').value} lines`;
  if (S.place && S.box && S.parsed) {
    const [X1] = toM(S.place.tl), [X2] = toM(S.place.br);
    const midLat = (S.place.tl[0] + S.place.br[0]) / 2;
    const mPerUnit = ((X2 - X1) / S.box.w) * Math.cos(midLat * Math.PI / 180);
    const sp = spacingFor(S.parsed.paths.filter((p) => S.layerOn.has(p.layer)), +$('density').value) * mPerUnit;
    txt += ` · ${sp >= 1000 ? `${(sp / 1000).toFixed(sp >= 10000 ? 0 : 1)} km` : `${Math.round(sp)} m`} apart`;
  }
  $('densityOut').textContent = txt;
}
for (const r of document.querySelectorAll('input[name="mode"]')) r.addEventListener('change', () => { store.set({ mode: r.value }); renderStyle(); reorder(); });
$('density').addEventListener('input', updateDensityLabel);
$('density').addEventListener('change', () => { store.set({ density: $('density').value }); reorder(); });
$('wobble').addEventListener('input', () => { $('wobbleOut').textContent = `${$('wobble').value}%`; });
$('wobble').addEventListener('change', () => { store.set({ wobble: $('wobble').value }); reorder(); });
$('shuffle').addEventListener('click', () => { S.seed = (Math.random() * 2 ** 31) >>> 0; reorder(); });
$('grouping').addEventListener('change', () => { store.set({ grouping: $('grouping').value }); reorder(); });
if (saved.density) $('density').value = saved.density;
if (saved.wobble != null) $('wobble').value = saved.wobble;
if (saved.grouping) $('grouping').value = saved.grouping;
if (saved.mode && $(`mode-${saved.mode}`)) $(`mode-${saved.mode}`).checked = true;

function renderFile() {
  const p = S.parsed;
  $('filecard').hidden = false;
  $('fname').textContent = S.sample ? 'Sample: bicycle' : S.fileName;
  $('ftag').textContent = S.sample ? 'sample' : S.kind;
  const dl = $('fstats'); dl.textContent = '';
  const add = (k, v) => { const dt = document.createElement('dt'); dt.textContent = k; const dd = document.createElement('dd'); dd.textContent = v; dl.append(dt, dd); };
  const on = p.paths.filter((x) => S.layerOn.has(x.layer));
  add('Pieces', on.length.toLocaleString());
  if (S.ordered) {
    if (S.built && S.built.shapes.length) add('Closed shapes', S.built.shapes.length.toLocaleString());
    add('Joined into', `${S.ordered.curves.length.toLocaleString()} line${S.ordered.curves.length === 1 ? '' : 's'}`);
    const b = getBox() || { w: 0, h: 0 };
    const u = p.units ? ` ${p.units}` : ' units';
    add('Drawing size', `${trim(b.w)} × ${trim(b.h)}${u}`);
  }
  $('fwarn').innerHTML = '';
  for (const w of p.warnings) { const li = document.createElement('li'); li.textContent = w; $('fwarn').append(li); }
  const lay = $('layers');
  if (p.layers.length > 1) {
    lay.hidden = false;
    const ll = $('layerlist'); ll.textContent = '';
    p.layers.forEach((name, i) => {
      const id = `layer-${i}`;
      const lab = document.createElement('label'); lab.className = 'check';
      const cb = document.createElement('input'); cb.type = 'checkbox'; cb.id = id; cb.checked = S.layerOn.has(name);
      cb.addEventListener('change', () => { cb.checked ? S.layerOn.add(name) : S.layerOn.delete(name); reorder(); });
      const n = p.paths.filter((x) => x.layer === name).length;
      lab.append(cb, ` ${name} `); const c = document.createElement('span'); c.className = 'unit'; c.textContent = `(${n})`; lab.append(c);
      ll.append(lab);
    });
  } else lay.hidden = true;
}
const trim = (v) => (Math.abs(v) >= 100 ? Math.round(v).toLocaleString() : v.toPrecision(3));

async function loadFile(file) {
  if (!file) return;
  if (file.size > 200 * 1024 * 1024) return fail(file.name, 'That file is over 200 MB.');
  const text = await file.text();
  await loadText(file.name, text);
}
$('file').addEventListener('change', (e) => { loadFile(e.target.files[0]); e.target.value = ''; });
const drop = $('drop');
let dragDepth = 0;
window.addEventListener('dragenter', (e) => { if ([...(e.dataTransfer?.types || [])].includes('Files')) { dragDepth++; $('dropveil').hidden = false; drop.classList.add('over'); } });
window.addEventListener('dragleave', () => { if (--dragDepth <= 0) { dragDepth = 0; $('dropveil').hidden = true; drop.classList.remove('over'); } });
window.addEventListener('dragover', (e) => e.preventDefault());
window.addEventListener('drop', (e) => {
  e.preventDefault(); dragDepth = 0; $('dropveil').hidden = true; drop.classList.remove('over');
  const f = e.dataTransfer?.files?.[0]; if (f) loadFile(f);
});

// first visit: show the sample so the page opens in a working state
map.once('load', () => { if (!S.parsed) loadText('sample-bike.svg', SAMPLE_SVG, { sample: true }); });
