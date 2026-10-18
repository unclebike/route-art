// Turn shapes + loose lines into ordered route groups.
// Runs in the worker (no DOM, no Clipper needed).
import { order } from './core.js';
import { squiggleFill } from './fill.js';

/**
 * @param {{shapes: Array<{rings}>, open: Array<{pts}>}} built  from buildShapes
 * @param {Array<{pts}>} rawPaths  the drawing's own lines (used for 'outline' + join)
 * @param {{mode:'outline'|'fill'|'both', grouping:'join'|'shape', spacing:number, seed:number, wobble:number}} o
 */
export function planGroups(built, rawPaths, o) {
  const { mode = 'outline', grouping = 'join' } = o;
  const fills = [];
  let fillLoops = 0, splitShapes = 0;
  if (mode !== 'outline') {
    for (const sh of built.shapes) {
      const loops = squiggleFill(sh.rings, { spacing: o.spacing, seed: o.seed, wobble: o.wobble ?? 0.22 });
      if (loops.length > 1) splitShapes++;
      fillLoops += loops.length;
      fills.push(loops);
    }
  }
  const groups = [];
  if (grouping === 'join') {
    let geo = [];
    if (mode === 'outline') geo = rawPaths.map((p) => p.pts);
    else {
      fills.forEach((f) => geo.push(...f));
      if (mode === 'both') built.shapes.forEach((sh) => geo.push(...sh.rings));
      geo.push(...built.open.map((p) => p.pts));
    }
    groups.push({ name: null, ...order(geo) });
  } else {
    built.shapes.forEach((sh, i) => {
      const geo = [];
      if (mode !== 'fill') geo.push(...sh.rings);
      if (mode !== 'outline') geo.push(...fills[i]);
      if (geo.length) groups.push({ name: `shape ${i + 1}`, ...order(geo) });
    });
    if (built.open.length) groups.push({ name: 'lines', ...order(built.open.map((p) => p.pts)) });
  }
  const curves = groups.flatMap((g) => g.curves);
  return { groups, curves, stats: { shapes: built.shapes.length, fillLoops, splitShapes } };
}
