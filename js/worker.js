import { planGroups } from './plan.js';

self.onmessage = (e) => {
  const { id, built, rawPaths, opts } = e.data;
  try {
    const t0 = Date.now();
    const res = planGroups(built, rawPaths, opts);
    self.postMessage({ id, ok: true, prepared: res, ms: Date.now() - t0 });
  } catch (err) {
    self.postMessage({ id, ok: false, error: String((err && err.message) || err) });
  }
};
