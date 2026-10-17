import { order } from './core.js';

self.onmessage = (e) => {
  const { id, paths } = e.data;
  try {
    const t0 = Date.now();
    const res = order(paths);
    self.postMessage({ id, ok: true, ordered: res, ms: Date.now() - t0 });
  } catch (err) {
    self.postMessage({ id, ok: false, error: String(err && err.message || err) });
  }
};
