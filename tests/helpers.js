import { Engine } from '../src/sim/engine.js';
export const near = (a, b, tol, msg) => { if (!(Math.abs(a - b) <= tol)) throw new Error(`${msg || 'not near'}: ${a} vs ${b} (tol ${tol})`); };
let shared = null;
/** A world is expensive to build; tests share one (read-only use) via a throwaway engine. */
export function sharedWorld() { if (!shared) shared = new Engine({ fleetSize: 1, seed: 1 }).world; return shared; }
export const makeEngine = (o = {}) => new Engine({ fleetSize: 20, seed: 11, ...o });
/** Engine advanced until at least `n` aircraft are airborne (or `maxT` s). */
export function engineWithFlights(n = 3, o = {}, maxT = 400) {
  const e = makeEngine({ fleetSize: 40, demand: 3, ...o });
  while (e.airborneCount() < n && e.t < maxT) e.advance(2);
  return e;
}
export const stubCtx = () => new Proxy({ n: 0 }, {
  get(t, k) { if (k === 'n') return t.n; if (k === 'fillRect' || k === 'putImageData') return (...a) => { t.n++; for (const v of a) if (typeof v === 'number' && !Number.isFinite(v)) throw new Error(`non-finite argument to ${String(k)}`); }; return t[k]; },
  set(t, k, v) { t[k] = v; return true; },
});
