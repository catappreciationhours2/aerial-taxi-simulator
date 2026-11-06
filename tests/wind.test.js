import test from 'node:test';
import assert from 'node:assert/strict';
import { TrueWind, ForecastWind, FleetWindMap, EstimatedWind } from '../src/env/wind.js';
import { FEATURE_DEFAULTS } from '../src/core/config.js';
import { sharedWorld, near } from './helpers.js';

const w = sharedWorld();
const mk = (o = {}) => { const truth = new TrueWind(w, 3, o), fc = new ForecastWind(truth, 4), map = new FleetWindMap(w); return { truth, fc, map, est: new EstimatedWind(fc, map, { ...FEATURE_DEFAULTS }) }; };
test('true wind is finite everywhere and has the requested magnitude scale', () => {
  const { truth } = mk({ speed10: 6 });
  for (let i = 0; i < 200; i++) { const v = truth.at((i * 37) % 7200, (i * 91) % 4800, 320 + (i % 4) * 60, i * 3); for (const c of v) assert.ok(Number.isFinite(c)); }
  const s = Math.hypot(...truth.mean(3600, 2400, 380, 0).slice(0, 2)); assert.ok(s > 2 && s < 25, 'mean wind ' + s);
});
test('wind speeds scale with the speed10 setting', () => {
  const lo = mk({ speed10: 2 }).truth, hi = mk({ speed10: 10 }).truth;
  assert.ok(Math.hypot(...hi.mean(3000, 2000, 440, 0)) > 2 * Math.hypot(...lo.mean(3000, 2000, 440, 0)));
});
test('wind direction convention: wind from 270 blows toward the east', () => { const { truth } = mk({ speed10: 8, dirFrom: 270 }); const v = truth.mean(3600, 2400, 500, 0); assert.ok(v[0] > 1 && Math.abs(v[1]) < Math.abs(v[0])); });
test('wind increases with height (power law)', () => { const { truth } = mk({ speed10: 5 }); const a = Math.hypot(...truth.mean(5000, 4000, 320, 0)), b = Math.hypot(...truth.mean(5000, 4000, 500, 0)); assert.ok(b > a * 0.95); });
test('storm multiplies the wind and ends', () => {
  const { truth } = mk({ speed10: 5 }); truth.setStorm(10, 2, 20, 100);
  assert.equal(truth.stormFactor(0), 1); near(truth.stormFactor(60), 2, 1e-9); near(truth.stormFactor(10 + 20 + 100 + 30), 1, 1e-9);
  truth.clearStorm(); assert.equal(truth.stormFactor(60), 1);
});
test('turbulence flag: gusts vanish when disabled', () => { const { truth } = mk({ speed10: 6 }); truth.turbulence = false; const a = truth.at(2000, 2000, 400, 5), b = truth.mean(2000, 2000, 400, 5); for (let i = 0; i < 3; i++) near(a[i], b[i], 1e-9); });
test('injected disturbance cell changes the local wind and expires', () => {
  const { truth } = mk({ speed10: 4 }); const before = truth.at(2000, 2000, 400, 10);
  truth.addDisturbance({ x: 2000, y: 2000, z: 400, r: 300, vec: [0, 15, 0], t0: 0, t1: 50 });
  const during = truth.at(2000, 2000, 400, 10); assert.ok(Math.abs(during[1] - before[1]) > 5);
  truth.pruneDisturbances(60); assert.equal(truth.disturbances.length, 0);
});
test('forecast is biased relative to truth; estimate with an empty fleet map equals forecast', () => {
  const { truth, fc, est } = mk({ speed10: 6 }); let diff = 0;
  for (let i = 0; i < 40; i++) { const x = 500 + i * 150, y = 800 + (i % 7) * 400; const a = truth.mean(x, y, 400, 0), b = fc.mean(x, y, 400, 0); diff += Math.hypot(a[0] - b[0], a[1] - b[1]); }
  assert.ok(diff / 40 > 0.2, 'forecast error exists');
  const e = est.at(3000, 2000, 400, 0), f = fc.mean(3000, 2000, 400, 0); near(e[0], f[0], 1e-6); near(e[1], f[1], 1e-6);
});
test('fleet wind map learns: estimate error shrinks after observations', () => {
  const { truth, fc, map, est } = mk({ speed10: 7 }); truth.turbulence = false;
  const pts = []; for (let i = 0; i < 12; i++) pts.push([1500 + i * 400, 1200 + (i % 4) * 700, 380]);
  const err = () => { let s = 0; for (const [x, y, z] of pts) { const a = truth.mean(x, y, z, 100), b = est.at(x, y, z, 100); s += Math.hypot(a[0] - b[0], a[1] - b[1]); } return s / pts.length; };
  const before = err();
  for (let rep = 0; rep < 6; rep++) for (const [x, y, z] of pts) { const a = truth.mean(x, y, z, 100), b = fc.mean(x, y, z, 100); map.update(x, y, z, 100 + rep, a[0] - b[0], a[1] - b[1]); est.bump(); }
  assert.ok(err() < before * 0.7, `error ${before.toFixed(2)} -> ${err().toFixed(2)}`);
  assert.ok(map.coverage(110) > 0);
});
test('estimated wind uncertainty is positive; speedBound is an upper bound of the mean wind', () => {
  const { est } = mk({ speed10: 6 });
  assert.ok(est.sigma(3000, 2000, 400, 0) > 0);
  const b = est.speedBound(3000, 2000, 400, 0, 600); for (let t = 0; t <= 600; t += 100) { const v = est.at(3000, 2000, 400, t); assert.ok(Math.hypot(v[0], v[1]) <= b + 1e-6); }
});
test('hazards broadcast by aircraft alter the estimate and then expire', () => {
  const { est } = mk({ speed10: 4 }); const a = est.at(2500, 2500, 380, 20);
  est.addHazard({ x: 2500, y: 2500, z: 380, r: 400, vec: [10, 0, 0], t0: 0, t1: 50 }); const b = est.at(2500, 2500, 380, 20);
  assert.ok(Math.abs(b[0] - a[0]) > 2); est.pruneHazards(100); assert.equal(est.hazards.length, 0);
});
