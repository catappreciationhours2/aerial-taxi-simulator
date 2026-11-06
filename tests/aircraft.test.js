import test from 'node:test';
import assert from 'node:assert/strict';
import { makeSpec, powerKW, energyKWh, hoverEndurance } from '../src/aircraft/power.js';
import { limitAccel, stepPlant, vMaxEff } from '../src/aircraft/dynamics.js';
import { NavFilter } from '../src/aircraft/estimator.js';
import { Aircraft, PHASE } from '../src/aircraft/aircraft.js';
import { AIRCRAFT_CLASSES, G, RHO } from '../src/core/config.js';
import { RNG } from '../src/core/util.js';
import { near, sharedWorld } from './helpers.js';

test('hover power follows actuator-disc momentum theory', () => {
  const s = makeSpec('standard'), ideal = Math.pow(s.mass * G, 1.5) / Math.sqrt(2 * RHO * s.discArea);
  near(s.pIdealHoverW, ideal, ideal * 1e-6);
  const p = powerKW(s, 0, 0); assert.ok(p > ideal / 1000 / 0.72 / 0.95 * 0.9 && p < ideal / 1000 / 0.72 * 1.4, 'hover kW ' + p);
});
test('heavier / larger classes need more hover power; bigger discs help', () => { const c = makeSpec('compact'), st = makeSpec('standard'), l = makeSpec('large'); assert.ok(c.pHoverKw < st.pHoverKw && st.pHoverKw < l.pHoverKw); assert.ok(Object.keys(AIRCRAFT_CLASSES).length === 3); });
test('power curve has a minimum at a forward speed below hover power (pMin bound)', () => {
  const s = makeSpec('standard'); assert.ok(s.pMinKw < s.pHoverKw); assert.ok(s.vBestEndurance > 5 && s.vBestEndurance <= s.vMax); // wingless model: forward flight unloads the rotors up to vMax
  for (let v = 0; v <= s.vMax; v += 1) assert.ok(powerKW(s, v, 0) >= s.pMinKw - 1e-9, 'v=' + v);
});
test('power rises with climb rate and with propulsion degradation or battery fault', () => {
  const s = makeSpec('standard'); assert.ok(powerKW(s, 20, 3) > powerKW(s, 20, 0)); assert.ok(powerKW(s, 20, 0, 0.7) > powerKW(s, 20, 0)); assert.ok(powerKW(s, 20, 0, 1, 2.5) > 2.4 * powerKW(s, 20, 0));
});
test('energy and endurance helpers are consistent', () => { const s = makeSpec('standard'); near(energyKWh(s, 20, 0, 3600), powerKW(s, 20, 0), 1e-9); near(hoverEndurance(s, 10), (10 * 3600) / s.pHoverKw, 1e-9); assert.ok(s.reserveKwh > 0 && s.reserveKwh < s.usableKwh); });
test('limitAccel respects the thrust-ball and horizontal limits', () => {
  const s = makeSpec('standard');
  for (const a of [[100, 0, 0], [0, 100, 50], [3, 3, -50], [0, 0, 100]]) { const r = limitAccel(a, s, 1); assert.ok(Math.hypot(r[0], r[1]) <= s.aHorizMax + 1e-9); assert.ok(r[2] <= s.aUpMax + 1e-9 && r[2] >= -s.aDownMax - 1e-9); }
  const small = limitAccel([100, 0, 0], s, 0.5); assert.ok(small[0] < s.aHorizMax, 'degraded thrust shrinks authority');
});
test('airspeed envelope shrinks with propulsion loss', () => { const s = makeSpec('standard'); assert.equal(vMaxEff(s, 1), s.vMax); assert.ok(vMaxEff(s, 0.5) < vMaxEff(s, 1)); assert.ok(vMaxEff(s, 0) > 0); });
test('plant integrates acceleration, drains energy, and clamps airspeed', () => {
  const site = { id: 'X', x: 100, y: 100, padElev: 300 }, ac = new Aircraft(0, 'standard', site, new RNG(1), 1);
  ac.phase = PHASE.FLYING; ac.p = [100, 100, 400]; ac.v = [0, 0, 0]; ac.aCmd = [2, 0, 0];
  const e0 = ac.energy; for (let i = 0; i < 10; i++) stepPlant(ac, [0, 0, 0], 0.1);
  near(ac.v[0], 2, 0.2, 'v after 1 s of 2 m/s^2'); assert.ok(ac.energy < e0);
  ac.v = [ac.spec.vMax * 3, 0, 0]; stepPlant(ac, [0, 0, 0], 0.1); assert.ok(Math.hypot(ac.v[0], ac.v[1]) <= ac.spec.vMax + 1e-6);
  ac.v = [0, 0, 0]; ac.aCmd = [0, 0, 0]; stepPlant(ac, [10, 0, 0], 0.1); assert.ok(Number.isFinite(ac.powerKw));
});
test('nav filter tracks a moving truth with small error when healthy', () => {
  const rng = new RNG(2), nav = new NavFilter([0, 0, 300], rng, 2.5); let p = [0, 0, 300], worst = 0;
  for (let i = 0; i < 600; i++) { p = [p[0] + 2, p[1], p[2]]; nav.predict([20, 0, 0], 0.1); nav.update(p, i * 0.1); worst = Math.max(worst, Math.hypot(nav.p[0] - p[0], nav.p[1] - p[1])); }
  assert.ok(worst < 12, 'worst error ' + worst); assert.equal(nav.isolated, false); assert.ok(nav.sigma > 0);
});
test('nav filter detects a GNSS step bias, isolates GNSS and keeps coasting', () => {
  const rng = new RNG(3), nav = new NavFilter([0, 0, 300], rng, 2.5); let p = [0, 0, 300], detected = null;
  for (let i = 0; i < 400; i++) {
    p = [p[0] + 2, p[1], p[2]]; if (i === 100) nav.setBias([70, 0, 0], 0, i * 0.1);
    nav.predict([20, 0, 0], 0.1); const r = nav.update(p, i * 0.1); if (r === 'gps_fault' && detected === null) detected = i;
  }
  assert.ok(detected !== null && detected < 140, 'detected at step ' + detected); assert.ok(nav.isolated);
  assert.ok(Math.hypot(nav.p[0] - p[0], nav.p[1] - p[1]) < 40, 'estimate not dragged by spoofed fixes');
});
test('nav filter does not false-alarm over a long healthy run', () => {
  const nav = new NavFilter([0, 0, 300], new RNG(5), 2.5); let p = [0, 0, 300], alarms = 0;
  for (let i = 0; i < 6000; i++) { p = [p[0] + 1.5, p[1] + 0.5, p[2]]; nav.predict([15, 5, 0], 0.1); if (nav.update(p, i * 0.1) === 'gps_fault') alarms++; }
  assert.equal(alarms, 0);
});
test('Aircraft starts parked with a sane state of charge', () => {
  const w = sharedWorld(), ac = new Aircraft(3, 'large', w.vertiports[0], new RNG(1), 0.8);
  assert.equal(ac.name, 'AC-004'); assert.equal(ac.phase, PHASE.GROUND); near(ac.soc, 0.8, 1e-9); assert.equal(ac.airborne, false); assert.equal(ac.p[0], w.vertiports[0].x);
});
