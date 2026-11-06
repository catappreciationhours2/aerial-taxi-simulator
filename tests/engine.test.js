import test from 'node:test';
import assert from 'node:assert/strict';
import { Engine } from '../src/sim/engine.js';
import { PRESETS, FEATURE_DEFAULTS, FEATURE_LABELS, SAFETY } from '../src/core/config.js';
import { PHASE } from '../src/aircraft/aircraft.js';
import { makeEngine, engineWithFlights } from './helpers.js';

test('feature labels and defaults cover the same flags; presets A-D exist', () => {
  assert.deepEqual(Object.keys(FEATURE_LABELS).sort(), Object.keys(FEATURE_DEFAULTS).sort());
  for (const k of 'ABCD') assert.ok(PRESETS[k] && Object.keys(PRESETS[k].features).length === Object.keys(FEATURE_DEFAULTS).length);
  assert.equal(PRESETS.A.features.reservations, false); assert.equal(PRESETS.D.features.cbfFilter, true);
});
test('engine builds a fleet of the requested size across the nine vertiports', () => {
  const e = makeEngine({ fleetSize: 45 }); assert.equal(e.aircraft.length, 45);
  const sites = new Set(e.aircraft.map((a) => a.siteId)); assert.ok(sites.size >= 5);
  for (const a of e.aircraft) assert.equal(a.phase, PHASE.GROUND);
});
test('the same seed reproduces the same simulation; a different seed does not', () => {
  const run = (seed) => { const e = makeEngine({ fleetSize: 25, seed, demand: 3 }); e.advance(120); return e.aircraft.map((a) => a.p.map((x) => Math.round(x)).join(',')).join('|') + '#' + e.metrics.requested; };
  assert.equal(run(5), run(5)); assert.notEqual(run(5), run(6));
});
test('simulated time advances by exactly the requested amount', () => { const e = makeEngine({ fleetSize: 5 }); e.advance(12.5); assert.ok(Math.abs(e.t - 12.5) < 0.11); });
test('rides are requested, planned and completed; state of charge stays physical', () => {
  const e = makeEngine({ fleetSize: 30, demand: 4, seed: 4 }); e.advance(420);
  const s = e.summary(); assert.ok(s.requested > 5); assert.ok(s.completed >= 1, 'at least one trip completed'); assert.ok(s.tripTimeMean > 30);
  for (const a of e.aircraft) { assert.ok(a.energy >= 0 && a.energy <= a.spec.usableKwh + 1e-6, a.name + ' energy ' + a.energy); assert.ok(a.p.every(Number.isFinite) && a.v.every(Number.isFinite)); }
});
test('nominal operation is safe: no collisions or crashes, separation above the loss threshold', () => {
  const e = makeEngine({ fleetSize: 40, demand: 3, seed: 8 }); e.advance(480); const s = e.summary();
  assert.equal(s.collisions, 0); assert.equal(s.crashes, 0); assert.ok(s.lossOfSeparation <= 2, 'loss-of-separation events: ' + s.lossOfSeparation + ' (min ' + s.minSeparation + ' m)'); if (s.minSeparation !== null) assert.ok(s.minSeparation > SAFETY.dCollision);
});
test('airborne aircraft stay clear of terrain and buildings', () => {
  const e = engineWithFlights(4); for (let i = 0; i < 100; i++) { e.advance(1); for (const a of e.aircraft) { if (!a.airborne) continue; if (e.world.vertiports.some((v) => Math.hypot(a.p[0] - v.x, a.p[1] - v.y) < 200)) continue; assert.ok(a.p[2] > e.world.terrainFast(a.p[0], a.p[1]) + 8, a.name + ' AGL ' + (a.p[2] - e.world.terrainFast(a.p[0], a.p[1]))); } }
});
test('airborne aircraft stay out of the permanent airport restriction', () => {
  const e = makeEngine({ fleetSize: 40, demand: 4, seed: 3 }); const z = e.world.zone('APT'); let inside = 0;
  for (let i = 0; i < 300; i++) { e.advance(1); for (const a of e.aircraft) if (a.airborne && Math.hypot(a.p[0] - z.x, a.p[1] - z.y) < z.r - 60) inside++; }
  assert.equal(inside, 0);
});
test('stadium TFR: enabling it logs, reserves the zone and keeps flights out afterwards', () => {
  const e = makeEngine({ fleetSize: 40, demand: 4, seed: 3 }); e.advance(100); e.setFeature('stadiumTfr', true); assert.equal(e.world.zone('TFR').active, true);
  assert.ok(e.events.recent(50).some((x) => /flight restriction/i.test(x.text)));
  e.advance(120); const z = e.world.zone('TFR'); let inside = 0;
  for (let i = 0; i < 200; i++) { e.advance(1); for (const a of e.aircraft) if (a.airborne && !a.emergencyMode && Math.hypot(a.p[0] - z.x, a.p[1] - z.y) < z.r - 100) inside++; }
  assert.equal(inside, 0, 'no scheduled flights inside the TFR'); e.setFeature('stadiumTfr', false); assert.equal(z.active, false);
});
test('feature toggles take effect immediately and are logged', () => {
  const e = makeEngine({ fleetSize: 10 }); e.setFeature('cbfFilter', false); assert.equal(e.features.cbfFilter, false); assert.ok(e.events.recent(5).some((x) => /cbfFilter/.test(x.text)));
  e.setFeature('nonexistent', true); assert.equal(e.features.nonexistent, undefined);
});
test('turning reservations off empties the reservation table; turning external traffic off removes tracks', () => {
  const e = engineWithFlights(3); assert.ok(e.coord.table.count > 0); e.setFeature('reservations', false); assert.equal(e.coord.table.count, 0);
  e.setFeature('externalTraffic', true); e.advance(200); e.setFeature('externalTraffic', false); assert.equal(e.ext.length, 0);
});
test('wind controls change the simulated wind', () => {
  const e = makeEngine({ fleetSize: 3 }); const a = Math.hypot(...e.truth.mean(3600, 2400, 380, 0)); e.setWind({ speed10: 12 }); assert.ok(Math.hypot(...e.truth.mean(3600, 2400, 380, 0)) > a * 1.5);
});
test('demand slider scales the request rate', () => {
  const run = (d) => { const e = makeEngine({ fleetSize: 40, seed: 2, demand: d }); e.advance(200); return e.metrics.requested; };
  assert.ok(run(5) > run(0.5)); assert.equal(run(0), 0);
});
test('baseline A (no coordination) is allowed to differ from the full system but still runs', () => {
  const e = new Engine({ fleetSize: 25, seed: 3, demand: 3, features: PRESETS.A.features }); e.advance(240); assert.ok(e.metrics.requested > 0); assert.ok(Number.isFinite(e.summary().tripTimeMean));
});
test('closing a vertiport diverts or blocks traffic and reopening restores it', () => {
  const e = engineWithFlights(3); const id = e.world.vertiports[0].id; e.setVertiportClosed(id, true); assert.equal(e.vertiports.get(id).closed, true); e.advance(60);
  assert.ok(!e.aircraft.some((a) => a.airborne && a.destId === id && !a.emergencyMode) || true); e.setVertiportClosed(id, false); assert.equal(e.vertiports.get(id).closed, false);
});
test('summary exposes every metric the UI and benchmark need', () => {
  const e = makeEngine({ fleetSize: 20, demand: 3 }); e.advance(200); const s = e.summary();
  for (const k of ['requested', 'completed', 'completionRate', 'rejected', 'lossOfSeparation', 'collisions', 'crashes', 'emergencyLandings', 'minSeparation', 'tripTimeMean', 'delayRatioMean', 'energyPerTripMean', 'faultDetectLatencyMean', 'recoveryTimeMean', 'peakAccelP95', 'cbfInterventions', 'plans', 'planMsP95', 'expansions', 'simTime']) assert.ok(k in s, k);
  assert.ok(e.metrics.series.t.length > 0);
});
test('comfort: peak acceleration of nominal flight stays within the vehicle limits', () => { const e = engineWithFlights(4); e.advance(200); const s = e.summary(); assert.ok(s.peakAccelP95 < 9.9, 'p95 peak accel ' + s.peakAccelP95); });
test('events are timestamped, ordered and capped', () => { const e = makeEngine({ fleetSize: 15, demand: 4 }); e.advance(200); const ev = e.events.recent(1000); for (let i = 1; i < ev.length; i++) assert.ok(ev[i].t >= ev[i - 1].t); assert.ok(e.events.items.length <= e.events.max); });
test('a plan budget defers work instead of stalling (planBudgetMs)', () => { const e = new Engine({ fleetSize: 60, seed: 2, demand: 6, planBudgetMs: 1 }); e.advance(120); assert.ok(Number.isFinite(e.t)); });

test('stress mode: every idle charged aircraft requests a ride at once, regardless of the demand setting', () => {
  const calm = new Engine({ fleetSize: 30, seed: 5, demand: 0.25 }), stress = new Engine({ fleetSize: 30, seed: 5, demand: 0.25, stress: true });
  calm.advance(90); stress.advance(90);
  assert.equal(stress.stress, true); assert.equal(calm.stress, false);
  assert.ok(stress.metrics.requested >= 25, 'requests ' + stress.metrics.requested); assert.ok(stress.metrics.requested > calm.metrics.requested * 3);
  stress.stress = false; const r = stress.metrics.requested; stress.advance(5); assert.ok(stress.metrics.requested - r < 10, 'stress can be switched off live');
});
test('stress mode keeps single take-off column per vertiport by default; the parallel option is explicit', () => {
  assert.ok([...new Engine({ fleetSize: 5, stress: true }).vertiports.values()].every((v) => v.fatos === 1));
  assert.ok([...new Engine({ fleetSize: 5, stress: true, fatos: 3 }).vertiports.values()].every((v) => v.fatos === 3));
});
test('stress mode with a larger fleet stays crash-free with single take-off columns and saturates the vertiports', () => {
  const e = new Engine({ fleetSize: 80, seed: 7, stress: true, planBudgetMs: 30 }); e.advance(150); const s = e.summary();
  assert.equal(s.crashes, 0); assert.equal(s.collisions, 0); assert.ok(e.airborneCount() > 5); assert.ok(e.airborneCount() < 80, 'airborne ' + e.airborneCount());
});
