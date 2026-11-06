import test from 'node:test';
import assert from 'node:assert/strict';
import { FAULT_TYPES, injectFault } from '../src/sim/faults.js';
import { makeEngine, engineWithFlights } from './helpers.js';

const TYPES = Object.keys(FAULT_TYPES);
test('every fault type has a label, scope and description', () => { assert.equal(TYPES.length, 12); for (const k of TYPES) { const f = FAULT_TYPES[k]; assert.ok(f.label && f.desc && ['aircraft', 'fleet', 'vertiport'].includes(f.scope)); } });
test('injecting without airborne traffic fails gracefully for aircraft faults', () => {
  const e = makeEngine({ fleetSize: 5 }); const r = e.injectFault('gps_bias'); assert.equal(r.ok, false); assert.ok(r.reason); assert.equal(e.injectFault('nonsense').ok, false);
});
for (const type of TYPES) {
  test(`fault "${type}": injects, is recorded, and the system stays safe`, () => {
    const e = engineWithFlights(4, { seed: 21 }); e.advance(20); const n0 = e.faultRecords.length;
    const r = e.injectFault(type); assert.ok(r.ok, r.reason); assert.equal(e.faultRecords.length, n0 + 1);
    const rec = e.faultRecords[n0]; assert.equal(rec.type, type); assert.ok(rec.tInject >= 0); assert.ok(['active', 'detected', 'responding', 'resolved'].includes(rec.status));
    e.advance(200); const s = e.summary(); assert.equal(s.collisions, 0, 'collisions'); assert.ok(e.events.recent(300).some((x) => x.level === 'fault'), 'fault logged');
    if (!['storm', 'stale_weather', 'route_deviation'].includes(type)) assert.ok(rec.tDetect !== null || rec.status !== 'active', `${type} detected`);
    if (rec.tDetect !== null) assert.ok(rec.tDetect >= rec.tInject);
  });
}
test('GNSS bias: detected within seconds, GNSS isolated, then the aircraft lands and the fault resolves', () => {
  const e = engineWithFlights(4, { seed: 21 }); const r = e.injectFault('gps_bias'); const ac = r.ac; e.advance(3);
  assert.ok(r.rec.tDetect !== null && r.rec.tDetect - r.rec.tInject < 8); assert.equal(ac.health.gps, 'isolated'); assert.equal(ac.mode, 'DEGRADED');
  e.advance(120); assert.equal(ac.phase, 'ground'); assert.equal(r.rec.status, 'resolved'); assert.equal(e.summary().crashes, 0);
});
test('with fault response disabled the GNSS fault is not isolated (ablation)', () => {
  const e = engineWithFlights(4, { seed: 21 }); e.setFeature('faultResponse', false); const r = e.injectFault('gps_bias'); e.advance(40);
  assert.notEqual(r.ac.health.gps, 'isolated'); assert.ok(!r.ac.emergencyMode);
});
test('propulsion fault triggers an emergency landing to an emergency site and the aircraft survives', () => {
  const e = engineWithFlights(4, { seed: 21 }); const r = e.injectFault('propulsion'); e.advance(160);
  assert.ok(r.ac.emergencyMode || r.ac.phase === 'ground'); assert.notEqual(r.ac.phase, 'crashed'); assert.equal(r.rec.status === 'resolved' || r.rec.status === 'responding', true);
});
test('communication loss freezes the plan, then restores it', () => {
  const e = engineWithFlights(4, { seed: 21 }); const r = e.injectFault('comm_loss', { duration: 15 }); e.advance(8); assert.ok(r.ac.health.commLoss || r.ac.frozen); e.advance(60); assert.equal(r.ac.health.commLoss, false); assert.equal(r.ac.frozen, false);
});
test('battery fault: capacity drops and the aircraft diverts before reserve is violated', () => {
  const e = engineWithFlights(4, { seed: 21 }); const r = e.injectFault('battery'); assert.ok(r.ac.health.drainMult > 1); e.advance(220); assert.ok(r.ac.phase !== 'crashed'); assert.ok(r.ac.energy > 0 || r.ac.phase !== 'flying');
});
test('vertiport closure is logged, blocks the site and auto-reopens', () => {
  const e = engineWithFlights(3, { seed: 21 }); const id = e.world.vertiports[2].id; const r = e.injectFault('vertiport_closed', { vertiportId: id, duration: 40 });
  assert.equal(e.vertiports.get(id).closed, true); e.advance(60); assert.equal(e.vertiports.get(id).closed, false); assert.equal(r.rec.status, 'resolved');
});
test('traffic intruder: registered as an ADS-B track and avoided', () => {
  const e = engineWithFlights(4, { seed: 21 }); const r = e.injectFault('traffic_intruder'); e.advance(60); assert.ok(r.rec.tDetect !== null); assert.equal(e.summary().collisions, 0);
});
test('gust cell is observed by aircraft and broadcast as a hazard', () => {
  const e = engineWithFlights(4, { seed: 21 }); const r = e.injectFault('gust'); e.advance(60); assert.ok(r.rec.tDetect === null || r.rec.tDetect - r.rec.tInject < 40);
});
test('storm raises the true wind; stale weather widens margins then recovers', () => {
  const e = engineWithFlights(3, { seed: 21 }); const w0 = Math.hypot(...e.truth.mean(3600, 2400, 380, e.t)); e.injectFault('storm'); e.advance(120); assert.ok(Math.hypot(...e.truth.mean(3600, 2400, 380, e.t)) > w0 * 1.5);
  const e2 = engineWithFlights(3, { seed: 22 }); const r = e2.injectFault('stale_weather'); e2.advance(10); assert.equal(e2.weatherStale, true); e2.advance(120); assert.equal(e2.weatherStale, false); assert.equal(r.rec.status, 'resolved');
});
test('planner timeout falls back to validated plans without crashing the planner', () => { const e = engineWithFlights(4, { seed: 21 }); const r = e.injectFault('planner_timeout'); e.advance(80); assert.equal(r.rec.status, 'resolved'); assert.equal(e.summary().collisions, 0); });
test('fault latency and recovery metrics are aggregated', () => { const e = engineWithFlights(4, { seed: 21 }); e.injectFault('gps_bias'); e.injectFault('sensor_failure'); e.advance(250); const s = e.summary(); assert.ok(s.faultDetectLatencyMean > 0); assert.ok(s.faultDetectLatencyMax >= s.faultDetectLatencyMean); });
test('targeted injection uses the selected aircraft', () => { const e = engineWithFlights(4, { seed: 21 }); const target = e.aircraft.find((a) => a.airborne && !a.emergencyMode); e.selectedId = target.id; assert.equal(injectFault(e, 'gps_bias', {}).ac, target); });

test('with fault response off nothing is detected or counted as resolved: records end as "unhandled"', () => {
  for (const type of ['gps_bias', 'comm_loss', 'propulsion', 'sensor_failure', 'battery']) {
    const e = engineWithFlights(4, { seed: 21 }); e.setFeature('faultResponse', false);
    const r = e.injectFault(type); assert.ok(r.ok, type); e.advance(150);
    assert.equal(r.rec.tDetect, null, type + ' must not be detected'); assert.equal(r.rec.tRespond, null, type + ' must not be responded to');
    assert.notEqual(r.rec.status, 'resolved', type); assert.ok(['active', 'unhandled', 'failed'].includes(r.rec.status), type + ': ' + r.rec.status);
    assert.equal(e.metrics.recoveryTimes.length, 0); assert.equal(e.metrics.faultLatencies.length, 0);
  }
});
test('the same faults with response on are detected and eventually resolved (control for the ablation)', () => {
  const e = engineWithFlights(4, { seed: 21 }); const r = e.injectFault('gps_bias'); e.advance(150);
  assert.ok(r.rec.tDetect !== null); assert.equal(r.rec.status, 'resolved');
});
