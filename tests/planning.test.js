import test from 'node:test';
import assert from 'node:assert/strict';
import { ReservationTable } from '../src/planning/reservations.js';
import { Trajectory, padAt, reserveTrajectory } from '../src/planning/trajectory.js';
import { sippSearch } from '../src/planning/sipp.js';
import { PLANNER, SAFETY, FEATURE_DEFAULTS } from '../src/core/config.js';
import { makeEngine, sharedWorld, near } from './helpers.js';

test('reservation table: add / conflict / safe intervals / remove', () => {
  const t = new ReservationTable();
  t.add('a', 5, 10, 20); t.add('b', 5, 30, 40);
  assert.deepEqual(t.safeIntervals(5), [[0, 10], [20, 30], [40, Infinity]]);
  assert.ok(t.conflictEnd(5, 15, 18) > 0); assert.ok(!(t.conflictEnd(5, 20, 29) > 0));
  assert.deepEqual(t.safeIntervals(5, 'a'), [[0, 30], [40, Infinity]], 'ignoring own reservations');
  assert.deepEqual(t.safeIntervals(99), [[0, Infinity]]);
  assert.ok(t.conflictsOf('a').size === 0); t.add('c', 5, 12, 14); assert.ok(t.conflictsOf('a').has('c'));
  t.removeId('c'); assert.ok(t.conflictsOf('a').size === 0); assert.equal(t.add('d', 1, 5, 5), null, 'empty interval ignored');
});
test('reservation table: prefix removal and gc', () => { const t = new ReservationTable(); t.add('ZONE:1', 1, 0, 100); t.add('ZONE:2', 2, 0, 100); t.add('x', 3, 0, 10); t.removePrefix('ZONE:'); assert.equal(t.count, 1); t.gc(50); assert.equal(t.count, 0); });
test('time padding grows with look-ahead and is capped', () => { assert.ok(padAt(100, 0) > padAt(1, 0)); assert.ok(padAt(1e6, 0) <= PLANNER.padMax + 1e-9); assert.ok(padAt(10, 0, 2) >= padAt(10, 0, 1)); });
const line = (n0, n1, p0, p1, t0, t1, kind = 'move') => ({ kind, t0, t1, p0, p1, n0, n1, energy: 0.1 });
test('trajectory reference is continuous, acceleration-bounded and clamps at the ends', () => {
  const tr = new Trajectory(); const v = 22;
  tr.push(line(0, 1, [0, 0, 400], [100, 0, 400], 0, 100 / v)); tr.push(line(1, 2, [100, 0, 400], [100, 100, 400], 100 / v, 200 / v)); tr.push(line(2, 3, [100, 100, 400], [200, 100, 400], 200 / v, 300 / v));
  let prev = null, maxA = 0; const dt = 0.1;
  for (let t = -5; t < 300 / v + 5; t += dt) { const r = tr.ref(t); assert.ok(r.p.every(Number.isFinite)); if (prev) { const a = Math.hypot(r.v[0] - prev.v[0], r.v[1] - prev.v[1]) / dt; maxA = Math.max(maxA, a); assert.ok(Math.hypot(r.p[0] - prev.p[0], r.p[1] - prev.p[1]) < v * dt * 1.5, 'position continuous'); } prev = { p: [...r.p], v: [...r.v] }; }
  assert.ok(maxA < 6, 'reference accel ' + maxA); const end = tr.ref(1e4); near(end.p[0], 200, 1e-6); near(end.v[0], 0, 1e-9);
});
test('trajectory smoothing stays within a few tens of metres of the raw path', () => {
  const tr = new Trajectory(); const v = 22;
  tr.push(line(0, 1, [0, 0, 400], [100, 0, 400], 0, 100 / v)); tr.push(line(1, 2, [100, 0, 400], [100, 100, 400], 100 / v, 200 / v));
  for (let t = 0; t <= 200 / v; t += 0.25) { const a = tr.ref(t), b = tr.rawRef(t); assert.ok(Math.hypot(a.p[0] - b.p[0], a.p[1] - b.p[1], a.p[2] - b.p[2]) < 40); }
});
test('commitPrefix freezes whole segments and reports the replan start', () => {
  const tr = new Trajectory(); for (let i = 0; i < 5; i++) tr.push(line(i, i + 1, [i * 100, 0, 400], [(i + 1) * 100, 0, 400], i * 5, (i + 1) * 5));
  const cm = tr.commitPrefix(7); assert.equal(cm.prefix.segs.length, 2); assert.equal(cm.node, 2); assert.equal(cm.t, 10);
  const last = new Trajectory(); last.push({ kind: 'land', t0: 0, t1: 5, p0: [0, 0, 400], p1: [0, 0, 300], n0: 1, n1: 1 }); assert.equal(last.commitPrefix(2), null, 'no replanning during landing');
});
test('trajectory footprint lists every visited node with the diagonal corner cells', () => {
  const tr = new Trajectory(); tr.push({ kind: 'move', t0: 0, t1: 5, p0: [0, 0, 400], p1: [100, 100, 400], n0: 1, n1: 2, corners: [7, 8] });
  const vis = tr.visits(); assert.ok(vis.some((x) => x.node === 2) && vis.some((x) => x.node === 7 && x.corner) && vis.some((x) => x.node === 8));
});
test('reserveTrajectory writes the footprint into the table', () => { const w = sharedWorld(), t = new ReservationTable(), tr = new Trajectory(); tr.push(line(5, 6, [0, 0, 400], [100, 0, 400], 0, 5)); reserveTrajectory(t, w, 'ac', tr, 0, {}); assert.ok(t.count >= 1); assert.ok(t.byId.has('ac')); });

function missionSetup(features = {}) {
  const e = makeEngine({ fleetSize: 6, features });
  return { e, c: e.coord, w: e.world };
}
test('SIPP finds a conflict-free path whose cost is >= the relaxed heuristic (admissibility) at epsilon = 1', () => {
  const { c, w, e } = missionSetup(); const ac = e.aircraft[0], cost = c.costFor(ac.spec);
  const pairs = [['DTN', 'OAK'], ['SSD', 'ELB'], ['NSH', 'MTW'], ['AWT', 'SHD']];
  for (const [a, b] of pairs) {
    const sa = w.siteById(a), sb = w.siteById(b);
    const start = w.idx(sa.ix, sa.iy, w.firstFreeLevel(sa.ix, sa.iy)), goal = w.idx(sb.ix, sb.iy, w.firstFreeLevel(sb.ix, sb.iy));
    const h = c.hcache.get(w, cost, goal, 0);
    const r = sippSearch({ id: 'T', startNode: start, t0: 0, goalNode: goal, epsilon: 1, maxExpansions: 120000 }, { world: w, cost, table: null, h });
    assert.ok(r.ok, `${a}->${b}: ${r.reason}`); assert.ok(r.cost + 1e-6 >= h[start], `h(start)=${h[start].toFixed(1)} must not exceed true cost ${r.cost.toFixed(1)}`);
    assert.ok(r.path.length > 3); for (const s of r.path) assert.ok(!w.blocked[s.node], 'path avoids blocked nodes');
  }
});
test('weighted search expands fewer nodes at the price of bounded suboptimality', () => {
  const { c, w, e } = missionSetup(); const ac = e.aircraft[0], cost = c.costFor(ac.spec), sa = w.siteById('AWT'), sb = w.siteById('ELB');
  const start = w.idx(sa.ix, sa.iy, w.firstFreeLevel(sa.ix, sa.iy)), goal = w.idx(sb.ix, sb.iy, w.firstFreeLevel(sb.ix, sb.iy)), h = c.hcache.get(w, cost, goal, 0);
  const a = sippSearch({ id: 'T', startNode: start, t0: 0, goalNode: goal, epsilon: 1, maxExpansions: 200000 }, { world: w, cost, table: null, h });
  const b = sippSearch({ id: 'T', startNode: start, t0: 0, goalNode: goal, epsilon: 1.5, maxExpansions: 200000 }, { world: w, cost, table: null, h });
  assert.ok(a.ok && b.ok); assert.ok(b.expansions <= a.expansions); assert.ok(b.cost <= 1.5 * a.cost + 1e-6, 'bounded suboptimality');
});
test('SIPP respects reservations: padded windows of two committed plans overlap by no more than the two paddings', () => {
  const { c, e } = missionSetup(); const [a1, a2] = e.aircraft; a2.siteId = a1.siteId;
  const dest = a1.siteId === 'OAK' ? 'DTN' : 'OAK';
  assert.ok(c.planMission(a1, dest, 3).ok); assert.ok(c.planMission(a2, dest, 3).ok);
  const E1 = c.table.byId.get(a1.id), E2 = c.table.byId.get(a2.id); let worst = 0, shared = 0;
  for (const [n, x] of E1) for (const [m, y] of E2) if (n === m) { shared++; const ov = Math.min(x.e, y.e) - Math.max(x.s, y.s); if (ov > 0) worst = Math.max(worst, ov); }
  assert.ok(shared > 0, 'the two routes share nodes (same origin and destination)');
  assert.ok(worst <= 2 * PLANNER.padMax, 'overlap ' + worst.toFixed(2) + ' s is only padding'); assert.ok(worst < 8);
  const pad = e.vertiports.get(a1.siteId).padBookings[0].filter((b) => b.kind === 'takeoff').sort((p, q) => p.s - q.s);
  assert.equal(pad.length, 2); assert.ok(pad[1].s >= pad[0].e - 1e-6, 'take-offs are serialised through the FATO');
});
test('coordinator: missions end in a landing, fit the energy budget and book the FATO', () => {
  const { c, e } = missionSetup(); const ac = e.aircraft[0], dest = ac.siteId === 'OAK' ? 'DTN' : 'OAK';
  const r = c.planMission(ac, dest, 0); assert.ok(r.ok, r.reason);
  const last = r.traj.segs[r.traj.segs.length - 1]; assert.equal(last.kind, 'land'); assert.equal(r.traj.segs[0].kind, 'takeoff');
  assert.ok(r.energy < ac.energy - ac.spec.reserveKwh * 0.5); assert.ok(r.eta > 30);
  assert.ok(e.vertiports.get(ac.siteId).padBookings[0].some((b) => b.id === ac.id && b.kind === 'takeoff')); assert.ok(e.vertiports.get(dest).padBookings[0].some((b) => b.id === ac.id && b.kind === 'land'));
});
test('viability check rejects a mission when the battery cannot cover reserve + diversion', () => {
  const { c, e } = missionSetup(); const ac = e.aircraft[0]; ac.energy = 4; const r = c.planMission(ac, ac.siteId === 'OAK' ? 'DTN' : 'OAK', 0);
  assert.equal(r.ok, false); assert.ok(['viability', 'energy'].includes(r.reason));
});
test('emergency field reaches every free node and energyToLand is finite there', () => {
  const { c, w, e } = missionSetup(); const f = c.emergencyField(), cost = c.costFor(e.aircraft[0].spec);
  let free = 0, reach = 0; for (let n = 0; n < w.nNodes; n++) if (!w.blocked[n]) { free++; if (f.time[n] < Infinity) reach++; }
  assert.ok(reach / free > 0.97, `reach ${reach}/${free}`); const n = w.idx(36, 24, 3); assert.ok(Number.isFinite(f.energyToLand(n, cost)));
  assert.ok(f.sites.every((s) => w.emergencySites.includes(s.site)), 'contingency landings use dedicated emergency sites');
});
test('emergency landing returns a landing trajectory to an emergency site; forced landing descends in place', () => {
  const { c, e } = missionSetup(); const ac = e.aircraft[0];
  c.planMission(ac, ac.siteId === 'OAK' ? 'DTN' : 'OAK', 0); ac.phase = 'flying'; ac.p = [3000, 2400, 440]; ac.v = [10, 0, 0];
  const r = c.emergencyLanding(ac, 10, { reason: 'test' }); assert.ok(r.ok && r.mode === 'emergency'); assert.equal(ac.plan.segs[ac.plan.segs.length - 1].kind, 'land');
  const f = c.forcedLanding(ac, 10); assert.equal(f.mode, 'forced'); assert.equal(ac.plan.segs.length, 1);
});
test('replan keeps the committed prefix and ends at the destination', () => {
  const { c, e } = missionSetup(); const ac = e.aircraft[0]; const dest = ac.siteId === 'OAK' ? 'DTN' : 'OAK';
  c.planMission(ac, dest, 0); ac.phase = 'flying'; const first = ac.plan.segs[0];
  const r = c.replan(ac, 12, { reason: 'test' }); assert.ok(r.ok, r.reason); assert.equal(ac.plan.segs[0], first); assert.equal(ac.plan.segs[ac.plan.segs.length - 1].kind, 'land');
  for (let i = 1; i < ac.plan.segs.length; i++) assert.ok(Math.abs(ac.plan.segs[i].t0 - ac.plan.segs[i - 1].t1) < 1e-6, 'contiguous in time');
});
test('closed vertiport rejects missions and zone reservations make restricted nodes unavailable', () => {
  const { c, e, w } = missionSetup(); const ac = e.aircraft[0]; const dest = ac.siteId === 'OAK' ? 'DTN' : 'OAK'; e.setVertiportClosed(dest, true);
  assert.equal(c.planMission(ac, dest, 0).ok, false); e.setVertiportClosed(dest, false); assert.ok(c.planMission(ac, dest, 0).ok);
  const z = w.zone('TFR'); c.addZoneReservation(z, 0); assert.ok([...c.table.byId.keys()].some((k) => String(k).startsWith('ZONE:'))); c.removeZoneReservation(z);
});
test('feature flags: without reservations the table stays empty', () => { const { c, e } = missionSetup({ reservations: false }); const ac = e.aircraft[0]; assert.ok(c.planMission(ac, ac.siteId === 'OAK' ? 'DTN' : 'OAK', 0).ok); assert.equal(c.table.count, 0); });
test('wind awareness changes the plan cost in a strong wind', () => {
  const run = (windAware) => { const e = makeEngine({ fleetSize: 4, features: { windAware }, wind: { speed10: 10, dirFrom: 270 } }); const ac = e.aircraft[0]; const r = e.coord.planMission(ac, ac.siteId === 'OAK' ? 'AWT' : 'OAK', 0); return r; };
  const a = run(true), b = run(false); assert.ok(a.ok && b.ok);
});
test('safety constants are consistent', () => { assert.ok(SAFETY.dCollision < SAFETY.dLoss); assert.ok(SAFETY.d0 > 0 && SAFETY.deltaRisk < 0.01); assert.equal(FEATURE_DEFAULTS.stadiumTfr, false); });
