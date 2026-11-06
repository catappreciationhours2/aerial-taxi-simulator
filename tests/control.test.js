import test from 'node:test';
import assert from 'node:assert/strict';
import { solveMPC, pdTrack } from '../src/control/mpc.js';
import { closestApproach, segmentMinDistance, protectedRadius, lossProbability, requiredMeanSeparation } from '../src/control/separation.js';
import { cbfFilter, dykstra, pairConstraint } from '../src/control/safety_filter.js';
import { near } from './helpers.js';

const lim = { h: 4.5, up: 2, dn: 2.5 };
const cvRef = (p0, v, dt = 0.5) => (k) => ({ p: [p0[0] + v[0] * dt * k, p0[1] + v[1] * dt * k, p0[2] + v[2] * dt * k], v });
test('MPC converges onto a constant-velocity reference', () => {
  let p = [0, 30, 400], v = [0, 0, 0], u = [0, 0, 0], warm = null; const vr = [20, 0, 0];
  for (let i = 0; i < 160; i++) { const t = i * 0.5; const sol = solveMPC({ p, v }, cvRef([vr[0] * t, 0, 400], vr), lim, [0, 0, 0], warm, u); warm = sol.u; u = sol.a0; for (let s = 0; s < 5; s++) { v = [v[0] + u[0] * 0.1, v[1] + u[1] * 0.1, v[2] + u[2] * 0.1]; p = [p[0] + v[0] * 0.1, p[1] + v[1] * 0.1, p[2] + v[2] * 0.1]; } }
  const t = 160 * 0.5; assert.ok(Math.abs(p[1]) < 1.5, 'lateral error ' + p[1]); assert.ok(Math.abs(p[0] - vr[0] * t) < 4, 'along error'); near(v[0], 20, 0.6);
});
test('MPC never exceeds the acceleration limits', () => {
  const sol = solveMPC({ p: [0, 0, 400], v: [0, 0, 0] }, cvRef([500, 500, 600], [0, 0, 0]), lim);
  for (let k = 0; k < sol.u.length; k += 3) { assert.ok(Math.hypot(sol.u[k], sol.u[k + 1]) <= lim.h + 1e-6); assert.ok(sol.u[k + 2] <= lim.up + 1e-6 && sol.u[k + 2] >= -lim.dn - 1e-6); }
  assert.equal(sol.saturated, true, 'far target saturates');
});
test('MPC compensates a constant disturbance estimate', () => {
  const sol = solveMPC({ p: [0, 0, 400], v: [10, 0, 0] }, cvRef([0, 0, 400], [10, 0, 0]), lim, [-1, 0, 0]);
  assert.ok(sol.a0[0] > 0.5, 'commands thrust against the headwind disturbance: ' + sol.a0[0]);
});
test('PD tracker is bounded and points toward the reference', () => { const a = pdTrack({ p: [0, 0, 0], v: [0, 0, 0] }, { p: [1000, 0, 100], v: [0, 0, 0] }, lim); assert.ok(a[0] > 0 && Math.hypot(a[0], a[1]) <= lim.h + 1e-9 && a[2] <= lim.up); });
test('closestApproach and segmentMinDistance', () => {
  const c = closestApproach([0, 0, 0], [10, 0, 0], [100, 20, 0], [0, 0, 0]); near(c.t, 10, 1e-9); near(c.d, 20, 1e-9);
  near(closestApproach([0, 0, 0], [0, 0, 0], [3, 4, 0], [0, 0, 0]).d, 5, 1e-9);
  near(segmentMinDistance([0, 0, 0], [100, 0, 0], [100, 30, 0], [0, 30, 0], 0, 10), 30, 1e-9);
  assert.ok(segmentMinDistance([0, 0, 0], [100, 0, 0], [100, 0, 0], [0, 0, 0], 0, 10) < 1e-6, 'head-on pass is a collision');
});
test('protected radius grows with closing speed and uncertainty', () => {
  const a = protectedRadius({ vClose: 0 }), b = protectedRadius({ vClose: 30 }), c = protectedRadius({ vClose: 30, sigma: 10 }); assert.ok(a < b && b < c);
});
test('chance-constrained separation: loss probability is monotone and invertible', () => {
  assert.ok(lossProbability(100, 50, 10) < lossProbability(60, 50, 10)); assert.ok(lossProbability(100, 50, 5) < lossProbability(100, 50, 20));
  const d = requiredMeanSeparation(50, 10, 1e-3); near(lossProbability(d, 50, 10), 1e-3, 5e-4);
});
test('pairConstraint: far pair is loose, approaching pair demands braking, violated pair escapes', () => {
  const far = pairConstraint([1000, 0, 0], [0, 0, 0], 40, 2.4, 0.9); assert.ok(far.b > 5, 'loose');
  const closing = pairConstraint([200, 0, 0], [-40, 0, 0], 40, 2.4, 0.9); assert.ok(closing.n[0] < 0 || closing.n[0] > 0); assert.ok(closing.b < far.b);
  assert.equal(pairConstraint([20, 0, 0], [0, 0, 0], 40, 2.4, 0.9).escape, true);
});
test('CBF filter leaves a safe nominal command untouched', () => {
  const r = cbfFilter([1, 0, 0], { p: [0, 0, 400], v: [10, 0, 0], sigma: 0 }, [{ p: [900, 600, 400], v: [0, 0, 0], sigma: 0, coop: true }], [], { Ds: 40, alpha: 2.4, gamma: 0.9, lim, range: 450 });
  assert.equal(r.modified, false); assert.deepEqual(r.a, [1, 0, 0]);
});
test('CBF gate ignores a fast but well-separated pass', () => {
  const r = cbfFilter([0, 0, 0], { p: [0, 0, 400], v: [20, 0, 0], sigma: 0 }, [{ p: [250, 300, 400], v: [-20, 0, 0], sigma: 0, coop: true }], [], { Ds: 40, alpha: 2.4, gamma: 0.9, lim, range: 450 });
  assert.equal(r.nActive, 0);
});
test('CBF filter brakes / deflects a head-on collision course', () => {
  const nb = [{ p: [180, 0, 400], v: [-20, 0, 0], sigma: 0, coop: false }];
  const r = cbfFilter([0, 0, 0], { p: [0, 0, 400], v: [20, 0, 0], sigma: 0 }, nb, [], { Ds: 40, alpha: 2.4, gamma: 0.9, lim, range: 450 });
  assert.equal(r.modified, true); assert.ok(r.a[0] < -1, 'decelerates: ' + r.a[0]);
  assert.ok(Math.hypot(...r.a.slice(0, 2)) <= lim.h + 1e-6);
});
test('CBF static barrier slows approach to an obstacle but ignores a receding one', () => {
  const p = { Ds: 40, alpha: 2.4, alphaStatic: 4, gamma: 0.9, lim, range: 450, dsStatic: 20 };
  const toward = cbfFilter([0, 0, 0], { p: [0, 0, 400], v: [20, 0, 0], sigma: 0 }, [], [{ n: [-1, 0, 0], d: 90 }], p); assert.ok(toward.a[0] < 0);
  const away = cbfFilter([0, 0, 0], { p: [0, 0, 400], v: [-20, 0, 0], sigma: 0 }, [], [{ n: [-1, 0, 0], d: 90 }], p); assert.equal(away.modified, false);
});
test('Dykstra projection satisfies half-spaces and reports infeasibility', () => {
  const ok = dykstra([5, 0, 0], [{ n: [1, 0, 0], b: 2 }], lim); assert.ok(ok.feasible); near(ok.a[0], 2, 1e-3);
  const bad = dykstra([0, 0, 0], [{ n: [1, 0, 0], b: -10 }, { n: [-1, 0, 0], b: -10 }], lim); assert.equal(bad.feasible, false);
});
