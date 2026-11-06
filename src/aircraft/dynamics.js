// Point-mass translational dynamics with actuator limits, aerodynamic drag in the AIR-relative frame,
// an airspeed envelope, and battery drain.
//
//   m dv/dt = F_thrust + F_drag + m g,   F_drag = -1/2 rho CdA |v - W| (v - W)
//   ||F_thrust|| <= T_max (thrust-to-weight limit), airspeed <= V_max(health)
// The controller commands a net acceleration `a` (what the plan wants beyond gravity); drag is an
// unmodelled disturbance the controller must reject. Wind enters through drag and the airspeed cap.

import { G } from '../core/config.js';
import { powerKW } from './power.js';

/** Saturate a commanded acceleration so that the implied thrust vector respects T_max and the axis limits. */
export function limitAccel(a, spec, thrustFactor = 1, out = [0, 0, 0]) {
  const tf = Math.max(0.05, thrustFactor);
  let ax = a[0], ay = a[1], az = a[2];
  const ah = Math.hypot(ax, ay), ahMax = spec.aHorizMax * tf;
  if (ah > ahMax) { const s = ahMax / ah; ax *= s; ay *= s; }
  az = Math.min(spec.aUpMax * tf, Math.max(-spec.aDownMax, az));
  // thrust ball: ||(ax, ay, az + g)|| <= tw * g * tf
  const tx = ax, ty = ay, tz = az + G, mag = Math.hypot(tx, ty, tz), lim = spec.thrustToWeightMax * G * tf;
  if (mag > lim) { const s = lim / mag; ax = tx * s; ay = ty * s; az = tz * s - G; }
  out[0] = ax; out[1] = ay; out[2] = az;
  return out;
}

/** Max airspeed available given propulsion health. */
export const vMaxEff = (spec, thrustFactor) => spec.vMax * (0.35 + 0.65 * Math.min(1, thrustFactor));

/**
 * Advance the plant one step. Mutates ac.p, ac.v, ac.energy (kWh), ac.powerKw, ac.airspeed.
 * @returns {number} electrical power (kW)
 */
export function stepPlant(ac, wind, dt) {
  const spec = ac.spec, h = ac.health;
  const a = limitAccel(ac.aCmd, spec, h.thrustFactor, ac._aSat || (ac._aSat = [0, 0, 0]));
  ac.aApplied = a;
  const vr = [ac.v[0] - wind[0], ac.v[1] - wind[1], ac.v[2] - wind[2]];
  const sp = Math.hypot(vr[0], vr[1], vr[2]);
  const k = spec.kDrag * sp;
  const nvx = ac.v[0] + (a[0] - k * vr[0]) * dt;
  const nvy = ac.v[1] + (a[1] - k * vr[1]) * dt;
  const nvz = ac.v[2] + (a[2] - k * vr[2]) * dt;
  ac.p[0] += 0.5 * (ac.v[0] + nvx) * dt;
  ac.p[1] += 0.5 * (ac.v[1] + nvy) * dt;
  ac.p[2] += 0.5 * (ac.v[2] + nvz) * dt;
  ac.v[0] = nvx; ac.v[1] = nvy; ac.v[2] = nvz;
  // airspeed envelope: the vehicle cannot push through the air faster than vMax
  const ux = ac.v[0] - wind[0], uy = ac.v[1] - wind[1], uz = ac.v[2] - wind[2];
  const us = Math.hypot(ux, uy, uz), vm = vMaxEff(spec, h.thrustFactor);
  if (us > vm) { const s = vm / us; ac.v[0] = wind[0] + ux * s; ac.v[1] = wind[1] + uy * s; ac.v[2] = wind[2] + uz * s; }
  ac.airspeed = Math.min(us, vm);
  const P = powerKW(spec, ac.airspeed, ac.v[2], h.thrustFactor, h.drainMult);
  ac.powerKw = P;
  ac.energy -= (P * dt) / 3600;
  const sh = Math.hypot(ac.v[0], ac.v[1]);
  if (sh > 2) { // heading follows ground track, limited yaw rate
    const want = Math.atan2(ac.v[0], ac.v[1]);
    let d = want - ac.heading; d = Math.atan2(Math.sin(d), Math.cos(d));
    const lim = spec.yawRateMax * dt;
    ac.heading += Math.max(-lim, Math.min(lim, d));
  }
  return P;
}
