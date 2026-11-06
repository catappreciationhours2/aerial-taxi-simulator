// Aircraft specification and electrical power model.
//
// Power is built from rotor momentum theory (actuator disc), the standard first-order helicopter model:
//   hover induced velocity       v_h  = sqrt(W / (2 rho A))
//   forward-flight induced ratio vi/vh = sqrt( -mu^2/2 + sqrt(mu^4/4 + 1) ),  mu = V / v_h
//   adapted from: J. G. Leishman, "Principles of Helicopter Aerodynamics", 2nd ed., Cambridge Univ. Press, 2006, ch. 2-5.
// Parasite drag power = 1/2 rho CdA V^3 and climb power = W * Vz are added; profile power is a constant fraction.
// This is a research-grade approximation of a *multicopter* eVTOL, not a certified performance model.

import { AIRCRAFT_CLASSES, AIRCRAFT_COMMON, G, RHO } from '../core/config.js';

export function makeSpec(classKey = 'standard', overrides = {}) {
  const k = AIRCRAFT_CLASSES[classKey];
  if (!k) throw new Error(`unknown aircraft class ${classKey}`);
  const spec = { ...AIRCRAFT_COMMON, ...k, ...overrides };
  spec.weight = spec.mass * G;
  spec.vHover = Math.sqrt(spec.weight / (2 * RHO * spec.discArea));
  spec.pIdealHoverW = spec.weight * spec.vHover;
  spec.pHoverShaftW = spec.pIdealHoverW / spec.figureOfMerit;
  spec.pProfileW = spec.profilePowerFrac * spec.pHoverShaftW;
  spec.pIndHoverW = spec.pHoverShaftW - spec.pProfileW;
  spec.kDrag = (0.5 * RHO * spec.cdA) / spec.mass; // (1/m): drag accel = -kDrag |v| v
  spec.usableKwh = spec.capacityKwh * spec.usableFraction;
  spec.reserveKwh = spec.usableKwh * spec.reserveFraction;
  spec.pHoverKw = powerKW(spec, 0, 0);
  // minimum over level-flight speeds: lower bound used in admissible heuristics
  let pmin = Infinity, vBest = 0;
  for (let v = 0; v <= spec.vMax * 1.01; v += 0.5) {
    const p = powerKW(spec, v, 0);
    if (p < pmin) { pmin = p; vBest = v; }
  }
  spec.pMinKw = pmin; spec.vBestEndurance = vBest;
  return spec;
}

/**
 * Electrical power draw in kW.
 * @param spec      from makeSpec
 * @param vAir      air-relative speed (m/s)
 * @param vz        vertical speed (m/s, +up)
 * @param thrustFactor  1 = healthy, <1 degraded propulsion (costs extra power)
 * @param drainMult battery fault multiplier (>=1)
 */
export function powerKW(spec, vAir, vz = 0, thrustFactor = 1, drainMult = 1) {
  const mu = vAir / spec.vHover;
  const viRatio = Math.sqrt(-0.5 * mu * mu + Math.sqrt(0.25 * mu ** 4 + 1));
  let pInd = spec.pIndHoverW * viRatio;
  let pClimb = 0;
  if (vz > 0) pClimb = spec.weight * vz;
  else if (vz < 0) pInd *= 1 - 0.18 * Math.min(1, -vz / spec.vHover); // mild descent unloading, far from vortex-ring state
  const pPar = 0.5 * RHO * spec.cdA * vAir ** 3;
  const pShaft = pInd + spec.pProfileW + pPar + pClimb;
  const degrade = 1 + 0.6 * (1 - Math.max(0.2, thrustFactor));
  return (pShaft / spec.driveEfficiency / 1000) * degrade * drainMult;
}

/** Energy (kWh) for flying at constant air speed for dt seconds. */
export const energyKWh = (spec, vAir, vz, dt, tf = 1, dm = 1) => (powerKW(spec, vAir, vz, tf, dm) * dt) / 3600;

/** Level-flight endurance at hover from a given energy (s). */
export const hoverEndurance = (spec, kwh) => (kwh * 3600) / spec.pHoverKw;
