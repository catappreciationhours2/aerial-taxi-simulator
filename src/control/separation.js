// Separation geometry and probabilistic separation tests (research approximations, NOT certified standards).

import { normalCdf, normalInv } from '../core/util.js';
import { SAFETY } from '../core/config.js';

/**
 * Closest point of approach for two constant-velocity points over [0, tMax].
 * @returns {{t:number, d:number}}  time of closest approach and the minimum distance.
 */
export function closestApproach(pi, vi, pj, vj, tMax = Infinity) {
  const dx = pi[0] - pj[0], dy = pi[1] - pj[1], dz = pi[2] - pj[2];
  const wx = vi[0] - vj[0], wy = vi[1] - vj[1], wz = vi[2] - vj[2];
  const w2 = wx * wx + wy * wy + wz * wz;
  let t = w2 < 1e-12 ? 0 : -(dx * wx + dy * wy + dz * wz) / w2;
  t = Math.max(0, Math.min(tMax, t));
  return { t, d: Math.hypot(dx + wx * t, dy + wy * t, dz + wz * t) };
}

/**
 * Minimum distance between two piecewise-linear segments flown simultaneously on [t0, t1]
 * (continuous collision check: catches fast crossings that frame-sampling misses).
 */
export function segmentMinDistance(a0, a1, b0, b1, t0, t1) {
  const T = t1 - t0;
  if (T <= 0) return Math.hypot(a0[0] - b0[0], a0[1] - b0[1], a0[2] - b0[2]);
  const va = [0, 1, 2].map((i) => (a1[i] - a0[i]) / T), vb = [0, 1, 2].map((i) => (b1[i] - b0[i]) / T);
  return closestApproach(a0, va, b0, vb, T).d;
}

/**
 * Adaptive protected radius: d0 + v_close*tau + d_brake + z_delta*sigma  (see project plan section 5.2).
 * @param p {d0, vClose, latency, aBrake, sigma, delta}
 */
export function protectedRadius({ d0 = SAFETY.d0, 
  vClose = 0, 
  latency = SAFETY.latency, 
  aBrake = SAFETY.aBrake, 
  sigma = 0, 
  delta = SAFETY.deltaRisk, 
  rSum = 0, 
  includeBraking = true }) {
  const vc = Math.max(0, vClose);
  const dBrake = includeBraking ? (vc * vc) / (2 * aBrake) : 0;
  const z = Math.max(0, normalInv(1 - delta));
  return rSum + d0 + vc * latency + dBrake + z * sigma;
}

/** Relative-position std-dev along a unit direction n for independent Gaussians: sqrt(n^T (S_i + S_j) n). */
export function relativeSigmaAlong(n, sigmaI, sigmaJ) {
  // isotropic per-axis sigmas (a scalar each) -> variance along any direction is sum of variances
  return Math.sqrt(sigmaI * sigmaI + sigmaJ * sigmaJ);
}

/** Chance of violating separation R given mean distance d and relative sigma s (1-D along the line of sight). */
export function lossProbability(d, R, s) {
  if (s <= 1e-9) return d < R ? 1 : 0;
  return normalCdf((R - d) / s) - normalCdf((-R - d) / s);
}

/** Mean separation required so that P(loss of separation R) <= delta. */
export function requiredMeanSeparation(R, s, delta = SAFETY.deltaRisk) {
  return R + Math.max(0, normalInv(1 - delta)) * s;
}
