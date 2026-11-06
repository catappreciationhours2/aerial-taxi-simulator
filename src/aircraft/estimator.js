// Navigation filter with GNSS fault detection and isolation (FDI).
//
// Fused position uses a per-axis Kalman filter: INS-style prediction from measured velocity, GNSS correction.
// Two independent detectors compare the GNSS fix against what the vehicle already believes:
//   (a) normalised-innovation-squared (NIS) test against the fused estimate  -> catches step errors fast
//   (b) disagreement with a slowly aligned, pure dead-reckoned INS position  -> catches slow ramps/spoofing
// When either persists, GNSS is isolated and the filter coasts on dead reckoning with growing uncertainty.

import { SAFETY } from '../core/config.js';

export class NavFilter {
  constructor(p0, rng, sigmaG = 2.5) {
    this.p = [p0[0], p0[1], p0[2]]; // fused position estimate
    this.ins = [p0[0], p0[1], p0[2]]; // dead-reckoned position
    this.var = 1.0; // per-axis position variance
    this.insVar = 1.0;
    this.sigmaG = sigmaG;
    this.rng = rng;
    this.vBias = [0, 0, 0]; // INS velocity bias random walk
    this.isolated = false;
    this.fault = null; // {tDetect, kind}
    this.nisCount = 0; this.disCount = 0;
    this.bias = [0, 0, 0]; // injected GNSS bias (truth, unknown to the filter)
    this.noiseMult = 1; // sensor degradation multiplier
    this.denied = false; // GNSS outage
    this.lastNis = 0;
  }

  /** Inject a GNSS error: step bias vector, optional ramp (m/s growth reaching `bias` over rampS). */
  setBias(b, rampS = 0, t = 0) { this.biasTarget = [b[0], b[1], b[2]]; this.biasRampS = rampS; this.biasT0 = t; if (rampS <= 0) this.bias = [b[0], b[1], b[2]]; }
  clearBias() { this.bias = [0, 0, 0]; this.biasTarget = null; }

  /** Per-step INS propagation from truth velocity + sensor error. */
  predict(vTrue, dt) {
    const r = this.rng;
    for (let a = 0; a < 3; a++) {
      this.vBias[a] += 0.02 * Math.sqrt(dt) * r.gauss();
      const vm = vTrue[a] + this.vBias[a] + 0.08 * this.noiseMult * r.gauss();
      this.p[a] += vm * dt; this.ins[a] += vm * dt;
    }
    this.var += 0.04 * this.noiseMult * dt;
    this.insVar += 0.04 * dt + 0.0025 * dt; // random-walk bias unc.
  }

  /** GNSS update. Returns 'gps_fault' once on first detection, else null. */
  update(pTrue, t) {
    if (this.biasTarget && this.biasRampS > 0) {
      const f = Math.min(1, (t - this.biasT0) / this.biasRampS);
      this.bias = this.biasTarget.map((b) => b * f);
    }
    if (this.isolated || this.denied) return null;
    const r = this.rng, sg = this.sigmaG * this.noiseMult;
    const z = [0, 1, 2].map((a) => pTrue[a] + this.bias[a] + sg * r.gauss());
    const S = this.var + sg * sg;
    let nis = 0;
    for (let a = 0; a < 3; a++) nis += (z[a] - this.p[a]) ** 2 / S;
    this.lastNis = nis;
    let dis = 0;
    const Sd = this.insVar + sg * sg;
    for (let a = 0; a < 3; a++) dis += (z[a] - this.ins[a]) ** 2 / Sd;
    this.nisCount = nis > SAFETY.nisThreshold ? this.nisCount + 1 : Math.max(0, this.nisCount - 1);
    this.disCount = dis > 4 * SAFETY.nisThreshold ? this.disCount + 1 : Math.max(0, this.disCount - 1);
    if (this.nisCount >= 3 || this.disCount >= 5) {
      this.isolated = true;
      this.fault = { tDetect: t, kind: this.nisCount >= 3 ? 'innovation' : 'ins-disagreement' };
      // roll back toward dead reckoning: it is the better-aligned solution at this point
      for (let a = 0; a < 3; a++) this.p[a] = this.ins[a];
      this.var = this.insVar;
      return 'gps_fault';
    }
    const K = this.var / S;
    for (let a = 0; a < 3; a++) this.p[a] += K * (z[a] - this.p[a]);
    this.var *= 1 - K;
    // slow INS alignment (complementary)
    for (let a = 0; a < 3; a++) this.ins[a] += 0.01 * (z[a] - this.ins[a]);
    this.insVar *= 0.995;
    return null;
  }
  get sigma() { return Math.sqrt(this.var); }
  /** Restore after a fault clears (e.g. a repaired receiver). */
  reset(pTrue) {
    this.p = [...pTrue]; this.ins = [...pTrue]; this.var = 1; this.insVar = 1;
    this.isolated = false; this.fault = null; this.nisCount = this.disCount = 0; this.clearBias(); this.denied = false;
  }
}
