// A single aerial taxi: state, health, navigation filter, and the tactical control stack
// (MPC tracking -> CBF safety filter -> actuator limits), plus its runtime-assurance mode.

import { SAFETY, SIM } from '../core/config.js';
import { makeSpec } from './power.js';
import { NavFilter } from './estimator.js';
import { solveMPC, pdTrack } from '../control/mpc.js';
import { cbfFilter } from '../control/safety_filter.js';
import { vMaxEff } from './dynamics.js';
import { hashStr } from '../core/util.js';

export const PHASE = { GROUND: 'ground', FLYING: 'flying', GROUNDED: 'grounded', CRASHED: 'crashed' };
export const MODE = { NOMINAL: 'NOMINAL', DEGRADED: 'DEGRADED', EMERGENCY: 'EMERGENCY' };

export class Aircraft {
  constructor(id, classKey, site, rng, energyFrac = 1) {
    this.id = id;
    this.name = `AC-${String(id + 1).padStart(3, '0')}`;
    this.spec = makeSpec(classKey);
    this.rng = rng;
    this.siteId = site.id;
    this.destId = null;
    this.p = [site.x, site.y, site.padElev];
    this.v = [0, 0, 0];
    this.aCmd = [0, 0, 0]; this.aApplied = [0, 0, 0]; this.aPrev = [0, 0, 0];
    this.heading = rng.range(0, Math.PI * 2);
    this.energy = this.spec.usableKwh * energyFrac;
    this.powerKw = 0; this.airspeed = 0;
    this.phase = PHASE.GROUND;
    this.mode = MODE.NOMINAL;
    this.plan = null; this.flown = null;
    this.health = { thrustFactor: 1, drainMult: 1, commLoss: false, gps: 'ok', sensor: 'ok' };
    this.nav = new NavFilter(this.p, rng.fork('nav' + id), this.spec.gnssSigma);
    this.sigmaPos = 1; this.padMult = 1;
    this.frozen = false; this.emergencyMode = false; this.forced = false;
    this.mpcWarm = null; this.dHat = [0, 0, 0]; this._vPrev = null;
    this.extraAccel = null; // injected disturbance {a:[3], until}
    this.devTime = 0; this.cbfBadTime = 0; this.mpcBadTime = 0;
    this.lastHeartbeat = 0; this.nextTry = 0;
    this.mission = null; // {t0, from, to, ideal, e0}
    this.status = 'parked';
    this.cbfActive = false; this.mpcSat = false; this.trackErr = 0;
    this.trail = [];
    this.peakAccel = 0; this.jerkSq = 0; this.jerkN = 0;
    this.groundedSince = 0;
    this.hue = hashStr(this.name) % 360;
    this.nbrCount = 0;
  }

  get soc() { return Math.max(0, this.energy / this.spec.usableKwh); }
  get airborne() { return this.phase === PHASE.FLYING; }
  get radius() { return this.spec.radius; }

  /** Reference trajectory the aircraft actually flies (differs from the coordinator's plan during comm loss). */
  activePlan() { return this.flown || this.plan; }

  /**
   * One control update (called at 5 Hz). `ctx`: {t, dt, features, world, neighbors:[{p,v,sigma,coop}]}
   * Writes this.aCmd.
   */
  control(ctx) {
    const { t, features, world } = ctx;
    const spec = this.spec, tf = this.health.thrustFactor;
    this.flown = this.health.commLoss ? (this.flown || this.plan) : this.plan;
    const plan = this.flown;
    const pEst = this.nav.p;
    const vMeas = [this.v[0] + 0.05 * this.rng.gauss(), this.v[1] + 0.05 * this.rng.gauss(), this.v[2] + 0.05 * this.rng.gauss()];
    // disturbance observer (unmodelled acceleration = drag mismatch, gusts, injected faults)
    if (this._vPrev) {
      for (let i = 0; i < 3; i++) {
        const dMeas = (this.v[i] - this._vPrev[i]) / ctx.dt - this.aApplied[i];
        this.dHat[i] = 0.8 * this.dHat[i] + 0.2 * dMeas;
      }
    }
    this._vPrev = [this.v[0], this.v[1], this.v[2]];
    const lim = { h: Math.max(0.8, spec.aHorizMax * tf * (features.mpc ? 0.9 : 1)), up: Math.max(0.3, spec.aUpMax * tf * 0.9), dn: spec.aDownMax * 0.9 };
    let aNom;
    let seg = null;
    if (!plan || plan.empty) {
      aNom = [-0.9 * this.v[0], -0.9 * this.v[1], -0.9 * this.v[2]];
    } else if (features.mpc) {
      const dt = SAFETY.mpcDt;
      const tmpP = [0, 0, 0], tmpV = [0, 0, 0];
      const first = plan.ref(t, tmpP, tmpV); seg = first ? first.seg : null;
      const refFn = (k) => { const r = plan.ref(t + k * dt, [0, 0, 0], [0, 0, 0]); return { p: r.p, v: r.v }; };
      const d = [this.dHat[0], this.dHat[1], this.dHat[2]];
      const sol = solveMPC({ p: pEst, v: vMeas }, refFn, lim, d, this.mpcWarm, this.aPrev, { H: SAFETY.mpcHorizon, dt });
      // shift warm start by one step
      const w = new Float64Array(sol.u.length); w.set(sol.u.subarray(3)); w.set(sol.u.subarray(sol.u.length - 3), sol.u.length - 3);
      this.mpcWarm = w;
      aNom = sol.a0;
      this.mpcSat = sol.saturated && sol.terminalErr > 45;
      this.mpcBadTime = this.mpcSat ? this.mpcBadTime + ctx.dt : Math.max(0, this.mpcBadTime - ctx.dt);
    } else {
      const r = plan.ref(t, [0, 0, 0], [0, 0, 0]); seg = r.seg;
      aNom = pdTrack({ p: pEst, v: vMeas }, r, lim);
    }
    aNom[0] -= 0; aNom[1] -= 0;
    this.trackErr = plan && !plan.empty ? (() => { const r = plan.ref(t, [0, 0, 0], [0, 0, 0]); return Math.hypot(this.p[0] - r.p[0], this.p[1] - r.p[1], this.p[2] - r.p[2]); })() : 0;

    // ---- safety filter
    let a = aNom;
    this.cbfActive = false;
    if (features.cbfFilter) {
      const adaptive = features.adaptiveEnvelope;
      const own = { p: pEst, v: this.v, sigma: adaptive ? this.nav.sigma : 0 };
      const statics = [];
      if (!seg || (seg.kind !== 'takeoff' && seg.kind !== 'land')) {
        const sd = world.sdfAt(pEst[0], pEst[1], pEst[2]);
        if (sd < 300) {
          const g = world.sdfGrad(pEst[0], pEst[1], pEst[2]);
          const dEff = sd; // SDF is already metres to the obstacle surface; vehicle radius is part of dsStatic
          if (dEff < 250) statics.push({ n: g, d: Math.max(0.5, dEff) });
        }
      }
      // The filter may never command descent into the ground: near the surface it can only act horizontally/upwards (landing excepted).
      const surfZ = world.terrainFast(pEst[0], pEst[1]) + world.bldg[world.cellIndex(pEst[0], pEst[1])];
      const noDescent = !(seg && seg.kind === 'land') && this.p[2] - surfZ < 18;
      const nbs = ctx.neighbors.map((n) => ({ ...n, sigma: adaptive ? n.sigma : 0 }));
      const Ds = adaptive ? SAFETY.d0 + this.radius : SAFETY.d0 * 0.8 + this.radius;
      const r = cbfFilter(aNom, own, nbs, statics, { Ds, alpha: SAFETY.aBrake * Math.max(0.5, tf), alphaStatic: 0.8 * spec.aHorizMax * Math.max(0.5, tf), dsStatic: SAFETY.dStatic + this.radius, gamma: SAFETY.cbfGamma, lim: { h: spec.aHorizMax * tf, up: spec.aUpMax * tf, dn: noDescent ? 0 : spec.aDownMax }, range: SAFETY.cbfRange });
      a = r.a;
      this.cbfActive = r.modified;
      this.cbfBadTime = r.feasible ? Math.max(0, this.cbfBadTime - ctx.dt) : this.cbfBadTime + ctx.dt;
    }
    this.aCmd[0] = a[0]; this.aCmd[1] = a[1]; this.aCmd[2] = a[2];
    // comfort metrics
    const am = Math.hypot(this.aApplied[0], this.aApplied[1], this.aApplied[2]);
    if (am > this.peakAccel) this.peakAccel = am;
    const jx = (this.aCmd[0] - this.aPrev[0]) / ctx.dt, jy = (this.aCmd[1] - this.aPrev[1]) / ctx.dt, jz = (this.aCmd[2] - this.aPrev[2]) / ctx.dt;
    this.jerkSq += jx * jx + jy * jy + jz * jz; this.jerkN++;
    this.aPrev = [this.aCmd[0], this.aCmd[1], this.aCmd[2]];
  }

  /** Effective top speed given health. */
  vMax() { return vMaxEff(this.spec, this.health.thrustFactor); }

  /** Reset a repaired / recovered vehicle. */
  repair() {
    this.health = { thrustFactor: 1, drainMult: 1, commLoss: false, gps: 'ok', sensor: 'ok' };
    this.nav.reset(this.p); this.nav.noiseMult = 1;
    this.mode = MODE.NOMINAL; this.padMult = 1; this.sigmaPos = 1; this.frozen = false; this.emergencyMode = false; this.forced = false;
    this.extraAccel = null; this.devTime = 0; this.cbfBadTime = 0; this.mpcBadTime = 0;
  }
}

export { SIM };
