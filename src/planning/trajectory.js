// Time-parameterised reference trajectory built from SIPP output, plus its space-time footprint.

import { PLANNER } from '../core/config.js';

export class Trajectory {
  constructor() { this.segs = []; this.landed = false; this.energyPlanned = 0; }

  get t0() { return this.segs.length ? this.segs[0].t0 : 0; }
  get t1() { return this.segs.length ? this.segs[this.segs.length - 1].t1 : 0; }
  get empty() { return this.segs.length === 0; }

  push(seg) { this.segs.push(seg); this.energyPlanned += seg.energy || 0; return this; }

  /** Index of the segment active at time t (clamped). */
  indexAt(t) {
    const s = this.segs;
    if (!s.length) return -1;
    if (t <= s[0].t0) return 0;
    let lo = 0, hi = s.length - 1;
    while (lo < hi) { const mid = (lo + hi + 1) >> 1; if (s[mid].t0 <= t) lo = mid; else hi = mid - 1; }
    return lo;
  }

  /**
   * Dynamically feasible reference: the raw SIPP path is piecewise linear in time, i.e. its velocity jumps at every
   * node (rest -> cruise, 45 deg turns), which no vehicle with bounded acceleration can track.
   * We take the running mean of the raw path over a window W (a box filter, integrated exactly for piecewise-linear
   * input). The velocity then ramps linearly over W so |a| = |dv|/W, and the position deviates from the raw path by at
   * most ~|dv|*W/8 (about 15-20 m at W = 6 s), which the reservation padding absorbs.
   * Box-filter smoothing of a waypoint path is standard practice; see e.g. Mellinger & Kumar 2011 for the
   * alternative of minimum-snap polynomials, which we do not need because the lattice path is already collision-free.
   */
  ref(t, p = [0, 0, 0], v = [0, 0, 0]) {
    const W = PLANNER.refSmoothS;
    const r = this.rawRef(t, p, v);
    if (!r || !(W > 0) || this.segs.length === 0) return r;
    const a = t - W / 2, b = t + W / 2;
    const tmp = [0, 0, 0], pa = [0, 0, 0], pb = [0, 0, 0];
    // breakpoints inside (a, b)
    let acc0 = 0, acc1 = 0, acc2 = 0, prevT = a;
    this._rawPos(a, pa);
    let prev = [pa[0], pa[1], pa[2]];
    let i = Math.max(0, this.indexAt(a));
    const pts = [];
    for (let k = i; k < this.segs.length; k++) { const s = this.segs[k]; if (s.t0 > a && s.t0 < b) pts.push(s.t0); if (s.t1 > a && s.t1 < b) pts.push(s.t1); if (s.t0 >= b) break; }
    pts.push(b);
    pts.sort((x, y) => x - y);
    for (const tb of pts) {
      if (tb <= prevT) continue;
      this._rawPos(tb, tmp);
      const dt = tb - prevT;
      acc0 += 0.5 * (prev[0] + tmp[0]) * dt; acc1 += 0.5 * (prev[1] + tmp[1]) * dt; acc2 += 0.5 * (prev[2] + tmp[2]) * dt;
      prev[0] = tmp[0]; prev[1] = tmp[1]; prev[2] = tmp[2]; prevT = tb;
    }
    p[0] = acc0 / W; p[1] = acc1 / W; p[2] = acc2 / W;
    v[0] = (prev[0] - pa[0]) / W; v[1] = (prev[1] - pa[1]) / W; v[2] = (prev[2] - pa[2]) / W;
    return { p, v, seg: r.seg };
  }

  /** Raw (unsmoothed) position with end clamping. */
  _rawPos(t, out) {
    const r = this.rawRef(t, out, [0, 0, 0]);
    return r ? out : null;
  }

  /** Raw piecewise-linear reference position and velocity at time t. */
  rawRef(t, p = [0, 0, 0], v = [0, 0, 0]) {
    const i = this.indexAt(t);
    if (i < 0) return null;
    const s = this.segs[i];
    const T = s.t1 - s.t0;
    if (t <= s.t0 || T <= 1e-9) { p[0] = s.p0[0]; p[1] = s.p0[1]; p[2] = s.p0[2]; v[0] = v[1] = v[2] = 0; if (T > 1e-9 && s.kind !== 'hold') { v[0] = (s.p1[0] - s.p0[0]) / T; v[1] = (s.p1[1] - s.p0[1]) / T; v[2] = (s.p1[2] - s.p0[2]) / T; } return { p, v, seg: s }; }
    if (t >= s.t1) { p[0] = s.p1[0]; p[1] = s.p1[1]; p[2] = s.p1[2]; v[0] = v[1] = v[2] = 0; return { p, v, seg: s }; }
    const f = (t - s.t0) / T;
    p[0] = s.p0[0] + (s.p1[0] - s.p0[0]) * f; p[1] = s.p0[1] + (s.p1[1] - s.p0[1]) * f; p[2] = s.p0[2] + (s.p1[2] - s.p0[2]) * f;
    v[0] = (s.p1[0] - s.p0[0]) / T; v[1] = (s.p1[1] - s.p0[1]) / T; v[2] = (s.p1[2] - s.p0[2]) / T;
    return { p, v, seg: s };
  }

  /**
   * Freeze everything up to the end of the segment containing `tCommit`.
   * Returns {prefix, node, t} where node/t is where new planning should start, or null if the trajectory
   * is already in its landing phase or has ended.
   */
  commitPrefix(tCommit) {
    const i = this.indexAt(tCommit);
    if (i < 0) return null;
    const s = this.segs[i];
    if (s.kind === 'land') return null;
    const prefix = new Trajectory();
    for (let k = 0; k < i; k++) prefix.push(this.segs[k]);
    let node, t;
    if (s.kind === 'hold') {
      const tc = Math.max(s.t0, Math.min(s.t1, tCommit));
      if (tc > s.t0 + 1e-9) prefix.push({ ...s, t1: tc, energy: (s.energy || 0) * ((tc - s.t0) / Math.max(1e-9, s.t1 - s.t0)) });
      node = s.n0; t = tc;
    } else {
      prefix.push(s); node = s.n1; t = s.t1;
    }
    if (s.kind === 'takeoff') { node = s.n1; t = s.t1; }
    return { prefix, node, t };
  }

  /** Space-time footprint: when each node is occupied (arrival of move start .. end of the departing move). */
  visits() {
    const out = [];
    let cur = null;
    for (const s of this.segs) {
      if (s.kind === 'takeoff') { cur = { node: s.n1, tIn: s.t0, tOut: s.t1 }; out.push(cur); continue; }
      if (s.kind === 'move' || s.kind === 'vmove') {
        if (cur) cur.tOut = Math.max(cur.tOut, s.t1);
        cur = { node: s.n1, tIn: s.t0, tOut: s.t1 };
        out.push(cur);
        if (s.corners) for (const c of s.corners) out.push({ node: c, tIn: s.t0, tOut: s.t1, corner: true });
      } else if (cur) { cur.tOut = Math.max(cur.tOut, s.t1); }
    }
    return out;
  }

  /** Total planned energy (kWh) and arrival time. */
  get eta() { return this.t1; }
}

export const padAt = (t, tPlan, mult = 1) =>
  Math.min(PLANNER.padMax * mult, (PLANNER.padBase + PLANNER.padGrow * Math.max(0, t - tPlan)) * mult);

/**
 * Register a trajectory's footprint in the reservation table.
 * @param inflate neighbour inflation in cells (uncertainty-aware envelopes); padMult scales time padding.
 */
export function reserveTrajectory(table, world, id, traj, tPlan, { inflate = 0, padMult = 1, fromT = -Infinity } = {}) {
  let n = 0;
  for (const v of traj.visits()) {
    if (v.tOut < fromT) continue;
    const padS = padAt(v.tIn, tPlan, padMult), padE = padAt(v.tOut, tPlan, padMult);
    const s = v.tIn - padS, e = v.tOut + padE;
    table.add(id, v.node, s, e); n++;
    if (inflate > 0) {
      const [ix, iy, iz] = world.decode(v.node);
      for (let dy = -inflate; dy <= inflate; dy++) for (let dx = -inflate; dx <= inflate; dx++) {
        if (!dx && !dy) continue;
        const jx = ix + dx, jy = iy + dy;
        if (jx < 0 || jy < 0 || jx >= world.nx || jy >= world.ny) continue;
        table.add(id, world.idx(jx, jy, iz), s, e); n++;
      }
    }
  }
  return n;
}
