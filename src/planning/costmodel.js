// Physics-aware edge model for the lattice planner.
//
// For a horizontal edge with unit direction u and wind W (planner's belief), a vehicle holding air speed Va
// achieves ground speed g along u when  |g u - W| = Va, i.e.
//        g = W.u + sqrt(Va^2 - |W_perp|^2),     W_perp = W - (W.u) u      (infeasible if |W_perp| >= Va)
// The planner evaluates a few candidate air speeds per edge and keeps the cheapest under
//        J = w_t * T + w_e * E[kWh] + w_r * R + w_n * N                      (seconds-equivalent units)
// Gust margins are applied conservatively: headwind is increased and crosswind inflated by k*sigma.

import { PLANNER } from '../core/config.js';
import { powerKW } from '../aircraft/power.js';

const SIG_REF = 2.0; // m/s turbulence at which risk exposure equals elapsed time

export class CostModel {
  /**
   * @param world  World
   * @param wind   EstimatedWind (wind.at / wind.sigma / wind.speedBound)
   * @param spec   aircraft spec
   * @param features live feature-flag object (physicsAware, noiseAware)
   */
  constructor(world, wind, spec, features, weights = PLANNER.weights) {
    this.world = world; this.wind = wind; this.spec = spec; this.features = features; this.w = weights;
    this.cx = new Float32Array(world.layer); this.cy = new Float32Array(world.layer);
    for (let i = 0; i < world.layer; i++) { this.cx[i] = ((i % world.nx) + 0.5) * world.cell; this.cy[i] = (Math.floor(i / world.nx) + 0.5) * world.cell; }
    this.pCand = PLANNER.speedCandidates.map((c) => powerKW(spec, c * spec.vCruise, 0));
    this.pCruise = powerKW(spec, spec.vCruise, 0);
    this.pClimb = powerKW(spec, 0, spec.vClimb);
    this.pDesc = powerKW(spec, 0, -spec.vDescend);
    this.pHover = spec.pHoverKw;
    this._tmp = [0, 0, 0];
    this.bounds = null; this.boundsKey = ''; this.boundsBuiltAt = -1e9;
    this._ec = new Map(); this._ecVer = -1;
    this.vaMaxPlan = Math.min(spec.vMax * PLANNER.windSafety, Math.max(...PLANNER.speedCandidates) * spec.vCruise);
  }

  /** Lower bound on cost per second of flight (time + minimum possible energy). */
  get lbRate() { return this.w.time + (this.w.energy * this.spec.pMinKw) / 3600; }

  holdCost(dt) { return dt <= 0 ? 0 : this.w.time * dt + (this.w.energy * this.pHover * dt) / 3600; }
  holdEnergy(dt) { return (this.pHover * dt) / 3600; }

  /** Is the horizontal move (dx,dy) from node n allowed w.r.t. static blockage (incl. both corner cells for diagonals)? */
  staticCorners(n, dx, dy) {
    const w = this.world, L = w.layer, nx = w.nx;
    const iz = (n / L) | 0, c = n % L, ix = c % nx, iy = (c - ix) / nx;
    const jx = ix + dx, jy = iy + dy;
    if (jx < 0 || jy < 0 || jx >= nx || jy >= w.ny) return null;
    const m = iz * L + jy * nx + jx;
    if (w.blocked[m]) return null;
    if (dx !== 0 && dy !== 0) {
      const c1 = iz * L + iy * nx + jx, c2 = iz * L + jy * nx + ix;
      if (w.blocked[c1] || w.blocked[c2]) return null;
      return { m, corners: [c1, c2] };
    }
    return { m, corners: null };
  }

  /**
   * Evaluate edge n->m departing at time t. Fills `out` and returns it; out.ok=false if infeasible.
   * out: {ok, dur, energy, risk, noise, cost, va, g}
   */
  evalEdge(n, m, t, out) {
    const w = this.world, L = w.layer, spec = this.spec, W = this.w;
    const cn = n % L, cm = m % L, zn = (n / L) | 0, zm = (m / L) | 0;
    out.ok = false;
    if (cn === cm) { // vertical (level change at a fixed cell)
      const dz = w.levels[zm] - w.levels[zn], d = Math.abs(dz), up = dz > 0;
      const vz = up ? spec.vClimb : spec.vDescend, dur = d / vz;
      const P = up ? this.pClimb : this.pDesc;
      const x = this.cx[cn], y = this.cy[cn], zmid = 0.5 * (w.levels[zm] + w.levels[zn]);
      const sg = this.wind.sigma(x, y, zmid, t + dur / 2);
      out.ok = true; out.dur = dur; out.energy = (P * dur) / 3600; out.va = 0; out.g = vz;
      out.risk = ((sg / SIG_REF) ** 2) * dur;
      out.noise = this.features.noiseAware ? w.noiseSens[cn] * 1.3 * (P / this.pCruise) * dur * 0.5 : 0;
      out.cost = W.time * dur + W.energy * out.energy + W.risk * out.risk + W.noise * out.noise;
      return out;
    }
    const x0 = this.cx[cn], y0 = this.cy[cn], x1 = this.cx[cm], y1 = this.cy[cm];
    const dx = x1 - x0, dy = y1 - y0, d = Math.hypot(dx, dy);
    const ux = dx / d, uy = dy / d, z = w.levels[zn];
    const tm = t + (0.5 * d) / spec.vCruise;
    // Wind/uncertainty memo: the same edge is evaluated many times by different searches at nearly the same time, and the
    // wind estimate is smooth over 10 s, so we cache (edge, 10 s bucket) -> [wx, wy, sigma]; invalidated on wind/world change.
    const ver = this.wind.version * 1000003 + w.version;
    if (ver !== this._ecVer || this._ec.size > 1.5e6) { this._ec.clear(); this._ecVer = ver; }
    const ek = (Math.floor(tm / 10) * w.nNodes + n) * w.nNodes + m;
    let ev = this._ec.get(ek);
    if (!ev) {
      const mx = 0.5 * (x0 + x1), my = 0.5 * (y0 + y1);
      const wv = this.wind.at(mx, my, z, tm, this._tmp);
      ev = [wv[0], wv[1], this.wind.sigma(mx, my, z, tm)];
      this._ec.set(ek, ev);
    }
    const Wv = ev, sg = ev[2];
    const wAlong0 = Wv[0] * ux + Wv[1] * uy;
    const wPerp0 = Math.sqrt(Math.max(0, Wv[0] * Wv[0] + Wv[1] * Wv[1] - wAlong0 * wAlong0));
    const wAlong = wAlong0 - PLANNER.gustSigmaAlong * sg;
    const wPerp = wPerp0 + PLANNER.gustSigmaCross * sg;
    const phys = this.features.physicsAware;
    const cands = phys ? PLANNER.speedCandidates : [1];
    let best = Infinity;
    const sens = w.noiseSens[w.cellIndex(0.5 * (x0 + x1), 0.5 * (y0 + y1))];
    const agl = Math.max(30, z - w.terrainFast(0.5 * (x0 + x1), 0.5 * (y0 + y1)));
    for (let k = 0; k < cands.length; k++) {
      const va = cands[k] * spec.vCruise;
      if (va > spec.vMax * PLANNER.windSafety) continue;
      if (wPerp >= 0.97 * va) continue;
      const g = wAlong + Math.sqrt(va * va - wPerp * wPerp);
      if (g < PLANNER.minGroundSpeed) continue;
      const dur = d / g;
      const P = phys ? this.pCand[PLANNER.speedCandidates.indexOf(cands[k])] : this.pCruise;
      const E = (P * dur) / 3600;
      const risk = ((sg / SIG_REF) ** 2) * dur;
      const noise = this.features.noiseAware ? sens * (P / this.pCruise) * dur / (1 + agl / 150) ** 2 : 0;
      const cost = W.time * dur + W.energy * E + W.risk * risk + W.noise * noise;
      if (cost < best) { best = cost; out.ok = true; out.dur = dur; out.energy = E; out.risk = risk; out.noise = noise; out.cost = cost; out.va = va; out.g = g; }
    }
    return out;
  }

  /**
   * Per-node upper bound on |mean wind| over a time window, used for the relaxed (admissible) heuristic.
   * Rebuilt lazily when the wind estimate changes materially.
   */
  windBounds(t) {
    const key = `${this.wind.version}|${this.features.windAware}|${this.features.fleetWindMap}|${this.world.version}`;
    if (this.bounds && (this.boundsKey === key || t - this.boundsBuiltAt < 180) && t - this.boundsBuiltAt < 1200) return this.bounds;
    const w = this.world, B = new Float32Array(w.nNodes);
    const slack = this.features.fleetWindMap ? 2.0 : 0.5;
    if (this.features.windAware) {
      for (let n = 0; n < w.nNodes; n++) {
        const c = n % w.layer;
        B[n] = this.wind.speedBound(this.cx[c], this.cy[c], w.levels[(n / w.layer) | 0], t, t + 1800, this._tmp) + slack;
      }
    }
    this.bounds = B; this.boundsKey = key; this.boundsBuiltAt = t;
    return B;
  }
}
