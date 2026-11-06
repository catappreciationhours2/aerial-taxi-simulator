// Wind models.
//   TrueWind        - the "ground truth" the physics simulator flies in (profile + terrain + channelling + gusts)
//   ForecastWind    - coarse forecast with deliberate bias / error, as from a numerical weather model
//   FleetWindMap    - Kalman-style spatiotemporal residual map learned from fleet observations
//   EstimatedWind   - what the planner actually uses: forecast (+ fleet correction), with uncertainty
//
// Wind vectors are [east, north, up] in m/s. Direction convention follows meteorology: "dirFrom" is the
// compass bearing the wind blows FROM.

import { DEG, clamp, smoothstep, makeNoise } from '../core/util.js';

const windVec = (speed, dirFromDeg) => [-speed * Math.sin(dirFromDeg * DEG), -speed * Math.cos(dirFromDeg * DEG)];

export class TrueWind {
  constructor(world, seed = 1, opts = {}) {
    this.world = world;
    this.speed10 = opts.speed10 ?? 3.5;
    this.dirFrom = opts.dirFrom ?? 250;
    this.gustScale = opts.gust ?? 1;
    this.turbulence = true;
    this.alpha = 0.27; // urban power-law exponent
    this.storm = null; // {t0, ramp, dur, mult}
    this.disturbances = []; // injected gust cells
    this._n1 = makeNoise(seed * 7 + 1); this._n2 = makeNoise(seed * 7 + 2); this._n3 = makeNoise(seed * 7 + 3);
    this._nb = makeNoise(seed * 7 + 4);
    this._buildFields();
  }

  _buildFields() {
    const w = this.world, { nx, ny } = w;
    this.ridge = new Float32Array(w.layer);
    this.bBlur = new Float32Array(w.layer);
    for (let iy = 0; iy < ny; iy++) for (let ix = 0; ix < nx; ix++) {
      let sg = 0, sb = 0, n = 0;
      for (let dy = -3; dy <= 3; dy++) for (let dx = -3; dx <= 3; dx++) {
        const jx = clamp(ix + dx, 0, nx - 1), jy = clamp(iy + dy, 0, ny - 1);
        sg += w.ground[jy * nx + jx]; n++;
      }
      let nb = 0, cnt = 0;
      for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
        const jx = clamp(ix + dx, 0, nx - 1), jy = clamp(iy + dy, 0, ny - 1);
        nb += w.bldg[jy * nx + jx]; cnt++;
      }
      this.ridge[iy * nx + ix] = w.ground[iy * nx + ix] - sg / n;
      this.bBlur[iy * nx + ix] = nb / cnt;
    }
  }

  setStorm(t0, mult = 2.2, ramp = 90, dur = 600) { this.storm = { t0, mult, ramp, dur }; }
  clearStorm() { this.storm = null; }
  stormFactor(t) {
    const s = this.storm;
    if (!s) return 1;
    if (t < s.t0) return 1;
    const up = clamp((t - s.t0) / s.ramp, 0, 1), down = clamp((s.t0 + s.dur - t) / s.ramp, 0, 1);
    return 1 + (s.mult - 1) * Math.min(up, down);
  }
  addDisturbance(d) { this.disturbances.push(d); }
  pruneDisturbances(t) { this.disturbances = this.disturbances.filter((d) => d.t1 > t); }

  timeFactor(t) { return 1 + 0.18 * Math.sin((2 * Math.PI * t) / 2400 + 0.7); }

  /** Mean (non-gust) wind. out = [u, v, w]. */
  mean(x, y, z, t, out = [0, 0, 0]) {
    const w = this.world, ci = w.cellIndex(x, y);
    const agl = Math.max(z - w.terrainFast(x, y), 8);
    const prof = Math.pow(Math.max(agl, 10) / 10, this.alpha);
    const dir = this.dirFrom + 18 * clamp(agl / 300, 0, 1);
    const S = this.speed10 * prof * this.timeFactor(t) * this.stormFactor(t);
    let [u, v] = windVec(S, dir);
    // ridge speed-up (hills accelerate flow, valleys slow it), fading with height
    const sp = 1 + 0.9 * clamp(this.ridge[ci] / 70, -0.35, 0.7) * Math.exp(-agl / 160);
    u *= sp; v *= sp;
    // valley channelling: cross-valley component is partly lost
    const ch = w.riverW[ci] * Math.exp(-agl / 140) * 0.6;
    if (ch > 1e-3) {
      const fx = w.flowX[ci], fy = w.flowY[ci];
      const along = u * fx + v * fy;
      u = (1 - ch) * u + ch * along * fx * 1.15;
      v = (1 - ch) * v + ch * along * fy * 1.15;
    }
    out[0] = u; out[1] = v; out[2] = 0;
    return out;
  }

  /** Turbulence intensity sigma (m/s) from shear, buildings, terrain lee. */
  sigma(x, y, z, t) {
    const w = this.world, ci = w.cellIndex(x, y);
    const agl = Math.max(z - w.terrainFast(x, y), 8);
    const roof = w.bldg[ci];
    const S = this.speed10 * Math.pow(Math.max(agl, 10) / 10, this.alpha) * this.timeFactor(t) * this.stormFactor(t);
    const bf = clamp(this.bBlur[ci] / 130, 0, 1) * Math.exp(-Math.max(0, agl - roof) / 120);
    const rt = clamp(Math.abs(this.ridge[ci]) / 80, 0, 1) * Math.exp(-agl / 120);
    return S * (0.1 + 0.32 * bf + 0.12 * rt) * this.gustScale;
  }

  /** Gust perturbation (frozen-turbulence advection, Taylor's hypothesis). */
  gust(x, y, z, t, out = [0, 0, 0]) {
    if (!this.turbulence || this.gustScale <= 0) { out[0] = out[1] = out[2] = 0; return out; }
    const s = this.sigma(x, y, z, t);
    const m = this.mean(x, y, z, 0, [0, 0, 0]);
    const px = (x - m[0] * t) / 140, py = (y - m[1] * t) / 140, pz = z / 80, pt = t / 45;
    const burst = 1 + 0.6 * Math.max(0, this._nb(t / 150, 0, 0));
    const k = 1.4 * s * burst;
    out[0] = k * this._n1(px, py, pz + pt);
    out[1] = k * this._n2(px + 17, py - 9, pz + pt);
    out[2] = 0.4 * k * this._n3(px - 5, py + 3, pz - pt);
    return out;
  }

  /** Full instantaneous wind (mean + gust + injected disturbances). */
  at(x, y, z, t, out = [0, 0, 0]) {
    this.mean(x, y, z, t, out);
    if (this.turbulence) {
      const g = this._g || (this._g = [0, 0, 0]);
      this.gust(x, y, z, t, g);
      out[0] += g[0]; out[1] += g[1]; out[2] += g[2];
    }
    for (const d of this.disturbances) {
      if (t < d.t0 || t > d.t1) continue;
      const dist = Math.hypot(x - d.x, y - d.y, (z - d.z) * 0.5);
      const wgt = 1 - smoothstep(0.4 * d.r, d.r, dist);
      if (wgt <= 0) continue;
      const ramp = Math.min(1, (t - d.t0) / 2);
      out[0] += d.vec[0] * wgt * ramp; out[1] += d.vec[1] * wgt * ramp; out[2] += (d.vec[2] || 0) * wgt * ramp;
    }
    return out;
  }
}

/** Forecast model: smooth, biased, no building-scale effects (documented limitation of coarse NWP). */
export class ForecastWind {
  constructor(truth, seed = 2, opts = {}) {
    this.truth = truth; this.world = truth.world;
    this.biasScale = opts.forecastBias ?? 1.0; // 1.0 = nominal error level
    this.speedErr = 0.82; this.dirErr = 22; // under-forecast speed, direction offset (deg)
    this.stale = false; this.ageS = 0;
    this._ne = makeNoise(seed * 11 + 5); this._nf = makeNoise(seed * 11 + 6);
    this.alpha = 0.22; // coarse model profile exponent differs from the real urban one
  }
  errAmp() { return 1.4 * this.biasScale * (1 + this.ageS / 1500); }
  mean(x, y, z, t, out = [0, 0, 0]) {
    const T = this.truth, w = this.world;
    const agl = Math.max(z - w.terrainFast(x, y), 8);
    const prof = Math.pow(Math.max(agl, 10) / 10, this.alpha);
    const dir = T.dirFrom + this.dirErr * this.biasScale + 18 * clamp(agl / 300, 0, 1);
    const stormF = 1 + 0.5 * (T.stormFactor(t) - 1); // forecast under-predicts storm intensity
    const S = T.speed10 * this.speedErr ** this.biasScale * prof * (1 + 0.18 * Math.sin((2 * Math.PI * t) / 2400 + 0.95)) * stormF;
    const [u, v] = windVec(S, dir);
    const a = this.errAmp();
    out[0] = u + a * this._ne(x / 2600, y / 2600, z / 220 + t / 2400);
    out[1] = v + a * this._nf(x / 2600 + 9, y / 2600 - 4, z / 220 + t / 2400);
    out[2] = 0;
    return out;
  }
  /** Forecast's belief about turbulence sigma (knows buildings, imperfect). */
  sigma(x, y, z, t) { return 0.8 * this.truth.sigma(x, y, z, t); }
  /** 1-sigma error of the forecast mean (m/s, per component). */
  meanSigma() { return this.errAmp() * 0.8 + 0.6 * this.biasScale; }
}

/** Per-cell scalar Kalman filter on the wind residual (truth - forecast), with kernel-spread updates. */
export class FleetWindMap {
  constructor(world, opts = {}) {
    this.world = world;
    this.csz = opts.cell ?? 200;
    this.nx = Math.ceil(world.width / this.csz); this.ny = Math.ceil(world.height / this.csz); this.nz = world.nz;
    const N = this.nx * this.ny * this.nz;
    this.mu = new Float32Array(N * 2);
    this.P = new Float32Array(N).fill(1);
    this.tLast = new Float32Array(N);
    this.P0 = opts.P0 ?? 9; // prior variance of residual (m/s)^2
    this.R = opts.R ?? 3.0; // observation noise variance (includes unresolved gust)
    this.ell = opts.ell ?? 330; // horizontal correlation length
    this.q = opts.q ?? 0.012; // process noise (variance growth, (m/s)^2 per s)
    this.tau = opts.tau ?? 1000; // residual memory (s)
    this.P.fill(this.P0);
    this.nObs = 0; this.version = 0;
  }
  _id(cx, cy, iz) { return (iz * this.ny + cy) * this.nx + cx; }
  _eff(c, t) {
    const dt = Math.max(0, t - this.tLast[c]);
    return { P: Math.min(this.P0, this.P[c] + this.q * dt), k: Math.exp(-dt / this.tau) };
  }
  /** Assimilate one residual observation r = [rx, ry] at (x, y, z, t). */
  update(x, y, z, t, rx, ry) {
    const w = this.world, iz0 = w.levelOf(z);
    const cx0 = Math.floor(x / this.csz), cy0 = Math.floor(y / this.csz);
    const rad = Math.ceil((2 * this.ell) / this.csz);
    for (let dz = -1; dz <= 1; dz++) {
      const iz = iz0 + dz;
      if (iz < 0 || iz >= this.nz) continue;
      const zw = dz === 0 ? 1 : 0.45;
      for (let dy = -rad; dy <= rad; dy++) for (let dx = -rad; dx <= rad; dx++) {
        const cx = cx0 + dx, cy = cy0 + dy;
        if (cx < 0 || cy < 0 || cx >= this.nx || cy >= this.ny) continue;
        const ddx = (cx + 0.5) * this.csz - x, ddy = (cy + 0.5) * this.csz - y;
        const wgt = zw * Math.exp(-(ddx * ddx + ddy * ddy) / (2 * this.ell * this.ell));
        if (wgt < 0.05) continue;
        const c = this._id(cx, cy, iz);
        const e = this._eff(c, t);
        const mx = this.mu[2 * c] * e.k, my = this.mu[2 * c + 1] * e.k;
        const K = e.P / (e.P + this.R / wgt);
        this.mu[2 * c] = mx + K * (rx - mx);
        this.mu[2 * c + 1] = my + K * (ry - my);
        this.P[c] = (1 - K) * e.P;
        this.tLast[c] = t;
      }
    }
    this.nObs++;
    if (this.nObs % 400 === 0) this.version++;
  }
  _cell(x, y, z) {
    const cx = clamp(Math.floor(x / this.csz), 0, this.nx - 1), cy = clamp(Math.floor(y / this.csz), 0, this.ny - 1);
    return this._id(cx, cy, this.world.levelOf(z));
  }
  correction(x, y, z, t, out = [0, 0, 0]) {
    const c = this._cell(x, y, z), e = this._eff(c, t);
    out[0] = this.mu[2 * c] * e.k; out[1] = this.mu[2 * c + 1] * e.k; out[2] = 0;
    return out;
  }
  variance(x, y, z, t) { const c = this._cell(x, y, z); return this._eff(c, t).P; }
  /** Fraction of the map that has been observed recently (variance well below the prior). */
  coverage(t) {
    let n = 0;
    for (let c = 0; c < this.P.length; c++) if (this._eff(c, t).P < 0.5 * this.P0) n++;
    return n / this.P.length;
  }
}

/** What the planner believes about the wind. */
export class EstimatedWind {
  constructor(forecast, fleetMap, flags) {
    this.forecast = forecast; this.map = fleetMap; this.flags = flags; // flags.windAware, flags.fleetWindMap
    this._c = [0, 0, 0]; this.manualVersion = 0;
    this.hazards = []; // broadcast hazard zones {x,y,z,r,vec,t0,t1} learned from fault detection (independent of the fleet map)
  }
  get version() { return this.map.version + this.manualVersion; }
  bump() { this.manualVersion++; }
  at(x, y, z, t, out = [0, 0, 0]) {
    if (!this.flags.windAware) { out[0] = out[1] = out[2] = 0; return out; }
    this.forecast.mean(x, y, z, t, out);
    if (this.flags.fleetWindMap) {
      const c = this.map.correction(x, y, z, t, this._c);
      out[0] += c[0]; out[1] += c[1];
    }
    for (const h of this.hazards) {
      if (t < h.t0 || t > h.t1) continue;
      const d = Math.hypot(x - h.x, y - h.y, (z - h.z) * 0.5);
      const wgt = 1 - smoothstep(0.4 * h.r, h.r, d);
      if (wgt > 0) { out[0] += h.vec[0] * wgt; out[1] += h.vec[1] * wgt; }
    }
    return out;
  }
  addHazard(h) { this.hazards.push(h); this.manualVersion++; }
  pruneHazards(t) { const n = this.hazards.length; this.hazards = this.hazards.filter((h) => h.t1 > t); if (this.hazards.length !== n) this.manualVersion++; }
  /** Planner-facing 1-sigma wind uncertainty (m/s): turbulence plus mean-wind uncertainty. */
  sigma(x, y, z, t) {
    if (!this.flags.windAware) return 0;
    const st = this.forecast.sigma(x, y, z, t);
    const mv = this.flags.fleetWindMap ? this.map.variance(x, y, z, t) : this.forecast.meanSigma() ** 2;
    let hz = 0;
    for (const h of this.hazards) if (t >= h.t0 && t <= h.t1 && Math.hypot(x - h.x, y - h.y) < h.r) hz = 4;
    return Math.sqrt(st * st + 0.5 * mv + hz * hz);
  }
  /** Upper bound on |mean wind| near (x,y,z) over [t0, t1] (used for admissible heuristics). */
  speedBound(x, y, z, t0, t1, tmp = [0, 0, 0]) {
    if (!this.flags.windAware) return 0;
    let m = 0;
    const steps = 6;
    for (let k = 0; k <= steps; k++) {
      this.at(x, y, z, t0 + ((t1 - t0) * k) / steps, tmp);
      m = Math.max(m, Math.hypot(tmp[0], tmp[1]));
    }
    return m + 0.6;
  }
}
