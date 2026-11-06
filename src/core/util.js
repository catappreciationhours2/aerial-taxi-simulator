// Shared numeric utilities: seeded RNG, binary heap, smooth value noise, normal distribution.

export const clamp = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);
export const lerp = (a, b, t) => a + (b - a) * t;
export const smoothstep = (a, b, x) => {
  const t = clamp((x - a) / (b - a), 0, 1);
  return t * t * (3 - 2 * t);
};
export const DEG = Math.PI / 180;

// Seeded PRNG.
// mulberry32 by Tommy Ettinger (public domain), widely circulated JS available online
export function mulberry32(seed) {
  let a = seed >>> 0;
  return function () {
    a |= 0;
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export class RNG {
  constructor(seed = 1) {
    this.seed = seed >>> 0;
    this._next = mulberry32(this.seed);
    this._spare = null;
  }
  float() { return this._next(); }
  range(a, b) { return a + (b - a) * this._next(); }
  int(n) { return Math.floor(this._next() * n); }
  chance(p) { return this._next() < p; }
  pick(arr) { return arr[this.int(arr.length)]; }
  /** Standard normal via Box-Muller. */
  gauss() {
    if (this._spare !== null) { const s = this._spare; this._spare = null; return s; }
    let u = 0, v = 0;
    while (u === 0) u = this._next();
    v = this._next();
    const r = Math.sqrt(-2 * Math.log(u));
    this._spare = r * Math.sin(2 * Math.PI * v);
    return r * Math.cos(2 * Math.PI * v);
  }
  /** Exponential with given rate (events / unit time). */
  exp(rate) { return -Math.log(1 - this._next()) / rate; }
  /** Deterministic child stream, so subsystems do not perturb each other. */
  fork(label) {
    let h = this.seed ^ 0x9e3779b9;
    for (let i = 0; i < label.length; i++) h = Math.imul(h ^ label.charCodeAt(i), 0x85ebca6b) >>> 0;
    return new RNG(h);
  }
}

export function hashStr(s) {
  let h = 2166136261 >>> 0;
  for (let i = 0; i < s.length; i++) { h ^= s.charCodeAt(i); h = Math.imul(h, 16777619) >>> 0; }
  return h >>> 0;
}

// Binary min-heap keyed by a numeric priority. Used by every graph search.
// ripped off my 210 hw lol rip
export class MinHeap {
  constructor() { this.keys = []; this.vals = []; }
  get size() { return this.keys.length; }
  push(key, val) {
    const k = this.keys, v = this.vals;
    let i = k.length;
    k.push(key); v.push(val);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (k[p] <= key) break;
      k[i] = k[p]; v[i] = v[p]; i = p;
    }
    k[i] = key; v[i] = val;
  }
  peekKey() { return this.keys[0]; }
  pop() {
    const k = this.keys, v = this.vals;
    const top = v[0];
    const lk = k.pop(), lv = v.pop();
    const n = k.length;
    if (n > 0) {
      let i = 0;
      for (;;) {
        let c = 2 * i + 1;
        if (c >= n) break;
        if (c + 1 < n && k[c + 1] < k[c]) c++;
        if (k[c] >= lk) break;
        k[i] = k[c]; v[i] = v[c]; i = c;
      }
      k[i] = lk; v[i] = lv;
    }
    return top;
  }
}

// Smooth value noise (3D), deterministic from a seed. Output in [-1, 1].
function lattice(ix, iy, iz, seed) {
  let h = Math.imul(ix, 374761393) ^ Math.imul(iy, 668265263) ^ Math.imul(iz, 2147483647) ^ Math.imul(seed, 1274126177);
  h = Math.imul(h ^ (h >>> 13), 1274126177);
  h ^= h >>> 16;
  return ((h >>> 0) / 4294967295) * 2 - 1;
}
const fade = (t) => t * t * t * (t * (t * 6 - 15) + 10);

export function makeNoise(seed) {
  return function noise(x, y = 0, z = 0) {
    const x0 = Math.floor(x), y0 = Math.floor(y), z0 = Math.floor(z);
    const fx = fade(x - x0), fy = fade(y - y0), fz = fade(z - z0);
    const c = (dx, dy, dz) => lattice(x0 + dx, y0 + dy, z0 + dz, seed);
    const x00 = lerp(c(0, 0, 0), c(1, 0, 0), fx), x10 = lerp(c(0, 1, 0), c(1, 1, 0), fx);
    const x01 = lerp(c(0, 0, 1), c(1, 0, 1), fx), x11 = lerp(c(0, 1, 1), c(1, 1, 1), fx);
    return lerp(lerp(x00, x10, fy), lerp(x01, x11, fy), fz);
  };
}

export function makeFbm(seed, octaves = 4) {
  const n = makeNoise(seed);
  return function fbm(x, y = 0, z = 0) {
    let amp = 1, f = 1, sum = 0, norm = 0;
    for (let o = 0; o < octaves; o++) {
      sum += amp * n(x * f, y * f, z * f);
      norm += amp; amp *= 0.5; f *= 2;
    }
    return sum / norm;
  };
}

// Normal distribution - code adapted from following papers read for reproducibility and speed
// normalCdf: Abramowitz & Stegun 7.1.26 erf approximation (|err| < 1.5e-7).
// normalInv: adapted from Peter J. Acklam's rational approximation of the
// inverse normal CDF (relative error < 1.15e-9). https://stackedboxes.org/2017/05/01/acklams-normal-quantile-function/
export function erf(x) {
  const s = x < 0 ? -1 : 1;
  x = Math.abs(x);
  const t = 1 / (1 + 0.3275911 * x);
  const y = 1 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t * Math.exp(-x * x);
  return s * y;
}
export const normalCdf = (x) => 0.5 * (1 + erf(x / Math.SQRT2));

export function normalInv(p) {
  if (p <= 0) return -Infinity;
  if (p >= 1) return Infinity;
  const a = [-3.969683028665376e+01, 2.209460984245205e+02, -2.759285104469687e+02, 1.383577518672690e+02, -3.066479806614716e+01, 2.506628277459239e+00];
  const b = [-5.447609879822406e+01, 1.615858368580409e+02, -1.556989798598866e+02, 6.680131188771972e+01, -1.328068155288572e+01];
  const c = [-7.784894002430293e-03, -3.223964580411365e-01, -2.400758277161838e+00, -2.549732539343734e+00, 4.374664141464968e+00, 2.938163982698783e+00];
  const d = [7.784695709041462e-03, 3.224671290700398e-01, 2.445134137142996e+00, 3.754408661907416e+00];
  const plow = 0.02425, phigh = 1 - plow;
  let q, r;
  if (p < plow) {
    q = Math.sqrt(-2 * Math.log(p));
    return (((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1);
  }
  if (p > phigh) {
    q = Math.sqrt(-2 * Math.log(1 - p));
    return -(((((c[0] * q + c[1]) * q + c[2]) * q + c[3]) * q + c[4]) * q + c[5]) / ((((d[0] * q + d[1]) * q + d[2]) * q + d[3]) * q + 1);
  }
  q = p - 0.5; r = q * q;
  return (((((a[0] * r + a[1]) * r + a[2]) * r + a[3]) * r + a[4]) * r + a[5]) * q / (((((b[0] * r + b[1]) * r + b[2]) * r + b[3]) * r + b[4]) * r + 1);
}

// Small vector helpers on plain arrays [x,y,z].
export const v3 = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  scale: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  norm: (a) => Math.hypot(a[0], a[1], a[2]),
  dist: (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]),
  hdist: (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]),
};
