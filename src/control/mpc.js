// Receding-horizon trajectory tracking (Model Predictive Control) for a 3-D double integrator with an
// estimated additive disturbance acceleration d (from a disturbance observer: wind/drag mismatch).
//
//   p_{k+1} = p_k + v_k dt + 1/2 (u_k + d) dt^2,   v_{k+1} = v_k + (u_k + d) dt
//   min  sum_k  1/2 qp |p_k - r_k|^2 + 1/2 qv |v_k - rv_k|^2 + 1/2 qa |u_k|^2 + 1/2 qj |u_k - u_{k-1}|^2
//   s.t. |u_k,horizontal| <= aH,  -aDn <= u_k,z <= aUp   (tightened by a disturbance margin = tube-MPC idea)
//
// Solver: projected Newton-style iteration. The unconstrained Hessian of this QP is constant, so we build
// and invert it once per parameter set (cached) and use it to precondition projected-gradient steps. The
// constraint tightening follows the "tube" idea of D. Q. Mayne, M. M. Seron, S. V. Rakovic, "Robust model
// predictive control of constrained linear systems with bounded disturbances", Automatica 41(2), 2005.

const cache = new Map();

function gradient(x0, u, p) {
  const { H, dt, d, refP, refV, qp, qv, qa, qj, wTerm, uPrev } = p;
  const P = p._P, V = p._V, G = p._G;
  // forward rollout
  let px = x0.p[0], py = x0.p[1], pz = x0.p[2], vx = x0.v[0], vy = x0.v[1], vz = x0.v[2];
  const h2 = 0.5 * dt * dt;
  for (let k = 0; k < H; k++) {
    const ax = u[3 * k] + d[0], ay = u[3 * k + 1] + d[1], az = u[3 * k + 2] + d[2];
    px += vx * dt + h2 * ax; py += vy * dt + h2 * ay; pz += vz * dt + h2 * az;
    vx += ax * dt; vy += ay * dt; vz += az * dt;
    const o = 3 * (k + 1);
    P[o] = px; P[o + 1] = py; P[o + 2] = pz; V[o] = vx; V[o + 1] = vy; V[o + 2] = vz;
  }
  // backward adjoint
  let lp0 = 0, lp1 = 0, lp2 = 0, lv0 = 0, lv1 = 0, lv2 = 0;
  for (let k = H - 1; k >= 0; k--) {
    const o = 3 * (k + 1), w = k === H - 1 ? wTerm : 1;
    lp0 += qp * w * (P[o] - refP[o]); lp1 += qp * w * (P[o + 1] - refP[o + 1]); lp2 += qp * w * (P[o + 2] - refP[o + 2]);
    lv0 += qv * w * (V[o] - refV[o]); lv1 += qv * w * (V[o + 1] - refV[o + 1]); lv2 += qv * w * (V[o + 2] - refV[o + 2]);
    for (let a = 0; a < 3; a++) {
      const lp = a === 0 ? lp0 : a === 1 ? lp1 : lp2, lv = a === 0 ? lv0 : a === 1 ? lv1 : lv2;
      const uk = u[3 * k + a], up = k === 0 ? uPrev[a] : u[3 * (k - 1) + a], un = k + 1 < H ? u[3 * (k + 1) + a] : uk;
      G[3 * k + a] = h2 * lp + dt * lv + qa * uk + qj * (uk - up) - qj * (un - uk);
    }
    lv0 += dt * lp0; lv1 += dt * lp1; lv2 += dt * lp2;
  }
  return G;
}

function hessianInverse(H, dt, qp, qv, qa, qj, wTerm) {
  const key = `${H}|${dt}|${qp}|${qv}|${qa}|${qj}|${wTerm}`;
  if (cache.has(key)) return cache.get(key);
  // Build per-axis Hessian by differencing the (linear) gradient on a scalar axis.
  const params = {
    H, dt, d: [0, 0, 0], qp, qv, qa, qj, wTerm, uPrev: [0, 0, 0],
    refP: new Float64Array(3 * (H + 1)), refV: new Float64Array(3 * (H + 1)),
    _P: new Float64Array(3 * (H + 1)), _V: new Float64Array(3 * (H + 1)), _G: new Float64Array(3 * H),
  };
  const x0 = { p: [0, 0, 0], v: [0, 0, 0] };
  const u0 = new Float64Array(3 * H);
  const g0 = Float64Array.from(gradient(x0, u0, params));
  const M = Array.from({ length: H }, () => new Float64Array(H));
  for (let j = 0; j < H; j++) {
    const u = new Float64Array(3 * H); u[3 * j] = 1;
    const g = gradient(x0, u, params);
    for (let i = 0; i < H; i++) M[i][j] = g[3 * i] - g0[3 * i];
  }
  // Gauss-Jordan inverse
  const A = M.map((r, i) => { const row = new Float64Array(2 * H); row.set(r); row[H + i] = 1; return row; });
  for (let c = 0; c < H; c++) {
    let piv = c;
    for (let r = c + 1; r < H; r++) if (Math.abs(A[r][c]) > Math.abs(A[piv][c])) piv = r;
    [A[c], A[piv]] = [A[piv], A[c]];
    const dv = A[c][c];
    for (let j = 0; j < 2 * H; j++) A[c][j] /= dv;
    for (let r = 0; r < H; r++) if (r !== c) { const f = A[r][c]; if (f) for (let j = 0; j < 2 * H; j++) A[r][j] -= f * A[c][j]; }
  }
  const inv = A.map((r) => r.slice(H));
  cache.set(key, inv);
  return inv;
}

export const MPC_DEFAULTS = { H: 10, dt: 0.5, qp: 1.0, qv: 1.2, qa: 0.02, qj: 0.15, wTerm: 4, iters: 5, alpha: 0.85 };

/**
 * Solve one MPC problem.
 * @param {{p:number[], v:number[]}} x0   current (estimated) state
 * @param {(k:number)=>{p:number[], v:number[]}} ref  reference at time now + k*dt, k = 1..H
 * @param {{h:number, up:number, dn:number}} lim  acceleration limits (already tightened by caller if desired)
 * @param {number[]} d  disturbance acceleration estimate (m/s^2)
 * @param {Float64Array} [warm]  previous solution (3H) for warm start
 * @param {number[]} [uPrev]  last applied acceleration
 * @returns {{u:Float64Array, a0:number[], saturated:boolean, terminalErr:number, cost:number}}
 */
export function solveMPC(x0, ref, lim, d = [0, 0, 0], warm = null, uPrev = [0, 0, 0], opts = {}) {
  const o = { ...MPC_DEFAULTS, ...opts };
  const { H, dt } = o;
  const Hinv = hessianInverse(H, dt, o.qp, o.qv, o.qa, o.qj, o.wTerm);
  const refP = new Float64Array(3 * (H + 1)), refV = new Float64Array(3 * (H + 1));
  for (let k = 1; k <= H; k++) {
    const r = ref(k);
    for (let a = 0; a < 3; a++) { refP[3 * k + a] = r.p[a]; refV[3 * k + a] = r.v[a]; }
  }
  const params = {
    H, dt, d, refP, refV, qp: o.qp, qv: o.qv, qa: o.qa, qj: o.qj, wTerm: o.wTerm, uPrev,
    _P: new Float64Array(3 * (H + 1)), _V: new Float64Array(3 * (H + 1)), _G: new Float64Array(3 * H),
  };
  const u = warm && warm.length === 3 * H ? Float64Array.from(warm) : new Float64Array(3 * H);
  let saturated = false;
  const project = () => {
    saturated = false;
    for (let k = 0; k < H; k++) {
      const ax = u[3 * k], ay = u[3 * k + 1], az = u[3 * k + 2];
      const ah = Math.hypot(ax, ay);
      if (ah > lim.h) { const s = lim.h / ah; u[3 * k] = ax * s; u[3 * k + 1] = ay * s; if (k === 0) saturated = true; }
      if (az > lim.up) { u[3 * k + 2] = lim.up; if (k === 0) saturated = true; } else if (az < -lim.dn) { u[3 * k + 2] = -lim.dn; if (k === 0) saturated = true; }
    }
  };
  project();
  for (let it = 0; it < o.iters; it++) {
    const g = gradient(x0, u, params);
    for (let a = 0; a < 3; a++) {
      for (let i = 0; i < H; i++) {
        let s = 0;
        for (let j = 0; j < H; j++) s += Hinv[i][j] * g[3 * j + a];
        u[3 * i + a] -= o.alpha * s;
      }
    }
    project();
  }
  // evaluate final cost + terminal error for diagnostics
  gradient(x0, u, params);
  let cost = 0, term = 0;
  for (let k = 1; k <= H; k++) {
    let ep = 0, ev = 0;
    for (let a = 0; a < 3; a++) { ep += (params._P[3 * k + a] - refP[3 * k + a]) ** 2; ev += (params._V[3 * k + a] - refV[3 * k + a]) ** 2; }
    cost += 0.5 * (o.qp * ep + o.qv * ev) * (k === H ? o.wTerm : 1);
    if (k === H) term = Math.sqrt(ep);
  }
  return { u, a0: [u[0], u[1], u[2]], saturated, terminalErr: term, cost };
}

/** Plain PD tracker (baseline when MPC is disabled): no preview, no disturbance feed-forward. */
export function pdTrack(x0, refNow, lim, kp = 0.22, kd = 0.9) {
  const a = [0, 1, 2].map((i) => kp * (refNow.p[i] - x0.p[i]) + kd * (refNow.v[i] - x0.v[i]));
  const ah = Math.hypot(a[0], a[1]);
  if (ah > lim.h) { a[0] *= lim.h / ah; a[1] *= lim.h / ah; }
  a[2] = Math.max(-lim.dn, Math.min(lim.up, a[2]));
  return a;
}
