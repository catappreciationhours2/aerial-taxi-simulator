// Control-Barrier-Function (CBF) safety filter.
//
// Given a nominal acceleration from the planner/MPC, find the closest acceleration (in the L2 sense) that
// satisfies pairwise separation barriers and static-obstacle barriers, plus actuator limits.
//
// Barrier for relative-degree-2 separation (stopping-distance form):
//   h(p, v) = sqrt(2 alpha (|p| - Ds)) + p.v / |p|          (alpha = braking authority, Ds = protected radius)
//   dh/dt >= -gamma h   =>   linear inequality on the own acceleration.
// adapted from: L. Wang, A. D. Ames, M. Egerstedt, "Safety Barrier Certificates for Collisions-Free
// Multirobot Systems", IEEE Transactions on Robotics 33(3), 2017 (the sqrt-braking barrier is their double-
// integrator construction); CBF-QP formulation from A. D. Ames et al., IEEE TAC 62(8), 2017.
// The QP is solved with Dykstra's alternating projection algorithm:
// adapted from: J. P. Boyle and R. L. Dykstra, "A method for finding projections onto the intersection of
// convex sets in Hilbert spaces", Lecture Notes in Statistics 37, 1986.
//
// Feasibility is NOT guaranteed (limited control authority, tight geometry): the result reports it and the
// runtime-assurance supervisor switches to a backup behaviour when it fails.

/** Half-space n . a <= b  */
export function pairConstraint(pRel, vRel, Ds, alpha, gamma, share = 0.5, aEscape = 1.5) {
  const d = Math.hypot(pRel[0], pRel[1], pRel[2]);
  if (d < 1e-6) return { n: [0, 0, -1], b: -aEscape, active: true, h: -1, escape: true };
  const n = [pRel[0] / d, pRel[1] / d, pRel[2] / d];
  const s = d - Ds;
  const nv = n[0] * vRel[0] + n[1] * vRel[1] + n[2] * vRel[2];
  if (s <= 0) return { n: [-n[0], -n[1], -n[2]], b: -aEscape, active: true, h: s, escape: true };
  const v2 = vRel[0] ** 2 + vRel[1] ** 2 + vRel[2] ** 2;
  const sq = Math.sqrt(2 * alpha * s);
  const h = sq + nv;
  const term1 = (alpha * nv) / sq;
  const term2 = (v2 - nv * nv) / d;
  const rhs = -gamma * h - term1 - term2; // need n.(a_i - a_j) >= rhs ; own share of it:
  return { n: [-n[0], -n[1], -n[2]], b: -share * rhs, active: true, h, escape: false };
}

/** Project a onto { a : n . a <= b }. */
function projHalf(a, n, b) {
  const nn = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
  const v = n[0] * a[0] + n[1] * a[1] + n[2] * a[2] - b;
  if (v <= 0 || nn < 1e-12) return a;
  const s = v / nn;
  return [a[0] - s * n[0], a[1] - s * n[1], a[2] - s * n[2]];
}
/** Project onto the acceleration set: horizontal norm <= h, z in [-dn, up]. */
function projAccel(a, lim) {
  let [x, y, z] = a;
  const h = Math.hypot(x, y);
  if (h > lim.h) { x *= lim.h / h; y *= lim.h / h; }
  z = Math.max(-lim.dn, Math.min(lim.up, z));
  return [x, y, z];
}

/**
 * Dykstra's algorithm: project aNom onto the intersection of the half-spaces and the accel set.
 * @returns {{a:number[], feasible:boolean, maxViolation:number, iters:number}}
 */
export function dykstra(aNom, halfspaces, lim, maxIter = 60, tol = 1e-3) {
  const m = halfspaces.length;
  let x = [aNom[0], aNom[1], aNom[2]];
  const incs = Array.from({ length: m + 1 }, () => [0, 0, 0]);
  let iters = 0;
  for (; iters < maxIter; iters++) {
    const x0 = x.slice();
    for (let i = 0; i <= m; i++) {
      const y = [x[0] + incs[i][0], x[1] + incs[i][1], x[2] + incs[i][2]];
      const p = i < m ? projHalf(y, halfspaces[i].n, halfspaces[i].b) : projAccel(y, lim);
      incs[i] = [y[0] - p[0], y[1] - p[1], y[2] - p[2]];
      x = p;
    }
    if (Math.hypot(x[0] - x0[0], x[1] - x0[1], x[2] - x0[2]) < 1e-6) { iters++; break; }
  }
  let maxV = 0;
  for (const hs of halfspaces) maxV = Math.max(maxV, hs.n[0] * x[0] + hs.n[1] * x[1] + hs.n[2] * x[2] - hs.b);
  const ah = Math.hypot(x[0], x[1]);
  maxV = Math.max(maxV, ah - lim.h, x[2] - lim.up, -lim.dn - x[2]);
  return { a: x, feasible: maxV <= tol * 5, maxViolation: maxV, iters };
}

/**
 * Build constraints and filter a nominal acceleration.
 * @param own       {p, v, sigma}
 * @param neighbors [{p, v, sigma, coop}]  (coop=false => own aircraft takes the whole avoidance burden)
 * @param statics   [{n:[3] unit away from obstacle, d: distance to surface, v:[3]}]
 */
export function cbfFilter(aNom, own, neighbors, statics, params) {
  const { Ds, alpha, gamma, lim, range = 450, dsStatic = 25, alphaStatic = alpha, gateMult = 1.8, gateTime = 15 } = params;
  const hs = [];
  let minH = Infinity, nActive = 0;
  for (const nb of neighbors) {
    const pRel = [own.p[0] - nb.p[0], own.p[1] - nb.p[1], own.p[2] - nb.p[2]];
    const d = Math.hypot(pRel[0], pRel[1], pRel[2]);
    if (d > range) continue;
    const vRel = [own.v[0] - nb.v[0], own.v[1] - nb.v[1], own.v[2] - nb.v[2]];
    const Dp = Ds + 1.5 * ((own.sigma || 0) + (nb.sigma || 0));
    // Collision-cone gate (engineering choice, not from the cited papers): the sqrt-braking barrier looks only along the
    // line of sight, so it would fire on every fast but well-separated pass. We apply it only when the predicted closest
    // approach inside the look-ahead window is inside `gateMult * Dp` (or the pair is already inside that radius).
    const vv = vRel[0] * vRel[0] + vRel[1] * vRel[1] + vRel[2] * vRel[2];
    if (d > gateMult * Dp) {
      const tca = vv > 1e-9 ? Math.min(gateTime, Math.max(0, -(pRel[0] * vRel[0] + pRel[1] * vRel[1] + pRel[2] * vRel[2]) / vv)) : 0;
      const cx = pRel[0] + vRel[0] * tca, cy = pRel[1] + vRel[1] * tca, cz = pRel[2] + vRel[2] * tca;
      if (Math.hypot(cx, cy, cz) > gateMult * Dp) continue;
    }
    const c = pairConstraint(pRel, vRel, Dp, alpha, gamma, nb.coop === false ? 1 : 0.5, 0.5 * lim.h);
    minH = Math.min(minH, c.h);
    hs.push(c); nActive++;
  }
  for (const s of statics || []) {
    const dd = s.d - dsStatic;
    if (dd > 250) continue;
    const c = pairConstraint([s.n[0] * s.d, s.n[1] * s.d, s.n[2] * s.d], own.v, dsStatic, alphaStatic, gamma, 1, 0.5 * lim.h);
    hs.push(c); nActive++;
    minH = Math.min(minH, c.h);
  }
  if (!hs.length) return { a: aNom.slice(), feasible: true, modified: false, minH, nActive: 0, maxViolation: 0 };
  const r = dykstra(aNom, hs, lim);
  const mod = Math.hypot(r.a[0] - aNom[0], r.a[1] - aNom[1], r.a[2] - aNom[2]) > 1e-3;
  return { a: r.a, feasible: r.feasible, modified: mod, minH, nActive, maxViolation: r.maxViolation };
}
