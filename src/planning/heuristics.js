// Heuristics for the lattice planners.
//
// 1) Relaxed-problem heuristic (the main research heuristic).
//    h(n) = exact optimal cost-to-go in a RELAXED problem: same lattice and obstacles, but no other aircraft,
//    no time dependence, and every edge priced at its BEST possible cost (fastest achievable ground speed
//    under the wind's upper bound, minimum possible power). Because each relaxed edge cost is a lower bound
//    on the real one, h is admissible; because it is an exact shortest-path distance, it is also consistent
//    and already "knows" about terrain/building/airspace detours, which Euclidean heuristics do not.
//    Idea: reverse resumable A* / true-distance heuristics from D. Silver, "Cooperative Pathfinding",
//    AIIDE 2005 (adapted here to wind- and energy-weighted costs), and relaxed-problem heuristics in
//    J. Pearl, "Heuristics", 1984.
//
//    Ground speed bound used per edge:   g <= Va_max + W.u <= Va_max + |W|_max       (|a| <= Va, g u = a + W)
//    Energy bound: E >= P_min * T   (P_min = minimum power over all airspeeds, from actuator-disc theory)
//
// 2) Emergency distance field: multi-source Dijkstra from all open landing sites. It doubles as a
//    precomputed contingency *policy* (follow the parent pointers) that needs no online search.

import { MinHeap } from '../core/util.js';

const H8 = [[1, 0], [-1, 0], [0, 1], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1]];

export function buildRelaxedHeuristic(world, cost, goalNode, t = 0) {
  const B = cost.windBounds(t);
  const phys = cost.features.physicsAware;
  const vaMax = phys ? cost.vaMaxPlan : cost.spec.vCruise;
  const lb = cost.lbRate, spec = cost.spec;
  const vVert = Math.max(spec.vClimb, spec.vDescend);
  const h = new Float64Array(world.nNodes).fill(Infinity);
  const heap = new MinHeap();
  h[goalNode] = 0; heap.push(0, goalNode);
  const L = world.layer, nx = world.nx;
  while (heap.size) {
    const d0 = heap.peekKey(), n = heap.pop();
    if (d0 > h[n]) continue;
    const iz = (n / L) | 0, c = n % L;
    for (let k = 0; k < 8; k++) {
      const r = cost.staticCorners(n, H8[k][0], H8[k][1]);
      if (!r) continue;
      const m = r.m;
      const d = world.cell * (H8[k][0] && H8[k][1] ? Math.SQRT2 : 1);
      const g = vaMax + Math.max(B[n], B[m]);
      const nd = d0 + (lb * d) / g;
      if (nd < h[m]) { h[m] = nd; heap.push(nd, m); }
    }
    for (const dz of [-1, 1]) {
      const jz = iz + dz;
      if (jz < 0 || jz >= world.nz) continue;
      const m = jz * L + c;
      if (world.blocked[m]) continue;
      const d = Math.abs(world.levels[jz] - world.levels[iz]);
      const nd = d0 + (lb * d) / vVert;
      if (nd < h[m]) { h[m] = nd; heap.push(nd, m); }
    }
  }
  return h;
}

/** Cache of relaxed heuristic tables keyed by goal, aircraft class and wind/world versions. */
export class HeuristicCache {
  constructor() { this.map = new Map(); this.builds = 0; }
  get(world, cost, goalNode, t) {
    const key = `${goalNode}|${cost.spec.key}|${cost.wind.version}|${cost.features.windAware}|${cost.features.fleetWindMap}|${cost.features.physicsAware}|${world.version}`;
    const hit = this.map.get(key);
    if (hit && t - hit.t < 900) return hit.h;
    if (this.map.size > 200) this.map.clear();
    const h = buildRelaxedHeuristic(world, cost, goalNode, t);
    this.map.set(key, { h, t }); this.builds++;
    return h;
  }
  clear() { this.map.clear(); }
}

/** Multi-source distance-to-nearest-landing-site field (time in seconds at cruise speed in still air). */
export class EmergencyField {
  constructor(world, spec) {
    this.world = world; this.spec = spec; this.version = -1;
    this.time = null; this.parent = null; this.src = null; this.sites = [];
  }

  build(sites) {
    const w = this.world, L = w.layer;
    this.sites = sites.map((s) => ({ site: s, node: -1 })).filter((e) => {
      const k = w.firstFreeLevel(e.site.ix, e.site.iy);
      if (k < 0) return false;
      e.node = w.idx(e.site.ix, e.site.iy, k); return true;
    });
    const t = new Float64Array(w.nNodes).fill(Infinity), par = new Int32Array(w.nNodes).fill(-1), src = new Int16Array(w.nNodes).fill(-1);
    const heap = new MinHeap();
    this.sites.forEach((e, i) => { t[e.node] = 0; src[e.node] = i; heap.push(0, e.node); });
    const v = this.spec.vCruise, vv = Math.max(this.spec.vClimb, this.spec.vDescend);
    const cost = { staticCorners: null };
    const tmpCost = (n, dx, dy) => this._corner(n, dx, dy);
    while (heap.size) {
      const d0 = heap.peekKey(), n = heap.pop();
      if (d0 > t[n]) continue;
      const iz = (n / L) | 0, c = n % L;
      for (let k = 0; k < 8; k++) {
        const r = tmpCost(n, H8[k][0], H8[k][1]);
        if (!r) continue;
        const m = r.m, d = w.cell * (H8[k][0] && H8[k][1] ? Math.SQRT2 : 1), nd = d0 + d / v;
        if (nd < t[m]) { t[m] = nd; par[m] = n; src[m] = src[n]; heap.push(nd, m); }
      }
      for (const dz of [-1, 1]) {
        const jz = iz + dz;
        if (jz < 0 || jz >= w.nz) continue;
        const m = jz * L + c;
        if (w.blocked[m]) continue;
        const nd = d0 + Math.abs(w.levels[jz] - w.levels[iz]) / vv;
        if (nd < t[m]) { t[m] = nd; par[m] = n; src[m] = src[n]; heap.push(nd, m); }
      }
    }
    this.time = t; this.parent = par; this.src = src; this.version = w.version;
    void cost;
    return this;
  }

  _corner(n, dx, dy) {
    const w = this.world, L = w.layer, nx = w.nx;
    const iz = (n / L) | 0, c = n % L, ix = c % nx, iy = (c - ix) / nx;
    const jx = ix + dx, jy = iy + dy;
    if (jx < 0 || jy < 0 || jx >= nx || jy >= w.ny) return null;
    const m = iz * L + jy * nx + jx;
    if (w.blocked[m]) return null;
    if (dx && dy && (w.blocked[iz * L + iy * nx + jx] || w.blocked[iz * L + jy * nx + ix])) return null;
    return { m };
  }

  reachable(node) { return this.time[node] < Infinity; }
  timeTo(node) { return this.time[node]; }
  siteOf(node) { const i = this.src[node]; return i >= 0 ? this.sites[i].site : null; }

  /** Node sequence from `node` to its nearest site's landing node (inclusive). */
  pathFrom(node) {
    const out = [node];
    let n = node, guard = 0;
    while (this.parent[n] >= 0 && guard++ < 5000) { n = this.parent[n]; out.push(n); }
    return out;
  }

  /** Energy (kWh) to divert from `node` to the nearest site and land, with a wind margin. */
  energyToLand(node, costModel, windMargin = 1.25) {
    const t = this.time[node];
    if (!(t < Infinity)) return Infinity;
    const site = this.siteOf(node);
    const w = this.world;
    const k = w.firstFreeLevel(site.ix, site.iy);
    const descent = Math.max(0, w.levels[k] - site.padElev);
    const eLand = (descent / this.spec.vDescend) * costModel.pDesc / 3600;
    return (t * costModel.pCruise * windMargin) / 3600 + eLand;
  }
}

export { H8 };
