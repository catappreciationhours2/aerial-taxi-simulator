// Fleet coordinator: owns the reservation table and turns requests (new mission, replan, diversion,
// emergency) into conflict-free trajectories.
//
// Hierarchy (project plan layers 2-3):
//   * individual kinodynamic-ish routing: wind/energy-aware SIPP with relaxed-problem heuristic
//   * prioritised planning: aircraft are planned one after another against the reservations of those already planned
//   * localized repair: when forced reservations (emergency, TFR, external traffic) create conflicts, only the
//     affected cluster is re-planned, with randomised priority orders, keeping the best result
//     (Large Neighborhood Search idea from J. Li, Z. Chen, D. Harabor, P. Stuckey, S. Koenig,
//      "Anytime Multi-Agent Path Finding via Large Neighborhood Search", IJCAI 2021;
//      randomised priorities from M. Bennewitz, W. Burgard, S. Thrun, "Finding and optimizing solvable priority
//      schemes for decoupled path planning techniques for teams of mobile robots", Robotics and Autonomous Systems 2002)
//   * vertiport pads are booked as capacity resources, tied to the airspace plan through the goal time

import { PLANNER } from '../core/config.js';
import { ReservationTable } from './reservations.js';
import { CostModel } from './costmodel.js';
import { HeuristicCache, EmergencyField } from './heuristics.js';
import { sippSearch, pathToSegments } from './sipp.js';
import { Trajectory, reserveTrajectory } from './trajectory.js';
import { makeSpec } from '../aircraft/power.js';
import { vMaxEff } from '../aircraft/dynamics.js';

const TAKEOFF_CLEAR = 14; // s FATO occupancy after climb-out (covers the smoothed departure ramp, see Trajectory.ref)
const TURNAROUND = 14; // s FATO occupancy after touchdown (then the aircraft taxis to a stand)
const EPSILON = 1.5; // bounded-suboptimality factor used by the live simulator

export class Coordinator {
  constructor({ world, wind, features, vertiports, rng, log = () => {} }) {
    this.world = world; this.wind = wind; this.features = features; this.vertiports = vertiports; this.rng = rng; this.log = log;
    this.table = new ReservationTable();
    this.hcache = new HeuristicCache();
    this.costs = new Map();
    this.refSpec = makeSpec('standard');
    this.emergency = new EmergencyField(world, this.refSpec);
    this._emKey = '';
    this.timeoutUntil = -1;
    this.epsilon = EPSILON;
    this.stats = { plans: 0, failures: 0, replans: 0, expansions: 0, planMs: [], lnsRuns: 0, lnsRecovered: 0, emergencies: 0, forced: 0, viabilityRejects: 0, deadlines: 0 };
  }

  costFor(spec) {
    let c = this.costs.get(spec.key);
    if (!c) { c = new CostModel(this.world, this.wind, spec, this.features); this.costs.set(spec.key, c); }
    return c;
  }

  _budget(tNow) { return tNow < this.timeoutUntil ? 25 : PLANNER.maxExpansions; }
  siteNode(site) { return this.world.idx(site.ix, site.iy, this.world.firstFreeLevel(site.ix, site.iy)); }

  siteUsable(site, t = 0) {
    if (site.closed) return false;
    const k = this.world.firstFreeLevel(site.ix, site.iy);
    if (k < 0) return false;
    if (this.world.restrictedAt(site.x, site.y, this.world.levels[k], t)) return false;
    return true;
  }

  /** (Re)build the emergency field when sites/closures/airspace change. */
  emergencyField() {
    // Contingency landings use the dedicated emergency sites (parks, fields) so that a distressed aircraft never lands on a pad that is
    // in use by scheduled operations; scheduled diversions still go to vertiports through the normal booking path.
    const sites = this.world.emergencySites.filter((s) => this.siteUsable(s, 0));
    const key = `${this.world.version}|${sites.map((s) => s.id).join(',')}`;
    if (key !== this._emKey) { this.emergency.build(sites); this._emKey = key; }
    return this.emergency;
  }

  _inflateFor(ac) { return this.features.adaptiveEnvelope && (ac.sigmaPos || 0) > 7 ? 1 : 0; }
  _padMult(ac) { return this.features.adaptiveEnvelope ? (ac.padMult || 1) : 1; }

  // core leg planner
  /** Plan from startNode at time t0 to a destination site. No side effects on table/pads. */
  planLeg(ac, { startNode, t0, destId, tNow, takeoff = false }) {
    const w = this.world, spec = ac.spec;
    const site = w.siteById(destId);
    if (!site) return { ok: false, reason: 'no-site' };
    if (!this.siteUsable(site, tNow)) return { ok: false, reason: 'dest-unusable' };
    const kD = w.firstFreeLevel(site.ix, site.iy);
    const goalNode = w.idx(site.ix, site.iy, kD);
    const cost = this.costFor(spec);
    const landDur = Math.max(1, (w.levels[kD] - site.padElev) / spec.vDescend);
    const landEnergy = (cost.pDesc * landDur) / 3600;
    const landCost = cost.w.time * landDur + cost.w.energy * landEnergy;
    const h = this.hcache.get(w, cost, goalNode, tNow);
    const vd = this.vertiports.get(destId);
    const vo = takeoff ? this.vertiports.get(ac.siteId) : null;
    const startSite = takeoff ? w.siteById(ac.siteId) : null;
    const kO = takeoff ? w.firstFreeLevel(startSite.ix, startSite.iy) : 0;
    const climbDur = takeoff ? Math.max(1, (w.levels[kO] - startSite.padElev) / spec.vClimb) : 0;
    const ctx = { world: w, cost, table: this.features.reservations ? this.table : null, h };
    let t0cur = t0, tGoalMin = 0, res = null, takeoffStart = 0, takeoffSlot = null, landSlot = null, takeoffOcc = 0, landStart = 0, landOcc = 0;
    const tStart = performance.now();
    for (let attempt = 0; attempt < 5; attempt++) {
      if (takeoff) {
        const slot = vo.findSlot(Math.max(t0cur - climbDur, tNow), climbDur + TAKEOFF_CLEAR, ac.id);
        if (!slot) return { ok: false, reason: 'origin-closed' };
        t0cur = Math.max(t0cur, slot.start + climbDur);
        takeoffSlot = slot;
      }
      res = sippSearch({ id: ac.id, startNode, t0: t0cur, goalNode, tGoalMin, landDur, landCost, epsilon: this.epsilon, maxExpansions: this._budget(tNow) }, ctx);
      this.stats.expansions += res.expansions;
      if (!res.ok) break;
      if (takeoff) {
        takeoffStart = res.path[0].t - climbDur;
        // The take-off column stays occupied until the aircraft has actually left the vertiport cell (it may hover while waiting for airspace).
        const cell0 = startNode % w.layer; let leaveT = res.path[res.path.length - 1].t;
        for (let i = 1; i < res.path.length; i++) if (res.path[i].node % w.layer !== cell0) { leaveT = res.path[i].dep; break; }
        takeoffOcc = leaveT - takeoffStart + TAKEOFF_CLEAR;
        const slot = vo.findSlot(takeoffStart, takeoffOcc, ac.id);
        if (slot && slot.start > takeoffStart + 0.01) { t0cur = slot.start + climbDur; continue; }
        takeoffSlot = slot || takeoffSlot;
      }
      if (vd) {
        // The landing column is occupied from arrival at the goal node (incl. any hover) until touchdown + turnaround.
        landStart = res.path[res.path.length - 1].t; landOcc = res.tLand - landStart + landDur + TURNAROUND;
        const slot = vd.findSlot(landStart, landOcc, ac.id);
        if (!slot) return { ok: false, reason: 'dest-closed' };
        if (slot.start > landStart + 0.01) { tGoalMin = res.tLand + (slot.start - landStart); continue; }
        landSlot = slot;
      }
      break;
    }
    this.stats.planMs.push(performance.now() - tStart);
    if (this.stats.planMs.length > 2000) this.stats.planMs.splice(0, 1000);
    if (!res || !res.ok) return { ok: false, reason: res ? res.reason : 'no-result', expansions: res?.expansions || 0 };
    if (res.reason === 'budget' || res.reason === 'deadline') return { ok: false, reason: res.reason };
    const segs = [];
    if (takeoff) {
      const p = w.center(startNode);
      segs.push({ kind: 'takeoff', t0: takeoffStart, t1: res.path[0].t, p0: [p[0], p[1], startSite.padElev], p1: p, n0: startNode, n1: startNode, energy: (cost.pClimb * climbDur) / 3600 });
    }
    segs.push(...pathToSegments(res.path, res.tLand, { world: w, cost, landDur, padElev: site.padElev, landEnergy }));
    return { ok: true, segs, cost: res.cost, tLand: res.tLand, takeoffSlot, landSlot, takeoffStart, takeoffOcc, landStart, landOcc, climbDur, landDur, destId, expansions: res.expansions };
  }

  /** Install a trajectory: reservations in the airspace table + pad bookings. */
  commit(ac, traj, info, tNow) {
    this.table.removeId(ac.id);
    if (this.features.reservations) reserveTrajectory(this.table, this.world, ac.id, traj, tNow, { inflate: this._inflateFor(ac), padMult: this._padMult(ac) });
    // Only replace the booking kinds this plan actually re-decides: a replan of a parked aircraft must keep its pending take-off slot.
    if (info?.takeoffSlot) for (const vp of this.vertiports.values()) vp.release(ac.id, 'takeoff');
    if (info?.landSlot || info?.destId) for (const vp of this.vertiports.values()) vp.release(ac.id, 'land');
    if (info?.takeoffSlot) this.vertiports.get(ac.siteId).book(ac.id, info.takeoffSlot.pad, info.takeoffStart, info.takeoffStart + (info.takeoffOcc || info.climbDur + TAKEOFF_CLEAR), 'takeoff');
    if (info?.landSlot) { const vd = this.vertiports.get(info.destId); if (vd) vd.book(ac.id, info.landSlot.pad, info.landStart ?? info.tLand, (info.landStart ?? info.tLand) + (info.landOcc || info.landDur + TURNAROUND), 'land'); }
    ac.plan = traj; ac.destId = info?.destId ?? ac.destId;
  }

  // viability (emergency reachability)
  /** Check that along the trajectory the battery always covers: remaining plan energy + diversion to a landing site + reserve. */
  viabilityCheck(ac, traj, tNow) {
    const spec = ac.spec, cost = this.costFor(spec), field = this.emergencyField();
    const reserve = 0.6 * spec.reserveKwh;
    let e = ac.energy, minMargin = Infinity, failAt = null;
    for (const s of traj.segs) {
      if (s.t1 <= tNow) continue;
      const frac = s.t0 >= tNow ? 1 : (s.t1 - tNow) / Math.max(1e-9, s.t1 - s.t0);
      e -= (s.energy || 0) * frac * (ac.health?.drainMult || 1);
      if (s.kind === 'land') { minMargin = Math.min(minMargin, e - reserve * 0.5); continue; }
      const need = field.energyToLand(s.n1, cost) + reserve;
      const margin = e - need;
      if (margin < minMargin) { minMargin = margin; if (margin < 0 && !failAt) failAt = { t: s.t1, node: s.n1 }; }
    }
    return { ok: minMargin >= 0 && e >= 0, minMargin, failAt, endEnergy: e };
  }

  // public planning API
  /** Plan a full mission from the aircraft's vertiport (ac.siteId) to destId. */
  planMission(ac, destId, tNow) {
    const w = this.world, origin = w.siteById(ac.siteId);
    if (!origin) return { ok: false, reason: 'no-origin' };
    if (!this.siteUsable(origin, tNow + 5)) return { ok: false, reason: 'origin-unusable' };
    const kO = w.firstFreeLevel(origin.ix, origin.iy);
    const startNode = w.idx(origin.ix, origin.iy, kO);
    const climbDur = Math.max(1, (w.levels[kO] - origin.padElev) / ac.spec.vClimb);
    this.stats.plans++;
    const r = this.planLeg(ac, { startNode, t0: tNow + 2 + climbDur, destId, tNow, takeoff: true });
    if (!r.ok) { this.stats.failures++; if (r.reason === 'budget' || r.reason === 'deadline') this.stats.deadlines++; return r; }
    const traj = new Trajectory();
    for (const s of r.segs) traj.push(s);
    if (this.features.viability) {
      const v = this.viabilityCheck(ac, traj, tNow);
      if (!v.ok) { this.stats.viabilityRejects++; return { ok: false, reason: 'viability', margin: v.minMargin }; }
    } else if (ac.energy - traj.energyPlanned < 0.3 * ac.spec.reserveKwh) {
      return { ok: false, reason: 'energy' };
    }
    this.commit(ac, traj, r, tNow);
    return { ok: true, traj, tLand: r.tLand, eta: r.tLand + r.landDur, energy: traj.energyPlanned };
  }

  /** Join a frozen prefix and freshly planned segments, inserting a hover if planning starts later than the prefix ends. */
  _join(cm, r, t0, ac) {
    const traj = new Trajectory();
    for (const s of cm.prefix.segs) traj.push(s);
    if (t0 > cm.t + 1e-6) {
      const p = this.world.center(cm.node);
      traj.push({ kind: 'hold', t0: cm.t, t1: t0, p0: p, p1: p, n0: cm.node, n1: cm.node, energy: this.costFor(ac.spec).holdEnergy(t0 - cm.t) });
    }
    for (const s of r.segs) traj.push(s);
    return traj;
  }

  /** Replan the remainder of an in-flight aircraft's route, keeping the next few seconds frozen. */
  replan(ac, tNow, { destId = null, reason = '' } = {}) {
    if (!ac.plan) return { ok: false, reason: 'no-plan' };
    const cm = ac.plan.commitPrefix(tNow + PLANNER.commitLead);
    if (!cm) return { ok: false, reason: 'committed' };
    const dest = destId || ac.destId;
    this.stats.replans++;
    const t0 = Math.max(cm.t, tNow + 1);
    const r = this.planLeg(ac, { startNode: cm.node, t0, destId: dest, tNow, takeoff: false });
    if (!r.ok) { if (r.reason === 'budget' || r.reason === 'deadline') this.stats.deadlines++; return { ...r, ok: false }; }
    const traj = this._join(cm, r, t0, ac);
    if (this.features.viability) {
      const v = this.viabilityCheck(ac, traj, tNow);
      if (!v.ok) return { ok: false, reason: 'viability', margin: v.minMargin };
    }
    this.commit(ac, traj, r, tNow);
    return { ok: true, traj, eta: r.tLand + r.landDur, reason };
  }

  /**
   * Recovery replan for an aircraft that has left its plan (tracking error, lag): start from the lattice node
   * nearest to its ACTUAL position, after a short bridging move, discarding the stale future.
   */
  replanFromState(ac, tNow, { destId = null, reason = '' } = {}) {
    const w = this.world, spec = ac.spec;
    const dest = destId || ac.destId;
    if (!dest) return { ok: false, reason: 'no-dest' };
    const node = this._usableNodeNear(ac.p);
    if (node < 0) return { ok: false, reason: 'no-node' };
    const c = w.center(node);
    const dist = Math.hypot(c[0] - ac.p[0], c[1] - ac.p[1], c[2] - ac.p[2]);
    const bridge = Math.max(2, dist / (0.5 * spec.vCruise));
    const t0 = tNow + bridge;
    this.stats.replans++;
    const r = this.planLeg(ac, { startNode: node, t0, destId: dest, tNow, takeoff: false });
    if (!r.ok) { if (r.reason === 'budget' || r.reason === 'deadline') this.stats.deadlines++; return { ...r, ok: false }; }
    const traj = new Trajectory();
    traj.push({ kind: 'move', t0: tNow, t1: t0, p0: [ac.p[0], ac.p[1], ac.p[2]], p1: c, n0: node, n1: node, energy: (this.costFor(spec).pCruise * bridge) / 3600 });
    for (const s of r.segs) traj.push(s);
    if (this.features.viability) {
      const v = this.viabilityCheck(ac, traj, tNow);
      if (!v.ok) return { ok: false, reason: 'viability', margin: v.minMargin };
    }
    this.commit(ac, traj, r, tNow);
    return { ok: true, traj, eta: r.tLand + r.landDur, reason };
  }

  /** Re-register an aircraft's existing plan with its current uncertainty (padding / inflation). */
  rebook(ac, tNow) {
    if (!ac.plan) return;
    this.table.removeId(ac.id);
    if (this.features.reservations) reserveTrajectory(this.table, this.world, ac.id, ac.plan, tNow, { inflate: this._inflateFor(ac), padMult: this._padMult(ac), fromT: tNow - 1 });
  }

  /** Divert to the best alternative site (cooperative planning first, emergency policy as fallback). */
  divert(ac, tNow, { reason = '', exclude = [] } = {}) {
    const w = this.world;
    const cands = w.landingSites().filter((s) => !exclude.includes(s.id) && this.siteUsable(s, tNow))
      .map((s) => ({ s, d: Math.hypot(s.x - ac.p[0], s.y - ac.p[1]) })).sort((a, b) => a.d - b.d).slice(0, 4);
    for (const { s } of cands) {
      const r = this.replan(ac, tNow, { destId: s.id, reason });
      if (r.ok) return { ok: true, site: s, mode: 'planned', eta: r.eta };
      if (r.reason === 'committed') break;
    }
    return this.emergencyLanding(ac, tNow, { reason });
  }

  _usableNodeNear(p) {
    const w = this.world, field = this.emergencyField();
    const base = w.nodeAt(p[0], p[1], p[2]);
    if (field.time[base] < Infinity) return base;
    const [bx, by, bz] = w.decode(base);
    let best = -1, bt = Infinity;
    for (let dz = -1; dz <= 1; dz++) for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
      const x = bx + dx, y = by + dy, z = bz + dz;
      if (x < 0 || y < 0 || z < 0 || x >= w.nx || y >= w.ny || z >= w.nz) continue;
      const n = w.idx(x, y, z);
      if (field.time[n] < bt) { bt = field.time[n]; best = n; }
    }
    return best;
  }

  /**
   * Emergency / contingency landing. Follows the precomputed distance-field policy to the nearest landing
   * site (no search, so it cannot time out), reserves the corridor with priority and returns the conflicting
   * ids the caller must repair.
   */
  emergencyLanding(ac, tNow, { reason = '' } = {}) {
    const w = this.world, spec = ac.spec, field = this.emergencyField();
    const tf = ac.health?.thrustFactor ?? 1;
    const speed = Math.max(6, Math.min(spec.vCruise * 0.7, vMaxEff(spec, tf) * 0.75));
    const node0 = this._usableNodeNear(ac.p);
    const cost = this.costFor(spec);
    this.stats.emergencies++;
    if (node0 < 0 || ac.energy < field.energyToLand(node0, cost, 1.0) * 0.9 || tf < 0.2) return this.forcedLanding(ac, tNow, { reason: reason || 'insufficient energy/thrust' });
    const path = field.pathFrom(node0), site = field.siteOf(node0);
    const traj = new Trajectory();
    const start = [ac.p[0], ac.p[1], ac.p[2]], c0 = w.center(node0);
    const bridge = Math.max(1, Math.hypot(c0[0] - start[0], c0[1] - start[1], c0[2] - start[2]) / speed);
    let t = tNow;
    traj.push({ kind: 'move', t0: t, t1: t + bridge, p0: start, p1: c0, n0: node0, n1: node0, energy: (cost.pCruise * bridge) / 3600 });
    t += bridge;
    for (let i = 0; i + 1 < path.length; i++) {
      const a = path[i], b = path[i + 1], pa = w.center(a), pb = w.center(b);
      const vertical = pa[0] === pb[0] && pa[1] === pb[1];
      const dur = vertical ? Math.abs(pb[2] - pa[2]) / spec.vDescend : Math.hypot(pb[0] - pa[0], pb[1] - pa[1]) / speed;
      traj.push({ kind: vertical ? 'vmove' : 'move', t0: t, t1: t + dur, p0: pa, p1: pb, n0: a, n1: b, energy: (cost.pCruise * dur) / 3600 });
      t += dur;
    }
    const last = path[path.length - 1], pl = w.center(last);
    const landDur = Math.max(1, (pl[2] - site.padElev) / (spec.vDescend * Math.max(0.6, tf)));
    traj.push({ kind: 'land', t0: t, t1: t + landDur, p0: pl, p1: [pl[0], pl[1], site.padElev], n0: last, n1: last, energy: (cost.pDesc * landDur) / 3600 });
    this.table.removeId(ac.id);
    for (const vp of this.vertiports.values()) vp.release(ac.id);
    reserveTrajectory(this.table, w, ac.id, traj, tNow, { inflate: 0, padMult: 2 });
    const vd = this.vertiports.get(site.id);
    if (vd) vd.book(ac.id, 0, t, t + landDur + TURNAROUND, 'land');
    ac.plan = traj; ac.destId = site.id;
    const conflicts = [...this.table.conflictsOf(ac.id, tNow)].filter((id) => typeof id === 'number');
    return { ok: true, mode: 'emergency', site, eta: t + landDur, conflicts };
  }

  /** Last resort: descend where we are. */
  forcedLanding(ac, tNow, { reason = '' } = {}) {
    const w = this.world, spec = ac.spec;
    this.stats.forced++;
    const surf = w.terrainFast(ac.p[0], ac.p[1]) + w.bldg[w.cellIndex(ac.p[0], ac.p[1])];
    const dur = Math.max(1, (ac.p[2] - surf) / (spec.vDescend * 1.2));
    const traj = new Trajectory();
    const n = w.nodeAt(ac.p[0], ac.p[1], ac.p[2]);
    traj.push({ kind: 'land', t0: tNow, t1: tNow + dur, p0: [...ac.p], p1: [ac.p[0], ac.p[1], surf], n0: n, n1: n, energy: 0, forced: true });
    this.table.removeId(ac.id);
    reserveTrajectory(this.table, w, ac.id, traj, tNow, { inflate: 0, padMult: 2 });
    ac.plan = traj; ac.destId = null; ac.forced = true;
    return { ok: true, mode: 'forced', site: null, eta: tNow + dur, reason, conflicts: [] };
  }

  // forced reservations
  /** Hard spatiotemporal restriction. Starts at t0 (+grace so aircraft already inside can leave). */
  addZoneReservation(zone, t0, t1 = Infinity) {
    const id = `ZONE:${zone.id}`;
    this.table.removeId(id);
    for (const n of this.world.nodesInZone(zone)) this.table.add(id, n, t0, t1);
    return id;
  }
  removeZoneReservation(zone) { this.table.removeId(`ZONE:${zone.id}`); }

  /**
   * Register a predicted straight-line track of external traffic as reservations (cells within its
   * uncertainty-inflated radius), horizon seconds ahead.
   */
  addExternalTrack(track, tNow, horizon = 240) {
    const id = `EXT:${track.id}`;
    this.table.removeId(id);
    const w = this.world;
    const speed = Math.max(1, Math.hypot(track.v[0], track.v[1]));
    const step = Math.max(0.25, (0.5 * w.cell) / speed);
    const runs = new Map();
    for (let dt = 0; dt <= horizon; dt += step) {
      const t = tNow + dt;
      const x = track.p[0] + track.v[0] * dt, y = track.p[1] + track.v[1] * dt, z = track.p[2] + (track.v[2] || 0) * dt;
      if (!w.inBounds(x, y)) break;
      const sigma = (track.sigma || 20) + 0.8 * dt;
      const k = Math.min(2, Math.ceil((track.radius + sigma) / w.cell - 0.5));
      const iz = w.levelOf(z);
      if (Math.abs(w.levels[iz] - z) > 50 + sigma * 0.3) continue; // not at a flight level we use
      const c0 = w.cellIndex(x, y), cx = c0 % w.nx, cy = (c0 - cx) / w.nx;
      for (let dy = -k; dy <= k; dy++) for (let dx = -k; dx <= k; dx++) {
        const jx = cx + dx, jy = cy + dy;
        if (jx < 0 || jy < 0 || jx >= w.nx || jy >= w.ny) continue;
        const n = w.idx(jx, jy, iz), pad = 1.5 + step;
        const r = runs.get(n);
        if (r && t - pad <= r.e) r.e = t + pad; else { if (r) this.table.add(id, n, r.s, r.e); runs.set(n, { s: t - pad, e: t + pad }); }
      }
    }
    for (const [n, r] of runs) this.table.add(id, n, r.s, r.e);
    return id;
  }
  removeExternalTrack(trackId) { this.table.removeId(`EXT:${trackId}`); }

  // localized repair (LNS)
  /**
   * Re-plan a cluster of aircraft whose reservations conflict. Tries several priority orders and keeps the best
   * (fewest failures, then lowest total arrival time). `getAc(id)` resolves aircraft objects.
   * @returns {{recovered:number, failed:number[], rounds:number}}
   */
  repairCluster(ids, tNow, getAc) {
    const cluster = ids.map(getAc).filter((a) => a && a.plan && !a.frozen && !a.emergencyMode);
    if (!cluster.length) return { recovered: 0, failed: [], rounds: 0 };
    this.stats.lnsRuns++;
    const rounds = this.features.lnsRepair ? PLANNER.lnsRounds : 1;
    const snapshot = cluster.map((a) => ({ a, plan: a.plan, destId: a.destId }));
    const prefixOf = (a) => a.plan.commitPrefix(tNow + PLANNER.commitLead);
    let best = null;
    for (let round = 0; round < rounds; round++) {
      const order = cluster.slice();
      if (round === 0) order.sort((a, b) => a.energy - b.energy); // low battery first
      else for (let i = order.length - 1; i > 0; i--) { const j = this.rng.int(i + 1); [order[i], order[j]] = [order[j], order[i]]; }
      // freeze the committed prefixes
      const pre = new Map();
      for (const a of order) {
        const cm = prefixOf(a);
        pre.set(a.id, cm);
        this.table.removeId(a.id);
        if (cm) reserveTrajectory(this.table, this.world, a.id, cm.prefix, tNow, { inflate: this._inflateFor(a), padMult: this._padMult(a) });
      }
      const result = { plans: new Map(), failed: [], sum: 0 };
      for (const a of order) {
        const cm = pre.get(a.id);
        if (!cm) { result.failed.push(a.id); continue; }
        const t0 = Math.max(cm.t, tNow + 1);
        const r = this.planLeg(a, { startNode: cm.node, t0, destId: a.destId, tNow, takeoff: false });
        if (!r.ok) { result.failed.push(a.id); continue; }
        const traj = this._join(cm, r, t0, a);
        result.plans.set(a.id, { traj, info: r });
        this.table.removeId(a.id);
        reserveTrajectory(this.table, this.world, a.id, traj, tNow, { inflate: this._inflateFor(a), padMult: this._padMult(a) });
        result.sum += r.tLand;
      }
      const score = result.failed.length * 1e6 + result.sum;
      if (!best || score < best.score) best = { ...result, score };
      // roll back before the next round
      for (const a of order) this.table.removeId(a.id);
      if (!result.failed.length && round >= 0 && !this.features.lnsRepair) break;
    }
    // apply best
    for (const { a, plan } of snapshot) {
      this.table.removeId(a.id);
      const chosen = best.plans.get(a.id);
      if (chosen) this.commit(a, chosen.traj, chosen.info, tNow);
      else { a.plan = plan; reserveTrajectory(this.table, this.world, a.id, plan, tNow, { inflate: this._inflateFor(a), padMult: this._padMult(a) }); }
    }
    const recovered = best.plans.size;
    this.stats.lnsRecovered += recovered;
    return { recovered, failed: best.failed, rounds };
  }

  /** Aircraft ids (numeric) whose future reservations conflict with another owner's. */
  conflictingAircraft(ids, tNow) {
    const out = new Set();
    for (const id of ids) {
      for (const o of this.table.conflictsOf(id, tNow)) if (typeof o === 'number') { out.add(o); out.add(id); }
    }
    return [...out];
  }

  gc(t) { this.table.gc(t); for (const vp of this.vertiports.values()) vp.gc(t); }
}
