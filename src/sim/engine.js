// Simulation engine. One fixed-step loop (default 10 Hz) runs the physics; control runs at 5 Hz;
// monitors, dispatch and fleet coordination run at ~1 Hz; wind-map assimilation every 2 s.
//
//   per step:  environment -> external traffic -> [control tick: neighbours, MPC, CBF, proximity metrics]
//              -> plant (wind, dynamics, battery, navigation filter) -> ground ops -> [1 Hz: monitors,
//              dispatch, replanning queue, track registration] -> [2 s: wind observations]

import { defaultConfig, SIM, SAFETY, AIRCRAFT_CLASSES } from '../core/config.js';
import { RNG } from '../core/util.js';
import { World } from '../world/world.js';
import { TrueWind, ForecastWind, FleetWindMap, EstimatedWind } from '../env/wind.js';
import { Aircraft, PHASE, MODE } from '../aircraft/aircraft.js';
import { stepPlant } from '../aircraft/dynamics.js';
import { Coordinator } from '../planning/coordinator.js';
import { VertiportRuntime } from '../fleet/vertiports.js';
import { EventLog, Metrics } from './events.js';
import { TrafficGenerator } from './traffic.js';
import * as F from './faults.js';

const BASE_RATE = 1 / 420; // ride requests per idle, charged aircraft per second at demand = 1

export class Engine {
  constructor(opts = {}) {
    this.cfg = defaultConfig(opts);
    this.features = this.cfg.features;
    this.rng = new RNG(this.cfg.seed);
    this.world = opts.world || new World();
    const w = this.world;
    this.truth = new TrueWind(w, this.cfg.seed, this.cfg.wind);
    this.truth.turbulence = this.features.turbulence;
    this.forecast = new ForecastWind(this.truth, this.cfg.seed, { forecastBias: this.cfg.wind.forecastBias });
    this.map = new FleetWindMap(w);
    this.wind = new EstimatedWind(this.forecast, this.map, this.features);
    this.vertiports = new Map(w.vertiports.map((v) => [v.id, new VertiportRuntime(v, opts.fatos || 1)]));
    this.events = new EventLog();
    this.metrics = new Metrics();
    this.coord = new Coordinator({ world: w, wind: this.wind, features: this.features, vertiports: this.vertiports, rng: this.rng.fork('coord'), log: (...a) => this.log(...a) });
    this.t = 0; this.n = 0;
    this.aircraft = [];
    this.ext = [];
    this.traffic = new TrafficGenerator(w, this.rng.fork('traffic'));
    this.faultRecords = [];
    this.replanQueue = new Map();
    this.selectedId = null;
    this.planBudgetMs = opts.planBudgetMs ?? Infinity;
    this.stress = !!opts.stress; // stress mode: dispatch every idle aircraft as soon as it is able (see _dispatch)
    this.weatherStale = false;
    this._hash = new Map();
    this._pairs = new Set();
    this._windProbe = [];
    this._wTmp = [0, 0, 0]; this._wTmp2 = [0, 0, 0];
    this._lastExtReg = 0;
    this.stormEnd = 0;
    this.spawnFleet(this.cfg.fleetSize);
    const pr = this.rng.fork('probe');
    for (let i = 0; i < 28; i++) this._windProbe.push([pr.range(300, w.width - 300), pr.range(300, w.height - 300), w.levels[pr.int(w.nz)]]);
    if (this.features.stadiumTfr) this._activateTfr(true);
  }

  log(level, text, ac = null, meta = {}) { return this.events.add(this.t, level, text, { ac: ac ? ac.id : undefined, acName: ac ? ac.name : undefined, ...meta }); }
  getAc(id) { return this.aircraft[id]; }

  // setup
  spawnFleet(n) {
    const w = this.world, r = this.rng.fork('fleet');
    const sites = w.vertiports, weights = sites.map((s) => s.pads + 0.5 * s.chargers);
    const total = weights.reduce((a, b) => a + b, 0);
    const keys = ['compact', 'standard', 'standard', 'large', 'compact'];
    for (let i = 0; i < n; i++) {
      let x = r.float() * total, k = 0;
      while (k < sites.length - 1 && x > weights[k]) { x -= weights[k]; k++; }
      const ac = new Aircraft(this.aircraft.length, keys[i % keys.length], sites[k], r.fork('ac' + i), r.range(0.6, 1.0));
      ac.padMult = 1;
      this.aircraft.push(ac);
      this.vertiports.get(sites[k].id).hangar.add(ac.id);
    }
  }

  // feature toggles
  setFeature(name, value) {
    if (!(name in this.features)) return;
    const prev = this.features[name];
    this.features[name] = value;
    if (prev === value) return;
    const t = this.t;
    switch (name) {
      case 'turbulence': this.truth.turbulence = value; break;
      case 'windAware': case 'fleetWindMap': this.wind.bump(); this.coord.hcache.clear(); break;
      case 'stadiumTfr': this._activateTfr(value); break;
      case 'reservations':
        if (!value) this.coord.table = new (this.coord.table.constructor)();
        else for (const ac of this.aircraft) if (ac.airborne && ac.plan) this.coord.rebook(ac, t);
        break;
      case 'adaptiveEnvelope': for (const ac of this.aircraft) if (ac.airborne && ac.plan) this.coord.rebook(ac, t); break;
      case 'externalTraffic':
        if (!value) { for (const e of this.ext) this.coord.removeExternalTrack(e.id); this.ext = []; }
        break;
      default: break;
    }
    this.log('info', `Feature "${name}" ${value ? 'enabled' : 'disabled'}`);
  }

  setWind({ speed10, dirFrom, gust }) {
    if (speed10 !== undefined) this.truth.speed10 = speed10;
    if (dirFrom !== undefined) this.truth.dirFrom = dirFrom;
    if (gust !== undefined) this.truth.gustScale = gust;
    this.wind.bump();
  }

  _activateTfr(on) {
    const zone = this.world.zone('TFR');
    if (!zone) return;
    zone.active = on;
    if (on) {
      this.coord.addZoneReservation(zone, this.t + 75);
      this.coord.table.removeId('ZONE:none');
      this.world.version++; // sites/fields depend on airspace
      this.coord.hcache.clear();
      this.log('warn', 'Temporary flight restriction issued over the North Shore stadium (effective in 75 s)');
      const affected = [];
      for (const ac of this.aircraft) if (ac.airborne && ac.plan && this.coord.table.conflictsOf(ac.id, this.t).has('ZONE:TFR')) affected.push(ac.id);
      if (affected.length) this._repair(affected, 'TFR');
      for (const ac of this.aircraft) if (ac.airborne && ac.destId && !this.coord.siteUsable(this.world.siteById(ac.destId), this.t + 60) && !affected.includes(ac.id)) this.requestReplan(ac, 'destination restricted');
    } else {
      this.coord.removeZoneReservation(zone);
      this.world.version++; this.coord.hcache.clear();
      this.log('ok', 'Stadium TFR cancelled');
    }
  }

  setVertiportClosed(id, closed) {
    const vp = this.vertiports.get(id);
    if (!vp || vp.closed === closed) return;
    vp.closed = closed;
    this.world.version++; this.coord.hcache.clear();
    this.log(closed ? 'warn' : 'ok', `Vertiport ${vp.name} ${closed ? 'CLOSED' : 'reopened'}`);
    if (closed) {
      for (const ac of this.aircraft) if (ac.airborne && ac.destId === id) this.requestReplan(ac, `${vp.name} closed`);
    }
  }

  // main loop
  advance(seconds) { const n = Math.round(seconds / SIM.dt); for (let i = 0; i < n; i++) this.step(); }

  step() {
    const dt = SIM.dt, w = this.world, f = this.features;
    this.n++; this.t += dt;
    const t = this.t;
    // external traffic
    if (f.externalTraffic) {
      const e = this.traffic.tick(t);
      if (e) this.ext.push(e);
      for (const x of this.ext) x.step(dt);
      if (this.n % 50 === 0) this.ext = this.ext.filter((x) => { const out = x.p[0] < -400 || x.p[1] < -400 || x.p[0] > w.width + 400 || x.p[1] > w.height + 400; if (out && x.age > 5) this.coord.removeExternalTrack(x.id); return !(out && x.age > 5); });
    }
    const controlTick = this.n % SIM.mpcEvery === 0;
    if (controlTick) this._controlTick();
    // plant
    const wind = this._wTmp;
    for (const ac of this.aircraft) {
      if (ac.phase === PHASE.FLYING) this._stepAirborne(ac, wind, dt);
      else this._stepGround(ac, dt);
    }
    if (this.n % SIM.monitorEvery === 0) this._monitorTick();
    if (this.n % SIM.windMapEvery === 0) this._windObservations();
  }

  // control tick
  _buildHash() {
    const h = this._hash; h.clear();
    const add = (e) => { const key = Math.floor(e.p[0] / 250) * 4096 + Math.floor(e.p[1] / 250); let a = h.get(key); if (!a) { a = []; h.set(key, a); } a.push(e); };
    for (const ac of this.aircraft) if (ac.phase === PHASE.FLYING) add(ac);
    for (const x of this.ext) add(x);
  }

  _neighbours(ac, range, out) {
    const cx = Math.floor(ac.p[0] / 250), cy = Math.floor(ac.p[1] / 250), k = Math.ceil(range / 250);
    out.length = 0;
    for (let dx = -k; dx <= k; dx++) for (let dy = -k; dy <= k; dy++) {
      const a = this._hash.get((cx + dx) * 4096 + (cy + dy));
      if (!a) continue;
      for (const o of a) if (o !== ac) out.push(o);
    }
    return out;
  }

  _controlTick() {
    const t = this.t, dtc = SIM.dt * SIM.mpcEvery;
    this._buildHash();
    const cand = [], used = new Set();
    for (const ac of this.aircraft) {
      if (ac.phase !== PHASE.FLYING) continue;
      this._neighbours(ac, SAFETY.cbfRange, cand);
      const nbs = [];
      for (const o of cand) {
        const isExt = o.kind !== undefined && !(o instanceof Aircraft);
        const d = Math.hypot(ac.p[0] - o.p[0], ac.p[1] - o.p[1], ac.p[2] - o.p[2]);
        // ---- ground-truth proximity metrics (independent of what the aircraft believes)
        if (isExt || o.id > ac.id) {
          const key = isExt ? `${ac.id}|${o.id}` : `${ac.id}|${o.id}`;
          if (d < SAFETY.dLoss * 1.5) used.add(key);
          if (d < this.metrics.minSep) this.metrics.minSep = d;
          if (d < SAFETY.dLoss && !this._pairs.has(key)) {
            if (d < SAFETY.dCollision) { this.metrics.collisions++; this.log('alert', `COLLISION ${ac.name} / ${isExt ? o.id : o.name} (${d.toFixed(0)} m)`, ac); }
            else { this.metrics.los++; this.log('alert', `Loss of separation ${ac.name} / ${isExt ? o.id : o.name}: ${d.toFixed(0)} m`, ac); }
          }
        }
        // ---- what the aircraft can use for avoidance: broadcast estimates (cooperative) or sensed tracks (external)
        if (isExt) nbs.push({ p: o.p, v: o.v, sigma: o.sigma, coop: false });
        else nbs.push({ p: o.nav.p, v: o.v, sigma: o.nav.sigma, coop: !o.health.commLoss });
      }
      ac.nbrCount = nbs.length;
      ac.control({ t, dt: dtc, features: this.features, world: this.world, neighbors: nbs });
      if (ac.cbfActive) this.metrics.cbfInterventions++;
    }
    this._pairs = used;
  }

  // plant
  _stepAirborne(ac, wind, dt) {
    const t = this.t, w = this.world;
    this.truth.at(ac.p[0], ac.p[1], ac.p[2], t, wind);
    ac.nav.predict(ac.v, dt);
    if (this.n % 2 === 0) {
      const r = ac.nav.update(ac.p, t);
      if (r === 'gps_fault') F.onGpsFault(this, ac);
    }
    const pos = ac.nav.p;
    ac.sigmaPos = ac.nav.sigma;
    stepPlant(ac, wind, dt);
    if (ac.extraAccel) {
      if (t < ac.extraAccel.until) { ac.v[0] += ac.extraAccel.a[0] * dt; ac.v[1] += ac.extraAccel.a[1] * dt; ac.v[2] += ac.extraAccel.a[2] * dt; } else ac.extraAccel = null;
    }
    void pos;
    // energy exhaustion => propulsion loss
    if (ac.energy <= 0 && ac.health.thrustFactor > 0.06) {
      ac.energy = 0; ac.health.thrustFactor = 0.05;
      this.log('alert', `${ac.name} battery exhausted — power loss`, ac);
    }
    // ground contact
    const surf = w.terrainFast(ac.p[0], ac.p[1]) + w.bldg[w.cellIndex(ac.p[0], ac.p[1])];
    const plan = ac.activePlan();
    const last = plan && !plan.empty ? plan.segs[plan.segs.length - 1] : null;
    if (ac.p[2] < surf - 1.0 && !(last && last.kind === 'land' && t >= last.t0 - 0.2 && ac.v[2] > -6)) {
      this._crash(ac, surf);
      return;
    }
    // touchdown at the end of the landing segment
    if (plan && last && last.kind === 'land' && t >= last.t1) this._touchdown(ac, last);
  }

  _crash(ac, surf) {
    ac.phase = PHASE.CRASHED; ac.p[2] = surf; ac.v = [0, 0, 0]; ac.status = 'CRASHED';
    this.metrics.crashes++; this.coord.table.removeId(ac.id);
    for (const vp of this.vertiports.values()) vp.release(ac.id);
    this.log('alert', `${ac.name} CRASHED (${ac.health.thrustFactor < 0.75 ? 'propulsion loss' : 'ground impact'})`, ac);
    F.resolveFaults(this, ac, 'crash');
  }

  _touchdown(ac, seg) {
    const w = this.world;
    ac.p = [seg.p1[0], seg.p1[1], seg.p1[2]]; ac.v = [0, 0, 0]; ac.aCmd = [0, 0, 0];
    this.coord.table.removeId(ac.id);
    const dest = ac.destId ? w.siteById(ac.destId) : null;
    const vp = dest ? this.vertiports.get(dest.id) : null;
    const mission = ac.mission;
    ac.plan = null; ac.flown = null; ac.mpcWarm = null; ac.trail.length = 0;
    this.metrics.peakAccel.push(ac.peakAccel);
    if (ac.jerkN) this.metrics.jerkRms.push(Math.sqrt(ac.jerkSq / ac.jerkN));
    ac.peakAccel = 0; ac.jerkSq = 0; ac.jerkN = 0;
    if (vp) {
      ac.phase = PHASE.GROUND; ac.siteId = vp.id; ac.status = 'parked';
      vp.hangar.add(ac.id);
      if (mission) {
        if (!ac.emergencyMode) {
          this.metrics.completed++;
          const dur = this.t - mission.t0;
          this.metrics.trips.push({ id: ac.id, dur, ideal: mission.ideal, energy: Math.max(0, mission.e0 - ac.energy) });
          if (mission.to !== vp.id) this.metrics.diversions++;
        }
        ac.mission = null;
      }
      if (ac.emergencyMode) { this.metrics.emergencyLandings++; this.log('ok', `${ac.name} landed safely at ${vp.name} after emergency`, ac); }
      F.resolveFaults(this, ac, 'landed');
      ac.repair();
    } else {
      ac.phase = PHASE.GROUNDED; ac.groundedSince = this.t; ac.status = 'GROUNDED';
      if (ac.forced) { this.metrics.forcedLandings++; this.log('warn', `${ac.name} forced landing at (${ac.p[0].toFixed(0)}, ${ac.p[1].toFixed(0)})`, ac); }
      else { this.metrics.emergencyLandings++; this.log('ok', `${ac.name} emergency landing at ${dest ? dest.name : 'unknown site'} — safe`, ac); }
      ac.mission = null;
      F.resolveFaults(this, ac, ac.forced ? 'forced landing' : 'emergency landing');
    }
  }

  _stepGround(ac, dt) {
    const t = this.t;
    if (ac.phase === PHASE.GROUND) {
      const vp = this.vertiports.get(ac.siteId);
      if (ac.plan && !ac.plan.empty && t >= ac.plan.t0 - 0.05) { // take off
        ac.phase = PHASE.FLYING; ac.status = 'takeoff';
        vp.hangar.delete(ac.id); vp.stopCharging(ac.id);
        ac.nav.reset(ac.p); ac.mpcWarm = null; ac._vPrev = null; ac.lastHeartbeat = t;
        if (ac.mission) ac.mission.e0 = ac.energy;
        return;
      }
      if (vp && ac.energy < ac.spec.usableKwh - 1e-6) {
        if (vp.tryStartCharging(ac.id)) {
          const soc = ac.soc, taper = soc > 0.8 ? Math.max(0.3, 1 - (soc - 0.8) / 0.2 * 0.7) : 1;
          ac.energy = Math.min(ac.spec.usableKwh, ac.energy + (ac.spec.chargeKw * taper * dt) / 3600);
          ac.status = 'charging';
        } else ac.status = 'waiting for charger';
      } else if (vp) { vp.stopCharging(ac.id); ac.status = ac.plan ? 'assigned' : 'ready'; }
    } else if (ac.phase === PHASE.GROUNDED && t - ac.groundedSince > 240) {
      this._recover(ac);
    }
  }

  _recover(ac) {
    const vps = this.world.vertiports.filter((v) => !v.closed).sort((a, b) => Math.hypot(a.x - ac.p[0], a.y - ac.p[1]) - Math.hypot(b.x - ac.p[0], b.y - ac.p[1]));
    const v = vps[0];
    ac.p = [v.x, v.y, v.padElev]; ac.v = [0, 0, 0]; ac.phase = PHASE.GROUND; ac.siteId = v.id; ac.status = 'parked';
    ac.energy = Math.max(ac.energy, 0.4 * ac.spec.usableKwh);
    this.vertiports.get(v.id).hangar.add(ac.id);
    ac.repair(); ac.plan = null;
    this.log('info', `${ac.name} recovered by ground crew to ${v.name}`, ac);
  }

  // 1 Hz monitors + fleet tasks
  _monitorTick() {
    const t = this.t, f = this.features;
    F.faultMonitor(this, t);
    for (const ac of this.aircraft) {
      if (ac.phase !== PHASE.FLYING) continue;
      ac.status = ac.emergencyMode ? 'EMERGENCY' : ac.mode === MODE.DEGRADED ? 'degraded' : ac.status === 'takeoff' ? 'climbing' : 'cruise';
      const seg = ac.activePlan() ? ac.activePlan().ref(t, [0, 0, 0], [0, 0, 0])?.seg : null;
      if (seg) ac.status = ac.emergencyMode ? 'EMERGENCY' : ac.mode === MODE.DEGRADED ? 'degraded' : seg.kind === 'takeoff' ? 'takeoff' : seg.kind === 'land' ? 'landing' : seg.kind === 'hold' ? 'holding' : 'cruise';
      if (!ac.health.commLoss) ac.lastHeartbeat = t;
      ac.trail.push([ac.p[0], ac.p[1], ac.p[2]]); if (ac.trail.length > 40) ac.trail.shift();
      if (!f.faultResponse) continue;
      // communication heartbeat
      if (ac.health.commLoss && !ac.frozen && t - ac.lastHeartbeat > ac.spec.heartbeatTimeout) F.onCommLoss(this, ac);
      // tracking deviation (wind lag, route deviation, navigation error) => invalidate reservations + replan
      if (!ac.frozen && !ac.emergencyMode && ac.plan) {
        const seg2 = seg ? seg.kind : '';
        if (ac.trackErr > SAFETY.trackingDeviationLimit && seg2 !== 'takeoff' && seg2 !== 'land') ac.devTime += 1; else ac.devTime = Math.max(0, ac.devTime - 2);
        if (ac.devTime >= SAFETY.trackingDeviationTime) {
          ac.devTime = 0; this.metrics.deviationReplans++;
          this.log('warn', `${ac.name} off route by ${ac.trackErr.toFixed(0)} m — invalidating reservations, replanning`, ac);
          this.requestReplan(ac, 'route deviation', true);
        }
      }
      // runtime assurance: controller trouble => backup (emergency) behaviour
      if (!ac.emergencyMode && (ac.mpcBadTime > 10 || ac.cbfBadTime > 4)) {
        this.metrics.rtaSwitches++;
        this.log('warn', `${ac.name} runtime assurance: ${ac.cbfBadTime > 4 ? 'CBF infeasible' : 'MPC cannot track'} — switching to backup controller`, ac);
        ac.mpcBadTime = 0; ac.cbfBadTime = 0;
        this.enterEmergency(ac, 'runtime-assurance switch');
      }
      // energy viability (emergency reachability): can I still get to a landing site with reserve?
      if (f.viability && !ac.emergencyMode && ac.plan) this._viabilityMonitor(ac, t);
    }
    // fleet-level tasks
    this._processReplans();
    this._registerTracks(t);
    this._dispatch(t);
    if (this.n % (SIM.monitorEvery * 5) === 0) { this.coord.gc(t); this.truth.pruneDisturbances(t); this.wind.pruneHazards(t); }
    if (this.n % (SIM.monitorEvery * 10) === 0) this._sample(t);
  }

  _viabilityMonitor(ac, t) {
    const field = this.coord.emergencyField(), cost = this.coord.costFor(ac.spec);
    const node = this.coord._usableNodeNear(ac.p);
    if (node < 0) return;
    const need = field.energyToLand(node, cost, 1.15);
    const reserve = 0.35 * ac.spec.reserveKwh;
    let remainPlan = 0;
    const plan = ac.plan;
    for (const s of plan.segs) { if (s.t1 <= t) continue; const fr = s.t0 >= t ? 1 : (s.t1 - t) / Math.max(1e-9, s.t1 - s.t0); remainPlan += (s.energy || 0) * fr; }
    remainPlan *= ac.health.drainMult;
    if (need + reserve > ac.energy) {
      if (!ac._lowBatt) { ac._lowBatt = true; F.onLowBattery(this, ac, 'emergency'); }
    } else if (ac.energy - remainPlan < 0.5 * ac.spec.reserveKwh && !ac._lowBatt && ac.health.drainMult > 1.2) {
      ac._lowBatt = true; F.onLowBattery(this, ac, 'divert');
    }
  }

  _processReplans() {
    if (!this.replanQueue.size) return;
    const t = this.t, tStart = performance.now();
    for (const [id, req] of [...this.replanQueue]) {
      this.replanQueue.delete(id);
      const ac = this.aircraft[id];
      if (!ac || ac.phase !== PHASE.FLYING || !ac.plan || ac.emergencyMode) continue;
      if (ac.frozen) continue;
      const r = req.fromState ? this.coord.replanFromState(ac, t, { reason: req.reason }) : this.coord.replan(ac, t, { reason: req.reason });
      if (r.ok) { this.log('info', `${ac.name} replanned (${req.reason}), ETA ${(r.eta - t).toFixed(0)} s`, ac); continue; }
      if (r.reason === 'budget' || r.reason === 'deadline') { F.onPlannerTimeout(this, ac, req.reason); continue; }
      if (r.reason === 'committed') continue; // landing already underway
      // could not reach the original destination -> divert
      this.divertAircraft(ac, `${req.reason}: ${r.reason}`);
      if (performance.now() - tStart > this.planBudgetMs) break;
    }
  }

  requestReplan(ac, reason, fromState = false) {
    if (!this.features.faultResponse && reason === 'route deviation') return;
    const prev = this.replanQueue.get(ac.id);
    this.replanQueue.set(ac.id, { reason, fromState: fromState || (prev && prev.fromState) });
  }

  /** Divert to the best alternative landing site; falls back to the emergency policy. */
  divertAircraft(ac, reason, exclude = []) {
    const t = this.t;
    const r = this.coord.divert(ac, t, { reason, exclude });
    if (!r.ok) return r;
    if (r.mode === 'planned') {
      ac.mode = MODE.DEGRADED;
      this.log('warn', `${ac.name} diverting to ${r.site.name} (${reason})`, ac);
    } else {
      ac.emergencyMode = true; ac.mode = MODE.EMERGENCY;
      this.log('alert', `${ac.name} EMERGENCY ${r.mode === 'forced' ? 'forced landing' : 'landing at ' + (r.site ? r.site.name : '?')} (${reason})`, ac);
      if (r.conflicts && r.conflicts.length) this._repair(r.conflicts, 'emergency corridor');
    }
    return r;
  }

  /** Enter emergency mode: contingency landing via the precomputed policy. */
  enterEmergency(ac, reason) {
    const r = this.coord.emergencyLanding(ac, this.t, { reason });
    ac.emergencyMode = true; ac.mode = MODE.EMERGENCY;
    if (ac.health.commLoss) ac.flown = ac.plan; // onboard autonomy decided this locally
    this.log('alert', `${ac.name} EMERGENCY → ${r.mode === 'forced' ? 'forced landing' : r.site ? r.site.name : '?'} (${reason})`, ac);
    if (r.conflicts && r.conflicts.length) this._repair(r.conflicts, 'emergency corridor');
    return r;
  }

  _repair(ids, why) {
    const all = this.coord.conflictingAircraft(ids, this.t);
    const set = new Set([...ids, ...all]);
    const res = this.coord.repairCluster([...set], this.t, (id) => this.aircraft[id]);
    if (set.size) this.log('info', `Cluster repair (${why}): ${res.recovered} of ${set.size} aircraft replanned in ${res.rounds} round(s)${res.failed.length ? `, ${res.failed.length} failed → diverting` : ''}`);
    for (const id of res.failed) { const ac = this.aircraft[id]; if (ac && ac.airborne && !ac.emergencyMode) this.divertAircraft(ac, `${why}: no feasible route`); }
    return res;
  }

  _registerTracks(t) {
    if (!this.features.externalTraffic || !this.features.reservations) return;
    if (t - this._lastExtReg < 8) return;
    this._lastExtReg = t;
    const hit = new Set();
    for (const x of this.ext) {
      if (x.p[0] < -200 || x.p[1] < -200 || x.p[0] > this.world.width + 200 || x.p[1] > this.world.height + 200) { if (x.age > 5) continue; }
      const id = this.coord.addExternalTrack({ id: x.id, p: x.p, v: x.v, radius: x.radius + 12, sigma: x.sigma }, t, 200);
      for (const o of this.coord.table.conflictsOf(id, t)) if (typeof o === 'number') hit.add(o);
    }
    if (hit.size) this._repair([...hit].filter((id) => this.aircraft[id] && this.aircraft[id].airborne), 'external traffic');
  }

  // dispatch
  _pickDest(ac) {
    const w = this.world, o = w.siteById(ac.siteId);
    const cands = [];
    for (const v of w.vertiports) {
      if (v.id === ac.siteId || v.closed || !this.coord.siteUsable(v, this.t + 120)) continue;
      const d = Math.hypot(v.x - o.x, v.y - o.y);
      if (d < 1500) continue;
      cands.push({ v, wgt: (v.pads + 1) / (d / 1000) });
    }
    if (!cands.length) return null;
    const tot = cands.reduce((s, c) => s + c.wgt, 0);
    let x = this.rng.float() * tot;
    for (const c of cands) { x -= c.wgt; if (x <= 0) return c.v; }
    return cands[cands.length - 1].v;
  }

  _dispatch(t) {
    if (this.weatherStale) return;
    const lambda = BASE_RATE * this.cfg.demand;
    const t0 = performance.now();
    const thr = SIM.chargeThreshold;
    for (const ac of this.aircraft) {
      if (ac.phase !== PHASE.GROUND || ac.plan || t < ac.nextTry) continue;
      if (ac.soc < thr) continue;
      if (!this.stress && !this.rng.chance(lambda)) continue; // stress mode: every idle, charged aircraft requests a ride immediately
      const dest = this._pickDest(ac);
      if (!dest) continue;
      this.metrics.requested++;
      const origin = this.world.siteById(ac.siteId);
      const r = this.coord.planMission(ac, dest.id, t);
      if (r.ok) {
        const dist = Math.hypot(dest.x - origin.x, dest.y - origin.y);
        ac.mission = { t0: t, from: ac.siteId, to: dest.id, ideal: dist / ac.spec.vCruise + 30 + 30, e0: ac.energy };
        ac.status = 'assigned';
      } else {
        this.metrics.reject(r.reason);
        ac.nextTry = t + 8 + this.rng.range(0, 20);
      }
      if (performance.now() - t0 > this.planBudgetMs) break;
    }
  }

  // wind observations + fleet map
  _windObservations() {
    const t = this.t, f = this.features, rng = this.rng;
    const W = this._wTmp2;
    for (const ac of this.aircraft) {
      if (ac.phase !== PHASE.FLYING) continue;
      this.truth.at(ac.p[0], ac.p[1], ac.p[2], t, W);
      // wind estimate = ground velocity - air velocity estimate (with sensor noise); aircraft only know their own nav estimate
      const ox = W[0] + 0.8 * rng.gauss(), oy = W[1] + 0.8 * rng.gauss();
      const fc = this.forecast.mean(ac.nav.p[0], ac.nav.p[1], ac.nav.p[2], t, this._c || (this._c = [0, 0, 0]));
      if (f.fleetWindMap && !ac.health.commLoss) this.map.update(ac.nav.p[0], ac.nav.p[1], ac.nav.p[2], t, ox - fc[0], oy - fc[1]);
      // hazard detection: observation far from what the planner believed
      const est = this.wind.at(ac.p[0], ac.p[1], ac.p[2], t, [0, 0, 0]);
      const resid = Math.hypot(ox - est[0], oy - est[1]);
      ac._gustCount = resid > 7.5 ? (ac._gustCount || 0) + 1 : 0;
      if (ac._gustCount === 2) F.onGustObserved(this, ac, [ox - est[0], oy - est[1]]);
    }
  }

  _sample(t) {
    const m = this.metrics, s = m.series;
    let air = 0, soc = 0, nGround = 0;
    for (const ac of this.aircraft) { if (ac.phase === PHASE.FLYING) air++; soc += ac.soc; nGround++; }
    let e1 = 0, e2 = 0;
    const a = [0, 0, 0], b = [0, 0, 0], c = [0, 0, 0];
    for (const [x, y, z] of this._windProbe) {
      this.truth.mean(x, y, z, t, a);
      this.forecast.mean(x, y, z, t, b);
      this.wind.at(x, y, z, t, c);
      e1 += (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2;
      e2 += (a[0] - c[0]) ** 2 + (a[1] - c[1]) ** 2;
    }
    const n = this._windProbe.length;
    s.t.push(t); s.airborne.push(air); s.soc.push(nGround ? soc / nGround : 0);
    s.windFc.push(Math.sqrt(e1 / n)); s.windMap.push(Math.sqrt(e2 / n));
    s.coverage.push(this.map.coverage(t)); s.los.push(m.los); s.completed.push(m.completed);
    if (s.t.length > 720) for (const k of Object.keys(s)) s[k].splice(0, s[k].length - 720);
  }

  // public helpers
  injectFault(type, opts = {}) { return F.injectFault(this, type, opts); }
  summary() { return this.metrics.summary(this); }
  airborneCount() { let n = 0; for (const a of this.aircraft) if (a.phase === PHASE.FLYING) n++; return n; }
}

export { AIRCRAFT_CLASSES, PHASE, MODE };
