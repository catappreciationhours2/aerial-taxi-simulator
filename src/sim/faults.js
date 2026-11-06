// Fault injection, detection and response.
//
// Each fault has: an INJECTION (changes the physical/environmental truth), a DETECTOR (what the vehicle or
// the fleet can actually observe), a RESPONSE (degraded mode, replanning, diversion, emergency policy) and
// a RESOLUTION (landing, link restored, hazard expired). Everything is recorded in `engine.faultRecords` so the
// UI and the benchmark can report detection latency, recovery time and outcome.
//
// With `features.faultResponse = false` the physical fault still happens but nothing reacts to it — useful to
// see what the safety architecture buys.

import { MODE, PHASE } from '../aircraft/aircraft.js';

export const FAULT_TYPES = {
  gps_bias: { label: 'GPS bias / spoof', scope: 'aircraft', desc: 'Adds a 60 m GNSS position error. Detected by innovation + INS-disagreement tests; GNSS isolated, envelope widened, aircraft diverts.' },
  comm_loss: { label: 'Comm loss (40 s)', scope: 'aircraft', desc: 'Drops the data link. Heartbeat timeout triggers lost-link logic: frozen reservations with extra margin, then autonomous landing if it lasts.' },
  propulsion: { label: 'Rotor failure', scope: 'aircraft', desc: 'Loses ~18% of thrust. Telemetry detects it; the aircraft flies the precomputed emergency policy to the nearest landing site, others replan around it.' },
  battery: { label: 'Battery fault', scope: 'aircraft', desc: 'Cell failure: 30% capacity lost and 2.5x drain. Reachability monitor diverts before reserve is violated.' },
  gust: { label: 'Sudden gust cell', scope: 'aircraft', desc: 'A 16 m/s crosswind cell appears ahead. Wind residuals reveal it; a hazard zone is broadcast and nearby routes are replanned.' },
  route_deviation: { label: 'Route deviation', scope: 'aircraft', desc: 'A lateral disturbance pushes the aircraft off its path. Tracking error invalidates its reservations and triggers replanning.' },
  sensor_failure: { label: 'Sensor degradation', scope: 'aircraft', desc: 'Inertial/air-data noise x4. Consistency checks flag it; larger envelopes and a diversion follow.' },
  traffic_intruder: { label: 'Intruder aircraft', scope: 'aircraft', desc: 'Non-cooperative aircraft on a collision course. Track registration reserves its path; the CBF filter protects as a last resort.' },
  vertiport_closed: { label: 'Close vertiport', scope: 'vertiport', desc: 'Closes a vertiport. Inbound aircraft are reassigned; departures there are blocked.' },
  planner_timeout: { label: 'Planner timeout (45 s)', scope: 'fleet', desc: 'The planner cannot return in time. Aircraft keep their last validated plan or fall back to the emergency policy (no search needed).' },
  stale_weather: { label: 'Stale weather feed', scope: 'fleet', desc: 'Forecast age exceeds limits. Uncertainty grows, margins widen, departures pause until refreshed.' },
  storm: { label: 'Storm front', scope: 'fleet', desc: 'Winds ramp to 2.3x. The forecast under-predicts; the fleet wind map learns it from aircraft observations.' },
};

function record(engine, type, ac, extra = {}) {
  const rec = {
    id: engine.faultRecords.length + 1, type, label: FAULT_TYPES[type].label, acId: ac ? ac.id : null, acName: ac ? ac.name : 'fleet',
    tInject: engine.t, tDetect: null, tRespond: null, tResolve: null, status: 'active', outcome: '', notes: [], ...extra,
  };
  engine.faultRecords.push(rec);
  if (ac) { ac.faults = ac.faults || []; ac.faults.push(rec); }
  return rec;
}

function activeRec(ac, type) {
  if (!ac || !ac.faults) return null;
  for (let i = ac.faults.length - 1; i >= 0; i--) { const r = ac.faults[i]; if (r.type === type && (r.status === 'active' || r.status === 'detected' || r.status === 'responding')) return r; }
  return null;
}

export function markDetected(engine, rec, note = '') {
  if (!rec || rec.tDetect !== null) return;
  if (!engine.features.faultResponse) return; // detection is part of the "Fault detection + response" feature: with it off, nothing notices
  rec.tDetect = engine.t; rec.status = 'detected';
  engine.metrics.faultLatencies.push(rec.tDetect - rec.tInject);
  if (note) rec.notes.push(note);
}
function markResponded(engine, rec, note = '') {
  if (!rec) return;
  if (rec.tRespond === null) rec.tRespond = engine.t;
  if (!TERMINAL.has(rec.status)) rec.status = 'responding';
  if (note) rec.notes.push(note);
}
const TERMINAL = new Set(['resolved', 'failed', 'unhandled']);
function markResolved(engine, rec, outcome, failed = false) {
  if (!rec || TERMINAL.has(rec.status)) return;
  if (!failed && !engine.features.faultResponse) { // the fault ran its course (expired / taxi landed) but nothing detected or handled it: do not call that "resolved"
    rec.tResolve = engine.t; rec.status = 'unhandled'; rec.outcome = 'never handled: ' + (outcome || 'ended on its own'); return;
  }
  rec.tResolve = engine.t; rec.status = failed ? 'failed' : 'resolved'; rec.outcome = outcome;
  if (!failed) engine.metrics.recoveryTimes.push(rec.tResolve - rec.tInject);
}

export function resolveFaults(engine, ac, how) {
  if (!ac.faults) return;
  const failed = how === 'crash';
  for (const r of ac.faults) if (!TERMINAL.has(r.status)) markResolved(engine, r, how, failed);
}

function pickTarget(engine, opts, { needPlan = true } = {}) {
  const ok = (a) => a && a.phase === PHASE.FLYING && !a.emergencyMode && (!needPlan || a.plan);
  if (opts.aircraftId !== undefined && opts.aircraftId !== null) { const a = engine.aircraft[opts.aircraftId]; if (ok(a)) return a; }
  if (engine.selectedId !== null && ok(engine.aircraft[engine.selectedId])) return engine.aircraft[engine.selectedId];
  const c = engine.aircraft.filter(ok);
  if (!c.length) return null;
  return c[engine.rng.int(c.length)];
}

export function injectFault(engine, type, opts = {}) {
  const t = engine.t, rng = engine.rng, f = engine.features;
  if (!FAULT_TYPES[type]) return { ok: false, reason: 'unknown fault type' };
  const noAc = { ok: false, reason: 'no airborne aircraft to target — wait for traffic or raise demand' };
  switch (type) {
    case 'gps_bias': {
      const ac = pickTarget(engine, opts); if (!ac) return noAc;
      const mag = opts.magnitude ?? 60, ang = rng.range(0, Math.PI * 2);
      ac.nav.setBias([Math.cos(ang) * mag, Math.sin(ang) * mag, 0], opts.rampS ?? 0, t);
      ac.health.gps = 'bias';
      const rec = record(engine, type, ac);
      engine.log('fault', `FAULT: GNSS bias of ${mag} m injected on ${ac.name}`, ac);
      return { ok: true, rec, ac };
    }
    case 'comm_loss': {
      const ac = pickTarget(engine, opts); if (!ac) return noAc;
      ac.health.commLoss = true; ac.lastHeartbeat = t - 0.0;
      const rec = record(engine, type, ac, { until: t + (opts.duration ?? 40) });
      engine.log('fault', `FAULT: data link to ${ac.name} lost`, ac);
      return { ok: true, rec, ac };
    }
    case 'propulsion': {
      const ac = pickTarget(engine, opts); if (!ac) return noAc;
      ac.health.thrustFactor = opts.factor ?? 0.82;
      const rec = record(engine, type, ac, { detectAt: t + 0.8 });
      engine.log('fault', `FAULT: rotor failure on ${ac.name} — thrust ${(ac.health.thrustFactor * 100).toFixed(0)}%`, ac);
      return { ok: true, rec, ac };
    }
    case 'battery': {
      const ac = pickTarget(engine, opts); if (!ac) return noAc;
      ac.health.drainMult = opts.drain ?? 2.5; ac.energy *= opts.capacityKeep ?? 0.7;
      const rec = record(engine, type, ac, { detectAt: t + 3 });
      engine.log('fault', `FAULT: battery fault on ${ac.name} — capacity −30%, drain ×${ac.health.drainMult}`, ac);
      return { ok: true, rec, ac };
    }
    case 'gust': {
      const ac = pickTarget(engine, opts); if (!ac) return noAc;
      const sp = Math.hypot(ac.v[0], ac.v[1]) || 1, hx = ac.v[0] / sp, hy = ac.v[1] / sp;
      const mag = opts.speed ?? 16;
      const cx = ac.p[0] + hx * 250, cy = ac.p[1] + hy * 250;
      engine.truth.addDisturbance({ x: cx, y: cy, z: ac.p[2], r: 380, vec: [-hy * mag, hx * mag, 0], t0: t, t1: t + 70 });
      const rec = record(engine, type, ac, { cx, cy });
      engine.log('fault', `FAULT: ${mag} m/s gust cell appeared ahead of ${ac.name}`, ac);
      return { ok: true, rec, ac };
    }
    case 'route_deviation': {
      const ac = pickTarget(engine, opts); if (!ac) return noAc;
      const sp = Math.hypot(ac.v[0], ac.v[1]) || 1, nx = -ac.v[1] / sp, ny = ac.v[0] / sp, s = rng.chance(0.5) ? 1 : -1;
      ac.extraAccel = { a: [nx * s * 9, ny * s * 9, 0], until: t + 5 };
      const rec = record(engine, type, ac);
      engine.log('fault', `FAULT: lateral disturbance pushing ${ac.name} off route`, ac);
      return { ok: true, rec, ac };
    }
    case 'sensor_failure': {
      const ac = pickTarget(engine, opts); if (!ac) return noAc;
      ac.nav.noiseMult = 4; ac.health.sensor = 'degraded';
      const rec = record(engine, type, ac, { detectAt: t + 2 });
      engine.log('fault', `FAULT: inertial/air-data degradation on ${ac.name}`, ac);
      return { ok: true, rec, ac };
    }
    case 'traffic_intruder': {
      const ac = pickTarget(engine, opts); if (!ac) return noAc;
      f.externalTraffic || engine.setFeature('externalTraffic', true);
      const lead = 28;
      const target = [ac.p[0] + ac.v[0] * lead, ac.p[1] + ac.v[1] * lead, ac.p[2]];
      const sp = Math.hypot(ac.v[0], ac.v[1]);
      const bearing = sp > 1 ? Math.atan2(ac.v[1], ac.v[0]) + Math.PI + rng.range(-0.9, 0.9) : null;
      const ext = engine.traffic.intruderAt(target, lead, 40, bearing);
      engine.ext.push(ext);
      const rec = record(engine, type, ac, { extId: ext.id, minDist: Infinity });
      engine.log('fault', `FAULT: non-cooperative intruder ${ext.id} on a collision course with ${ac.name}`, ac);
      return { ok: true, rec, ac };
    }
    case 'vertiport_closed': {
      let id = opts.vertiportId;
      if (!id) {
        const inbound = engine.aircraft.filter((a) => a.airborne && a.destId && engine.vertiports.has(a.destId));
        id = inbound.length ? inbound[rng.int(inbound.length)].destId : engine.world.vertiports[rng.int(engine.world.vertiports.length)].id;
      }
      const vp = engine.vertiports.get(id); if (!vp || vp.closed) return { ok: false, reason: 'vertiport already closed' };
      const rec = record(engine, type, null, { vertiportId: id, acName: vp.name, until: t + (opts.duration ?? 150) });
      engine.log('fault', `FAULT: ${vp.name} closed`, null);
      markDetected(engine, rec, 'status feed');
      if (f.faultResponse) {
        engine.setVertiportClosed(id, true);
        markResponded(engine, rec, 'inbound traffic reassigned');
      } else { vp.closed = false; rec.notes.push('no response enabled — aircraft keep flying to the closed vertiport'); }
      return { ok: true, rec };
    }
    case 'planner_timeout': {
      engine.coord.timeoutUntil = t + 45;
      const rec = record(engine, type, null, { until: t + 45 });
      engine.log('fault', 'FAULT: planner service is stalling (deadline overruns for 45 s)');
      const air = engine.aircraft.filter((a) => a.airborne && a.plan && !a.emergencyMode);
      for (let i = 0; i < Math.min(3, air.length); i++) engine.requestReplan(air[rng.int(air.length)], 'routine replan during outage');
      return { ok: true, rec };
    }
    case 'stale_weather': {
      engine.forecast.ageS = 2700; engine.forecast.stale = true;
      const rec = record(engine, type, null, { until: t + 90 });
      engine.log('fault', 'FAULT: weather feed stopped updating (forecast age 45 min)');
      return { ok: true, rec };
    }
    case 'storm': {
      engine.truth.setStorm(t + 5, opts.mult ?? 2.3, 90, 420);
      const rec = record(engine, type, null, { until: t + 5 + 420 });
      markDetected(engine, rec, 'wind observations');
      engine.log('fault', 'FAULT: storm front approaching — winds ramping up; forecast under-predicts');
      return { ok: true, rec };
    }
    default: return { ok: false, reason: 'unhandled' };
  }
}

// detections / responses
export function onGpsFault(engine, ac) {
  const f = engine.features, t = engine.t;
  const rec = activeRec(ac, 'gps_bias') || record(engine, 'gps_bias', ac, { natural: true });
  markDetected(engine, rec, `detector: ${ac.nav.fault.kind}`);
  if (!f.faultResponse) return;
  ac.health.gps = 'isolated'; ac.mode = MODE.DEGRADED; ac.padMult = 1.8;
  engine.log('warn', `${ac.name} GNSS fault detected (${ac.nav.fault.kind}) ${(t - rec.tInject).toFixed(1)} s after onset — GNSS isolated, dead-reckoning, safety envelope widened`, ac);
  engine.coord.rebook(ac, t);
  markResponded(engine, rec, 'GNSS isolated');
  if (!ac.emergencyMode && !ac.frozen) engine.divertAircraft(ac, 'GNSS fault');
}

export function onCommLoss(engine, ac) {
  const t = engine.t;
  const rec = activeRec(ac, 'comm_loss');
  markDetected(engine, rec, 'heartbeat timeout');
  ac.frozen = true; ac.mode = ac.mode === MODE.EMERGENCY ? ac.mode : MODE.DEGRADED; ac.padMult = Math.max(ac.padMult, 2);
  engine.log('warn', `${ac.name} heartbeat lost — lost-link procedure: holding last validated plan, reservations frozen with ×2 margin`, ac);
  engine.coord.rebook(ac, t);
  markResponded(engine, rec, 'frozen plan');
}

export function onLowBattery(engine, ac, kind) {
  const rec = activeRec(ac, 'battery');
  markDetected(engine, rec, 'reserve violation predicted');
  if (!engine.features.faultResponse) return;
  engine.log('warn', `${ac.name} cannot keep reserve (${ac.energy.toFixed(1)} kWh left) — ${kind === 'emergency' ? 'emergency landing' : 'diverting to nearest vertiport'}`, ac);
  markResponded(engine, rec, kind);
  if (ac.emergencyMode) return;
  if (kind === 'emergency') engine.enterEmergency(ac, 'insufficient energy'); else engine.divertAircraft(ac, 'low battery');
}

export function onGustObserved(engine, ac, resid) {
  const t = engine.t;
  let rec = null;
  for (const r of engine.faultRecords) if (r.type === 'gust' && r.tDetect === null && t - r.tInject < 120) rec = r;
  if (rec) markDetected(engine, rec, `${Math.hypot(resid[0], resid[1]).toFixed(1)} m/s residual`);
  if (!engine.features.faultResponse) return;
  engine.wind.addHazard({ x: ac.p[0], y: ac.p[1], z: ac.p[2], r: 420, vec: [resid[0], resid[1], 0], t0: t, t1: t + 90 });
  engine.log('warn', `${ac.name} measured ${Math.hypot(resid[0], resid[1]).toFixed(1)} m/s wind residual — hazard zone broadcast to fleet`, ac);
  if (rec) markResponded(engine, rec, 'hazard broadcast');
  let n = 0;
  for (const o of engine.aircraft) {
    if (!o.airborne || o.emergencyMode || !o.plan || o === ac) continue;
    if (Math.hypot(o.p[0] - ac.p[0], o.p[1] - ac.p[1]) < 1100) { engine.requestReplan(o, 'gust hazard'); n++; }
  }
  if (n) engine.log('info', `${n} nearby aircraft queued for hazard-aware replanning`);
}

export function onPlannerTimeout(engine, ac, reason) {
  const t = engine.t;
  let rec = null;
  for (const r of engine.faultRecords) if (r.type === 'planner_timeout' && !TERMINAL.has(r.status)) rec = r;
  if (rec) markDetected(engine, rec, 'deadline monitor');
  engine.log('warn', `Planner deadline overrun while replanning ${ac.name} (${reason})`, ac);
  if (!engine.features.faultResponse) return;
  const conflicts = [...engine.coord.table.conflictsOf(ac.id, t)].filter((x) => typeof x === 'number' || typeof x === 'string');
  if (!conflicts.length) {
    engine.log('info', `${ac.name} keeps its last validated plan (no conflicts) — fallback while planner recovers`, ac);
    if (rec) markResponded(engine, rec, 'kept validated plan');
  } else {
    engine.log('alert', `${ac.name}'s plan is no longer conflict-free and no new plan arrived — contingency policy`, ac);
    engine.enterEmergency(ac, 'planner deadline + invalid plan');
    if (rec) markResponded(engine, rec, 'contingency policy');
  }
}

// 1 Hz fault logic
export function faultMonitor(engine, t) {
  const f = engine.features;
  for (const rec of engine.faultRecords) {
    if (TERMINAL.has(rec.status)) continue;
    const ac = rec.acId !== null ? engine.aircraft[rec.acId] : null;
    switch (rec.type) {
      case 'comm_loss':
        if (ac && ac.health.commLoss && ac.faults && rec.tDetect !== null && !ac.emergencyMode && t - rec.tInject > 25 && f.faultResponse) {
          engine.log('alert', `${ac.name} lost-link timeout — autonomous landing at nearest site`, ac);
          engine.enterEmergency(ac, 'lost-link timeout');
          markResponded(engine, rec, 'autonomous landing');
        }
        if (t >= rec.until && ac) {
          ac.health.commLoss = false; ac.lastHeartbeat = t;
          if (ac.frozen) { ac.frozen = false; ac.padMult = 1; ac.flown = ac.plan; engine.coord.rebook(ac, t); }
          engine.log('ok', `${ac.name} data link restored — plan resynchronised`, ac);
          if (ac.airborne && !ac.emergencyMode) { markResolved(engine, rec, 'link restored'); if (ac.mode === MODE.DEGRADED) ac.mode = MODE.NOMINAL; }
          else rec.until = Infinity;
        }
        break;
      case 'propulsion':
        if (rec.tDetect === null && t >= rec.detectAt && ac && ac.airborne) {
          markDetected(engine, rec, 'motor telemetry');
          if (f.faultResponse) {
            engine.log('warn', `${ac.name} motor telemetry fault detected — switching to failure-specific contingency`, ac);
            engine.enterEmergency(ac, 'propulsion fault');
            markResponded(engine, rec, 'emergency policy');
          }
        }
        break;
      case 'battery':
        if (rec.tDetect === null && t >= rec.detectAt && ac && ac.airborne && f.faultResponse && !ac.emergencyMode) onLowBattery(engine, ac, 'divert');
        break;
      case 'sensor_failure':
        if (rec.tDetect === null && t >= rec.detectAt && ac && ac.airborne) {
          markDetected(engine, rec, 'consistency check');
          if (f.faultResponse) {
            ac.mode = MODE.DEGRADED; ac.padMult = Math.max(ac.padMult, 1.5);
            engine.log('warn', `${ac.name} sensor consistency check failed — degraded mode, wider margins, diverting`, ac);
            engine.coord.rebook(ac, t);
            markResponded(engine, rec, 'degraded mode');
            if (!ac.emergencyMode && !ac.frozen) engine.divertAircraft(ac, 'sensor degradation');
          }
        }
        break;
      case 'route_deviation':
        if (!ac || !ac.airborne) { markResolved(engine, rec, ac && ac.phase === PHASE.CRASHED ? 'crash' : 'landed', ac && ac.phase === PHASE.CRASHED); break; }
        rec.peakErr = Math.max(rec.peakErr || 0, ac.trackErr);
        if (rec.tDetect === null && (ac.trackErr > 18 || ac.devTime > 0)) { markDetected(engine, rec, `tracking error ${ac.trackErr.toFixed(0)} m`); if (f.faultResponse) markResponded(engine, rec, 'controller + replan'); }
        if (!ac.extraAccel && ac.trackErr < 12 && t - rec.tInject > 8) markResolved(engine, rec, rec.tDetect !== null ? `back on plan (peak ${rec.peakErr.toFixed(0)} m)` : `absorbed by the controller (peak ${rec.peakErr.toFixed(0)} m)`);
        break;
      case 'gust':
        if (t - rec.tInject > 75) markResolved(engine, rec, rec.tDetect !== null ? 'cell dissipated; hazard handled' : 'cell dissipated undetected', rec.tDetect === null && f.faultResponse);
        break;
      case 'traffic_intruder': {
        const ext = engine.ext.find((e) => e.id === rec.extId);
        if (rec.tDetect === null && engine.coord.table.byId.has(`EXT:${rec.extId}`)) { markDetected(engine, rec, 'ADS-B track'); markResponded(engine, rec, 'track reserved'); }
        if (ext && ac && ac.airborne) rec.minDist = Math.min(rec.minDist, Math.hypot(ac.p[0] - ext.p[0], ac.p[1] - ext.p[1], ac.p[2] - ext.p[2]));
        if (ext && ac && t - rec.tInject > 40) markResolved(engine, rec, `closest approach ${Number.isFinite(rec.minDist) ? rec.minDist.toFixed(0) : '?'} m`);
        if (!ext && t - rec.tInject > 40) markResolved(engine, rec, `closest approach ${Number.isFinite(rec.minDist) ? rec.minDist.toFixed(0) : '?'} m`);
        break;
      }
      case 'planner_timeout':
        if (t >= rec.until) { markResolved(engine, rec, 'planner recovered'); engine.log('ok', 'Planner service recovered'); }
        break;
      case 'stale_weather':
        if (rec.tDetect === null && engine.forecast.ageS > 1800) {
          markDetected(engine, rec, 'timestamp check');
          if (f.faultResponse) {
            engine.weatherStale = true; engine.wind.bump();
            for (const a of engine.aircraft) if (a.airborne) { a.padMult = Math.max(a.padMult, 1.4); engine.coord.rebook(a, t); }
            engine.log('warn', 'Weather feed stale: uncertainty inflated, margins ×1.4, new departures suspended');
            markResponded(engine, rec, 'margins widened, departures paused');
          }
        }
        if (t >= rec.until) {
          engine.forecast.ageS = 0; engine.forecast.stale = false; engine.weatherStale = false; engine.wind.bump();
          for (const a of engine.aircraft) if (a.airborne && a.padMult === 1.4) { a.padMult = 1; engine.coord.rebook(a, t); }
          markResolved(engine, rec, 'feed restored'); engine.log('ok', 'Weather feed restored — departures resume');
        }
        break;
      case 'storm':
        if (t >= rec.until) { markResolved(engine, rec, 'front passed'); engine.log('ok', 'Storm front has passed'); }
        break;
      case 'vertiport_closed':
        if (f.faultResponse && t >= rec.until && engine.vertiports.get(rec.vertiportId).closed) { engine.setVertiportClosed(rec.vertiportId, false); engine.log('ok', `${rec.acName} reopened`); }
        if (!engine.vertiports.get(rec.vertiportId).closed && t > rec.tInject + 1) markResolved(engine, rec, 'reopened');
        if (!f.faultResponse && t - rec.tInject > 2) markResolved(engine, rec, 'unhandled (no response enabled)', false);
        break;
      default: break;
    }
  }
}

export function reopenAll(engine) {
  for (const [id, vp] of engine.vertiports) if (vp.closed) engine.setVertiportClosed(id, false);
}
