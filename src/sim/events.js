// Event log + metrics collection.

export class EventLog {
  constructor(max = 600) { this.max = max; this.items = []; this.listeners = []; this.n = 0; }
  add(t, level, text, meta = {}) {
    const e = { id: ++this.n, t, level, text, ...meta };
    this.items.push(e);
    if (this.items.length > this.max) this.items.splice(0, this.items.length - this.max);
    for (const l of this.listeners) l(e);
    return e;
  }
  on(fn) { this.listeners.push(fn); }
  recent(n = 50, levels = null) {
    const out = [];
    for (let i = this.items.length - 1; i >= 0 && out.length < n; i--) if (!levels || levels.includes(this.items[i].level)) out.push(this.items[i]);
    return out.reverse();
  }
}

const mean = (a) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
const pct = (a, p) => { if (!a.length) return 0; const s = [...a].sort((x, y) => x - y); return s[Math.min(s.length - 1, Math.floor(p * s.length))]; };

export class Metrics {
  constructor() {
    this.requested = 0; this.completed = 0; this.rejected = {}; this.los = 0; this.collisions = 0; this.crashes = 0;
    this.forcedLandings = 0; this.emergencyLandings = 0; this.diversions = 0; this.groundImpacts = 0;
    this.minSep = Infinity; this.trips = []; this.faultLatencies = []; this.recoveryTimes = [];
    this.peakAccel = []; this.jerkRms = [];
    this.series = { t: [], airborne: [], soc: [], windFc: [], windMap: [], coverage: [], los: [], completed: [] };
    this.cbfInterventions = 0; this.cbfInfeasible = 0; this.deviationReplans = 0; this.rtaSwitches = 0;
    this.lastLosPairs = new Set();
  }
  reject(reason) { this.rejected[reason] = (this.rejected[reason] || 0) + 1; }

  summary(engine = null) {
    const tripT = this.trips.map((x) => x.dur), delay = this.trips.map((x) => x.dur / Math.max(1, x.ideal)), en = this.trips.map((x) => x.energy);
    const out = {
      requested: this.requested, completed: this.completed,
      completionRate: this.requested ? this.completed / this.requested : 0,
      rejected: { ...this.rejected },
      lossOfSeparation: this.los, collisions: this.collisions, crashes: this.crashes,
      forcedLandings: this.forcedLandings, emergencyLandings: this.emergencyLandings, diversions: this.diversions,
      minSeparation: this.minSep === Infinity ? null : this.minSep,
      tripTimeMean: mean(tripT), tripTimeP95: pct(tripT, 0.95),
      delayRatioMean: mean(delay), delayRatioP95: pct(delay, 0.95), delayRatioWorst: delay.length ? Math.max(...delay) : 0,
      energyPerTripMean: mean(en),
      faultDetectLatencyMean: mean(this.faultLatencies), faultDetectLatencyMax: this.faultLatencies.length ? Math.max(...this.faultLatencies) : 0,
      recoveryTimeMean: mean(this.recoveryTimes),
      peakAccelP95: pct(this.peakAccel, 0.95), jerkRmsMean: mean(this.jerkRms),
      cbfInterventions: this.cbfInterventions, cbfInfeasible: this.cbfInfeasible,
      deviationReplans: this.deviationReplans, rtaSwitches: this.rtaSwitches,
    };
    if (engine) {
      const st = engine.coord.stats;
      out.plans = st.plans; out.planFailures = st.failures; out.replans = st.replans; out.deadlines = st.deadlines;
      out.planMsMean = mean(st.planMs); out.planMsP95 = pct(st.planMs, 0.95); out.expansions = st.expansions;
      out.lnsRuns = st.lnsRuns; out.viabilityRejects = st.viabilityRejects;
      out.reservations = engine.coord.table.count;
      out.simTime = engine.t;
    }
    return out;
  }
}
