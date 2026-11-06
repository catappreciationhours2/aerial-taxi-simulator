// Central configuration. Every number here is an ASSUMED research parameter, not a manufacturer
// specification (see README "Assumptions"). Units are SI unless a name says otherwise (kW, kWh, deg).

import { PITTSBURGH } from './geo.js';

export const G = 9.80665;
export const RHO = 1.2; // air density, kg/m^3

/** Reference vehicle classes. Hover power comes from actuator-disc theory (see aircraft/power.js). */
export const AIRCRAFT_CLASSES = {
  compact: { key: 'compact', label: 'Compact 3-seat', mass: 1400, discArea: 28, capacityKwh: 70, radius: 5.0, seats: 3, cdA: 1.1 },
  standard: { key: 'standard', label: 'Standard 4-seat', mass: 1800, discArea: 36, capacityKwh: 85, radius: 6.0, seats: 4, cdA: 1.5 },
  large: { key: 'large', label: 'Large 5-seat', mass: 2400, discArea: 46, capacityKwh: 110, radius: 7.5, seats: 5, cdA: 1.9 },
};

// here i assume some kind of helicoptor blade mechanism is helping with lift for the 
// purposes of the physics injection into my sim
export const AIRCRAFT_COMMON = {
  figureOfMerit: 0.72, // rotor efficiency in hover
  driveEfficiency: 0.92, // battery -> shaft
  profilePowerFrac: 0.09, // blade profile power as fraction of ideal hover power
  vCruise: 22, // planned cruise airspeed, m/s (~43 kt; low-level urban operation)
  vMax: 30, // max airspeed, m/s
  vClimb: 4.0, // m/s
  vDescend: 3.0, // m/s
  aHorizMax: 5.0, // horizontal accel limit, m/s^2
  aUpMax: 2.0,
  aDownMax: 2.5,
  thrustToWeightMax: 1.35, // ||T|| <= 1.35 m g
  yawRateMax: 60 * (Math.PI / 180),
  usableFraction: 0.9, // usable fraction of nameplate battery
  reserveFraction: 0.12, // fraction of usable energy held as emergency reserve
  chargeKw: 150,
  gnssSigma: 2.5, // m, 1-sigma per axis
  heartbeatTimeout: 3.0, // s
};

export const WORLD = {
  lat0: PITTSBURGH.lat,
  lon0: PITTSBURGH.lon,
  nx: 72,
  ny: 48,
  cell: 100, // metres
  levelsMsl: [320, 380, 440, 500], // flight levels, metres above mean sea level
  clearance: 35, // m, minimum clearance over terrain+buildings
  baseGroundMsl: 222, // river level
};

export const PLANNER = {
  weights: { time: 1.0, energy: 8.0, risk: 0.6, noise: 1.2 }, // J = wt*T + we*E(kWh) + wr*R + wn*N   (seconds-equivalent)
  padBase: 1.5, // s time padding around each reservation
  padGrow: 0.012, // extra padding per second of look-ahead
  padMax: 9,
  commitLead: 4, // s of plan that is frozen when replanning
  refSmoothS: 7, // s box-filter window that turns the lattice path into an acceleration-bounded reference (see Trajectory.ref)
  maxExpansions: 40000,
  speedCandidates: [0.7, 0.85, 1.0], // multiples of vCruise considered per edge
  gustSigmaAlong: 1.0, // headwind margin (in sigmas)
  gustSigmaCross: 2.0,
  minGroundSpeed: 4,
  windSafety: 0.92, // fraction of vMax usable for wind compensation
  lnsRounds: 3,
};

export const SAFETY = {
  dLoss: 45, // m: 3-D distance counted as loss of separation
  dCollision: 18, // m: 3-D distance counted as collision
  d0: 32, // m base protected radius
  latency: 1.5, // s, control + comm latency
  deltaRisk: 1e-3, // chance constraint P(loss of separation) <= delta
  dStatic: 12, // m: extra clearance to terrain/buildings enforced by the CBF (planner clears 35 m)
  aBrake: 2.4, // m/s^2 braking authority assumed in barrier functions
  cbfGamma: 0.9,
  cbfRange: 450,
  mpcDt: 0.5,
  mpcHorizon: 10,
  trackingDeviationLimit: 32, // m
  trackingDeviationTime: 3, // s
  nisThreshold: 16, // chi-square-ish threshold on normalised innovation (3 dof ~ p<0.001)
};

export const SIM = {
  dt: 0.1, // physics step
  mpcEvery: 2,
  monitorEvery: 10, // steps (1 s)
  coordEvery: 10,
  windMapEvery: 20,
  chargeThreshold: 0.55, // fraction of usable energy required to accept a mission
};

export const FEATURE_DEFAULTS = {
  reservations: true, // multi-agent space-time reservations (off => independent A*)
  windAware: true, // planner uses forecast/estimated wind
  physicsAware: true, // energy-aware cost + wind-optimal airspeed selection
  fleetWindMap: true, // collaborative wind estimation feeds the planner
  adaptiveEnvelope: true, // uncertainty-aware padding / inflated reservations
  mpc: true, // robust MPC tracking (off => PD tracking)
  cbfFilter: true, // control-barrier-function safety filter
  viability: true, // emergency-reachability constraint on dispatch + runtime monitor
  faultResponse: true, // detect + respond to faults (off => faults go unhandled)
  lnsRepair: true, // localized cluster repair on conflicts
  noiseAware: true, // noise-sensitive routing
  turbulence: true, // gusts/turbulence in the true wind
  externalTraffic: true, // background (non-fleet) aircraft
  stadiumTfr: false, // temporary flight restriction around the North Shore stadium
};

export const FEATURE_LABELS = {
  reservations: 'Multi-agent reservations (SIPP)',
  windAware: 'Wind-aware planning',
  physicsAware: 'Energy-aware / wind-optimal speed',
  fleetWindMap: 'Fleet wind map (Kalman)',
  adaptiveEnvelope: 'Adaptive safety envelopes',
  mpc: 'Robust MPC tracking',
  cbfFilter: 'CBF safety filter',
  viability: 'Emergency reachability',
  faultResponse: 'Fault detection + response',
  lnsRepair: 'Cluster repair (LNS)',
  noiseAware: 'Noise-aware routing',
  turbulence: 'Gusts + turbulence',
  externalTraffic: 'External air traffic',
  stadiumTfr: 'Stadium TFR (temporary)',
};

const off = (o) => ({ ...Object.fromEntries(Object.keys(FEATURE_DEFAULTS).map((k) => [k, false])), turbulence: true, externalTraffic: true, ...o });

/** Experiment presets. Baselines A-D follow the project plan. */
export const PRESETS = {
  A: { label: 'A: Independent A*', features: off({}) },
  B: { label: 'B: Space-time A*', features: off({ reservations: true }) },
  C: { label: 'C: Physics-aware', features: off({ reservations: true, windAware: true, physicsAware: true, mpc: true, faultResponse: true }) },
  D: { label: 'D: Full system', features: { ...FEATURE_DEFAULTS } },
};

export function defaultConfig(over = {}) {
  const base = {
    seed: 7,
    fleetSize: 60,
    demand: 1.0, // multiplier on ride request rate
    features: { ...FEATURE_DEFAULTS },
    wind: { speed10: 3.5, dirFrom: 250, gust: 1.0, forecastBias: 1.0 },
  };
  return {
    ...base,
    ...over,
    features: { ...base.features, ...(over.features || {}) },
    wind: { ...base.wind, ...(over.wind || {}) },
  };
}
