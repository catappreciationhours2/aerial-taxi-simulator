// External (non-fleet) air traffic: helicopters and general-aviation aircraft crossing the city on
// straight tracks at flight levels. They are non-cooperative: the fleet sees them via an ADS-B-like
// feed (position/velocity with uncertainty) but cannot coordinate with them.
// NOTE: the real OpenSky feed (src/data/loaders.js) can be mapped into this same structure.

export class ExtAircraft {
  constructor(id, p, v, kind = 'helicopter') {
    this.id = id; this.p = p; this.v = v; this.kind = kind;
    this.radius = kind === 'helicopter' ? 7 : 6;
    this.sigma = 12; // ADS-B-like position uncertainty (m)
    this.coop = false; this.age = 0; this.tracked = false;
  }
  step(dt) { this.p[0] += this.v[0] * dt; this.p[1] += this.v[1] * dt; this.p[2] += this.v[2] * dt; this.age += dt; }
}

export class TrafficGenerator {
  constructor(world, rng, ratePerS = 1 / 70) {
    this.world = world; this.rng = rng; this.rate = ratePerS; this.n = 0;
    this.next = rng.exp(ratePerS);
  }
  /** Possibly spawn a crossing track; returns a new ExtAircraft or null. */
  tick(t) {
    if (t < this.next) return null;
    this.next = t + this.rng.exp(this.rate);
    const w = this.world, r = this.rng;
    const side = r.int(4), m = 150;
    let x, y;
    if (side === 0) { x = -m; y = r.range(0, w.height); } else if (side === 1) { x = w.width + m; y = r.range(0, w.height); }
    else if (side === 2) { x = r.range(0, w.width); y = -m; } else { x = r.range(0, w.width); y = w.height + m; }
    const tx = r.range(0.25, 0.75) * w.width, ty = r.range(0.25, 0.75) * w.height;
    const d = Math.hypot(tx - x, ty - y), spd = r.range(32, 58);
    const z = w.levels[r.chance(0.5) ? w.nz - 1 : w.nz - 2];
    const kind = r.chance(0.6) ? 'helicopter' : 'ga';
    return new ExtAircraft(`${kind === 'ga' ? 'GA' : 'HEL'}${++this.n}`, [x, y, z], [((tx - x) / d) * spd, ((ty - y) / d) * spd, 0], kind);
  }
  /** An intruder aimed at a target point, arriving there `lead` seconds from now (for fault injection). */
  intruderAt(target, lead, speed = 42, bearing = null) {
    const r = this.rng, ang = bearing ?? r.range(0, Math.PI * 2);
    const x = target[0] - Math.cos(ang) * speed * lead, y = target[1] - Math.sin(ang) * speed * lead;
    return new ExtAircraft(`INT${++this.n}`, [x, y, target[2]], [Math.cos(ang) * speed, Math.sin(ang) * speed, 0], 'helicopter');
  }
}
