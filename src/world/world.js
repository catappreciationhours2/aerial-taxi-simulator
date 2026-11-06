// Pittsburgh-inspired 3-D urban world on a regular grid of nodes (ix, iy, iz).
//
// The default map is a *stylised synthetic* city: three rivers meeting at "the Point", river valleys with
// hills (terrain), a dense downtown triangle, neighbourhood clusters, vertiports, parks used as emergency
// landing sites, permanent restricted airspace and an optional temporary stadium TFR. It is NOT real
// survey data. `applyBuildingsGeoJSON` lets you drop in real OpenStreetMap footprints (see scripts/fetch-osm.js).
//
// Node coordinates: x = east (m), y = north (m), z = altitude MSL (m), flight levels at fixed MSL heights.

import { WORLD } from '../core/config.js';
import { RNG, clamp, smoothstep, makeFbm, MinHeap } from '../core/util.js';
import { ENUFrame, GEOID_UNDULATION_PIT } from '../core/geo.js';

const RIVERS = [
  { name: 'Allegheny', hw: 140, pts: [[7200, 4300], [6200, 3900], [5200, 3500], [4300, 3050], [3500, 2650], [3150, 2450]] },
  { name: 'Monongahela', hw: 130, pts: [[5800, 0], [5000, 500], [4300, 1100], [3700, 1700], [3300, 2150], [3150, 2450]] },
  { name: 'Ohio', hw: 210, pts: [[3150, 2450], [2500, 2900], [1600, 3300], [800, 3500], [0, 3900]] },
];

const VERTIPORT_DEFS = [
  { id: 'DTN', name: 'Downtown Roof', x: 3700, y: 2850, roof: 115, pads: 4, chargers: 6 },
  { id: 'NSH', name: 'North Shore', x: 3300, y: 3350, roof: 0, pads: 3, chargers: 4 },
  { id: 'SSD', name: 'South Side', x: 3950, y: 1950, roof: 0, pads: 3, chargers: 4 },
  { id: 'OAK', name: 'Oakland Med', x: 5500, y: 2300, roof: 40, pads: 4, chargers: 5 },
  { id: 'STR', name: 'Strip District', x: 4300, y: 3250, roof: 0, pads: 2, chargers: 3 },
  { id: 'ELB', name: 'East Liberty', x: 6300, y: 3150, roof: 0, pads: 3, chargers: 4 },
  { id: 'SHD', name: 'Shadyside', x: 5000, y: 2800, roof: 0, pads: 2, chargers: 3 },
  { id: 'AWT', name: 'Airport Link', x: 1000, y: 2700, roof: 0, pads: 3, chargers: 4 },
  { id: 'MTW', name: 'Mt Washington', x: 3300, y: 1700, roof: 0, pads: 2, chargers: 3 },
];

const EMERGENCY_DEFS = [
  { id: 'SCH', name: 'Schenley Park', x: 5700, y: 1700 },
  { id: 'HLP', name: 'Highland Park', x: 6600, y: 3500 },
  { id: 'FRK', name: 'Frick Park', x: 6300, y: 1100 },
  { id: 'RVP', name: 'Riverview Park', x: 2800, y: 4200 },
  { id: 'WEP', name: 'West End Park', x: 2300, y: 2200 },
  { id: 'ACM', name: 'Allegheny Commons', x: 3000, y: 3700 },
  { id: 'MLP', name: 'Mellon Park', x: 5300, y: 2950 },
  { id: 'SHP', name: 'Southside Park', x: 4600, y: 1500 },
  { id: 'BRK', name: 'Brookline Fields', x: 2400, y: 800 },
];

function distToPolyline(x, y, pts) {
  let best = Infinity, tx = 1, ty = 0;
  for (let i = 0; i < pts.length - 1; i++) {
    const [ax, ay] = pts[i], [bx, by] = pts[i + 1];
    const dx = bx - ax, dy = by - ay;
    const L2 = dx * dx + dy * dy;
    const t = clamp(((x - ax) * dx + (y - ay) * dy) / L2, 0, 1);
    const d = Math.hypot(x - (ax + t * dx), y - (ay + t * dy));
    if (d < best) { best = d; const L = Math.sqrt(L2); tx = dx / L; ty = dy / L; }
  }
  return { d: best, tx, ty };
}

const gauss2 = (x, y, cx, cy, s) => Math.exp(-((x - cx) ** 2 + (y - cy) ** 2) / (2 * s * s));

export class World {
  constructor(opts = {}) {
    const c = { ...WORLD, ...opts };
    this.cfg = c;
    this.nx = c.nx; this.ny = c.ny; this.cell = c.cell;
    this.levels = c.levelsMsl.slice();
    this.nz = this.levels.length;
    this.width = this.nx * this.cell; this.height = this.ny * this.cell;
    this.clearance = c.clearance;
    this.layer = this.nx * this.ny;
    this.nNodes = this.layer * this.nz;
    this.frame = new ENUFrame(c.lat0, c.lon0, c.baseGroundMsl + GEOID_UNDULATION_PIT);
    this.rivers = RIVERS;
    this.seed = opts.worldSeed ?? 412;
    const rng = new RNG(this.seed);
    this._fbmA = makeFbm(this.seed + 1, 4);
    this._fbmB = makeFbm(this.seed + 2, 3);
    this.ground = new Float32Array(this.layer); // max terrain MSL within a cell
    this.terrainGrid = new Float32Array(this.layer); // terrain MSL at cell centres (for fast bilinear lookup)
    this.bldg = new Float32Array(this.layer); // building height AGL
    this.water = new Uint8Array(this.layer);
    this.flowX = new Float32Array(this.layer); // river flow direction (unit) weighted by proximity
    this.flowY = new Float32Array(this.layer);
    this.riverW = new Float32Array(this.layer); // channeling weight in [0,1]
    this.noiseSens = new Float32Array(this.layer);
    this.popDensity = new Float32Array(this.layer);
    this.restricted = [
      { id: 'APT', name: 'Airport approach corridor', x: 500, y: 4300, r: 950, perm: true, active: true, t0: 0, t1: Infinity },
      { id: 'TFR', name: 'Stadium TFR', x: 3000, y: 3200, r: 600, perm: false, active: false, t0: 0, t1: Infinity },
    ];
    this.version = 0;
    this._buildTerrain();
    this._buildBuildings(rng);
    this._placeSites(rng);
    this.finalize();
  }

  // terrain
  riverInfo(x, y) {
    let best = { dw: Infinity, tx: 1, ty: 0 };
    for (const r of this.rivers) {
      const q = distToPolyline(x, y, r.pts);
      const dw = q.d - r.hw;
      if (dw < best.dw) best = { dw, tx: q.tx, ty: q.ty };
    }
    return best;
  }
  isWaterAt(x, y) { return this.riverInfo(x, y).dw <= 0; }

  /** Fast bilinear terrain height from the cell-centre grid (use in hot loops instead of terrainAt). */
  terrainFast(x, y) {
    const fx = clamp(x / this.cell - 0.5, 0, this.nx - 1.001), fy = clamp(y / this.cell - 0.5, 0, this.ny - 1.001);
    const ix = Math.floor(fx), iy = Math.floor(fy), ax = fx - ix, ay = fy - iy, g = this.terrainGrid, nx = this.nx;
    const i = iy * nx + ix;
    return (g[i] * (1 - ax) + g[i + 1] * ax) * (1 - ay) + (g[i + nx] * (1 - ax) + g[i + nx + 1] * ax) * ay;
  }

  /** Analytic terrain height (MSL, metres) at a point. */
  terrainAt(x, y) {
    const r = this.riverInfo(x, y);
    if (r.dw <= 0) return this.cfg.baseGroundMsl - 4;
    const v = smoothstep(0, 1000, r.dw);
    const fb = 0.5 + 0.5 * this._fbmA(x / 1700, y / 1700);
    const tilt = 14 * this._fbmB(x / 4200 + 3, y / 4200);
    const flood = 10 * smoothstep(0, 350, r.dw);
    const hill = (24 + 110 * (0.3 + 0.7 * fb)) * v;
    return this.cfg.baseGroundMsl + flood + hill + Math.max(0, tilt) * v;
  }

  _buildTerrain() {
    const { nx, ny, cell } = this;
    for (let iy = 0; iy < ny; iy++) {
      for (let ix = 0; ix < nx; ix++) {
        const i = iy * nx + ix;
        let m = 0;
        for (let sy = 0; sy < 3; sy++) for (let sx = 0; sx < 3; sx++) {
          m = Math.max(m, this.terrainAt((ix + 0.15 + 0.35 * sx) * cell, (iy + 0.15 + 0.35 * sy) * cell));
        }
        this.ground[i] = m;
        const cx = (ix + 0.5) * cell, cy = (iy + 0.5) * cell;
        this.terrainGrid[i] = this.terrainAt(cx, cy);
        const r = this.riverInfo(cx, cy);
        this.water[i] = r.dw <= 0.35 * cell ? 1 : 0;
        const w = Math.exp(-Math.max(0, r.dw) / 350);
        this.riverW[i] = w;
        this.flowX[i] = r.tx; this.flowY[i] = r.ty;
      }
    }
  }

  // buildings + people
  _density(x, y) {
    return clamp(
      0.95 * gauss2(x, y, 3750, 2850, 520) + 0.55 * gauss2(x, y, 5500, 2300, 520) + 0.42 * gauss2(x, y, 3400, 3400, 420) +
      0.36 * gauss2(x, y, 3900, 1900, 460) + 0.3 * gauss2(x, y, 6300, 3200, 460) + 0.26 * gauss2(x, y, 5000, 2850, 420) +
      0.12 + 0.1 * this._fbmB(x / 900, y / 900), 0, 1);
  }

  _buildBuildings(rng) {
    const { nx, ny, cell } = this;
    for (let iy = 0; iy < ny; iy++) {
      for (let ix = 0; ix < nx; ix++) {
        const i = iy * nx + ix;
        const x = (ix + 0.5) * cell, y = (iy + 0.5) * cell;
        const d = this._density(x, y);
        this.popDensity[i] = d;
        if (this.water[i]) { this.bldg[i] = 0; continue; }
        const slope = Math.abs(this.ground[i] - this.terrainAt(x, y));
        const p = clamp(0.1 + d * 0.95, 0, 0.97);
        if (rng.float() < p && slope < 40) {
          let h = 8 + 14 * rng.float() + d * rng.float() * 110;
          if (d > 0.8 && rng.float() < 0.18) h = 130 + 120 * rng.float();
          this.bldg[i] = Math.min(270, h);
        }
      }
    }
  }

  // sites
  _snapFree(x, y, wantRoof = false) {
    // spiral search for a cell that is land, building-free (unless roof), and reasonably flat.
    for (let r = 0; r < 12; r++) {
      for (let dy = -r; dy <= r; dy++) for (let dx = -r; dx <= r; dx++) {
        if (Math.max(Math.abs(dx), Math.abs(dy)) !== r) continue;
        const ix = Math.floor(x / this.cell) + dx, iy = Math.floor(y / this.cell) + dy;
        if (ix < 1 || iy < 1 || ix >= this.nx - 1 || iy >= this.ny - 1) continue;
        const i = iy * this.nx + ix;
        if (this.water[i]) continue;
        if (!wantRoof && this.bldg[i] > 0) continue;
        if (this._nearRestricted((ix + 0.5) * this.cell, (iy + 0.5) * this.cell, this.cell * 2)) continue;
        return [ix, iy];
      }
    }
    return [Math.floor(x / this.cell), Math.floor(y / this.cell)];
  }

  _nearRestricted(x, y, pad) {
    for (const r of this.restricted) if (Math.hypot(x - r.x, y - r.y) < r.r + pad) return true;
    return false;
  }

  _placeSites() {
    this.vertiports = [];
    this.emergencySites = [];
    for (const d of VERTIPORT_DEFS) {
      const [ix, iy] = this._snapFree(d.x, d.y, d.roof > 0);
      const i = iy * this.nx + ix;
      this.bldg[i] = d.roof;
      const padElev = this.ground[i] + d.roof;
      this.vertiports.push({
        id: d.id, name: d.name, ix, iy, x: (ix + 0.5) * this.cell, y: (iy + 0.5) * this.cell,
        padElev, pads: d.pads, chargers: d.chargers, closed: false, kind: 'vertiport',
      });
    }
    for (const d of EMERGENCY_DEFS) {
      const [ix, iy] = this._snapFree(d.x, d.y, false);
      const i = iy * this.nx + ix;
      this.emergencySites.push({
        id: d.id, name: d.name, ix, iy, x: (ix + 0.5) * this.cell, y: (iy + 0.5) * this.cell,
        padElev: this.ground[i], kind: 'park',
      });
    }
  }

  /** Landing sites usable in an emergency: open vertiports + parks. */
  landingSites() {
    return [...this.vertiports.filter((v) => !v.closed), ...this.emergencySites];
  }

  // derived data
  /** (Re)compute static blockage, SDF and noise map. Call after changing buildings or restrictions. */
  finalize() {
    const { nx, ny, nz, cell } = this;
    this.version++;
    this.blocked = new Uint8Array(this.nNodes);
    for (let iz = 0; iz < nz; iz++) {
      const alt = this.levels[iz];
      for (let iy = 0; iy < ny; iy++) {
        for (let ix = 0; ix < nx; ix++) {
          const i = iy * nx + ix;
          const surface = this.ground[i] + this.bldg[i];
          let b = alt < surface + this.clearance;
          if (!b) {
            const x = (ix + 0.5) * cell, y = (iy + 0.5) * cell;
            for (const r of this.restricted) if (r.perm && r.active && Math.hypot(x - r.x, y - r.y) < r.r + cell * 0.5) { b = true; break; }
          }
          this.blocked[iz * this.layer + i] = b ? 1 : 0;
        }
      }
    }
    // noise sensitivity: residents + hospitals/schools, none on water
    for (let i = 0; i < this.layer; i++) {
      let s = 0.15 + 0.85 * clamp(this.popDensity[i] * 1.2, 0, 1);
      if (this.water[i]) s = 0.03;
      this.noiseSens[i] = s;
    }
    for (const hot of [[5500, 2300, 450], [5100, 2400, 300], [3950, 2900, 250]]) {
      for (let iy = 0; iy < ny; iy++) for (let ix = 0; ix < nx; ix++) {
        const d = Math.hypot((ix + 0.5) * cell - hot[0], (iy + 0.5) * cell - hot[1]);
        if (d < hot[2]) this.noiseSens[iy * nx + ix] = Math.min(1.5, this.noiseSens[iy * nx + ix] + 0.5);
      }
    }
    this._buildSdf();
  }

  _buildSdf() {
    // Multi-source Dijkstra over the 26-neighbourhood giving metric distance to the nearest blocked node centre.
    const { nx, ny, nz, cell } = this;
    const sdf = new Float32Array(this.nNodes).fill(1e9);
    const heap = new MinHeap();
    for (let n = 0; n < this.nNodes; n++) if (this.blocked[n]) { sdf[n] = 0; heap.push(0, n); }
    const dzs = [];
    for (let k = 0; k < nz - 1; k++) dzs.push(this.levels[k + 1] - this.levels[k]);
    const dz = dzs.length ? dzs.reduce((a, b) => a + b, 0) / dzs.length : 60;
    while (heap.size) {
      const d0 = heap.peekKey(); const n = heap.pop();
      if (d0 > sdf[n]) continue;
      const iz = Math.floor(n / this.layer), r = n % this.layer, iy = Math.floor(r / nx), ix = r % nx;
      for (let oz = -1; oz <= 1; oz++) for (let oy = -1; oy <= 1; oy++) for (let ox = -1; ox <= 1; ox++) {
        if (!ox && !oy && !oz) continue;
        const jx = ix + ox, jy = iy + oy, jz = iz + oz;
        if (jx < 0 || jy < 0 || jz < 0 || jx >= nx || jy >= ny || jz >= nz) continue;
        const m = jz * this.layer + jy * nx + jx;
        const nd = d0 + Math.hypot(ox * cell, oy * cell, oz * dz);
        if (nd < sdf[m]) { sdf[m] = nd; heap.push(nd, m); }
      }
    }
    this.sdf = sdf;
  }

  /** Signed-ish distance (m) from a point to the nearest blocked region, trilinear in the node lattice. */
  sdfAt(x, y, z) {
    const fx = clamp(x / this.cell - 0.5, 0, this.nx - 1.001), fy = clamp(y / this.cell - 0.5, 0, this.ny - 1.001);
    const fz = clamp(this._levelFrac(z), 0, this.nz - 1.001);
    const ix = Math.floor(fx), iy = Math.floor(fy), iz = Math.floor(fz);
    const ax = fx - ix, ay = fy - iy, az = fz - iz;
    let s = 0;
    for (let dz = 0; dz < 2; dz++) for (let dy = 0; dy < 2; dy++) for (let dx = 0; dx < 2; dx++) {
      const w = (dx ? ax : 1 - ax) * (dy ? ay : 1 - ay) * (dz ? az : 1 - az);
      s += w * this.sdf[(iz + dz) * this.layer + (iy + dy) * this.nx + ix + dx];
    }
    return s;
  }

  /** Gradient of the SDF (points away from obstacles). */
  sdfGrad(x, y, z, h = 40) {
    const gx = (this.sdfAt(x + h, y, z) - this.sdfAt(x - h, y, z)) / (2 * h);
    const gy = (this.sdfAt(x, y + h, z) - this.sdfAt(x, y - h, z)) / (2 * h);
    const gz = (this.sdfAt(x, y, z + h) - this.sdfAt(x, y, z - h)) / (2 * h);
    const n = Math.hypot(gx, gy, gz) || 1;
    return [gx / n, gy / n, gz / n];
  }

  _levelFrac(z) {
    const L = this.levels;
    if (z <= L[0]) return (z - L[0]) / (L[1] - L[0]);
    for (let k = 0; k < L.length - 1; k++) if (z <= L[k + 1]) return k + (z - L[k]) / (L[k + 1] - L[k]);
    return L.length - 1 + (z - L[L.length - 1]) / (L[L.length - 1] - L[L.length - 2]);
  }

  // indexing helpers
  idx(ix, iy, iz) { return iz * this.layer + iy * this.nx + ix; }
  decode(n) { const iz = Math.floor(n / this.layer), r = n % this.layer; return [r % this.nx, Math.floor(r / this.nx), iz]; }
  center(n) {
    const iz = Math.floor(n / this.layer), r = n % this.layer;
    return [((r % this.nx) + 0.5) * this.cell, (Math.floor(r / this.nx) + 0.5) * this.cell, this.levels[iz]];
  }
  cellIndex(x, y) {
    const ix = clamp(Math.floor(x / this.cell), 0, this.nx - 1), iy = clamp(Math.floor(y / this.cell), 0, this.ny - 1);
    return iy * this.nx + ix;
  }
  levelOf(z) {
    let best = 0, bd = Infinity;
    for (let k = 0; k < this.nz; k++) { const d = Math.abs(this.levels[k] - z); if (d < bd) { bd = d; best = k; } }
    return best;
  }
  nodeAt(x, y, z) {
    const ix = clamp(Math.floor(x / this.cell), 0, this.nx - 1), iy = clamp(Math.floor(y / this.cell), 0, this.ny - 1);
    return this.idx(ix, iy, this.levelOf(z));
  }
  /**
   * Nearest unblocked lattice node to a continuous position (searches the own cell's column first, then the 8
   * neighbours). A vehicle can legitimately be inside a cell whose nearest node is blocked (e.g. just beside a
   * building, or between levels), so field lookups must not use the raw nodeAt.
   * @returns {number} node index, or -1 if nothing free within one cell
   */
  nearestFreeNode(x, y, z) {
    const cx = clamp(Math.floor(x / this.cell), 0, this.nx - 1), cy = clamp(Math.floor(y / this.cell), 0, this.ny - 1);
    let best = -1, bd = Infinity;
    for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
      const ix = cx + dx, iy = cy + dy;
      if (ix < 0 || iy < 0 || ix >= this.nx || iy >= this.ny) continue;
      const px = (ix + 0.5) * this.cell - x, py = (iy + 0.5) * this.cell - y;
      for (let k = 0; k < this.nz; k++) {
        const n = this.idx(ix, iy, k);
        if (this.blocked[n]) continue;
        const pz = this.levels[k] - z;
        const d = px * px + py * py + pz * pz;
        if (d < bd) { bd = d; best = n; }
      }
    }
    return best;
  }
  inBounds(x, y) { return x >= 0 && y >= 0 && x < this.width && y < this.height; }

  surfaceAt(x, y) { const i = this.cellIndex(x, y); return this.ground[i] + this.bldg[i]; }
  /** Altitude above ground level (AGL) of an MSL altitude at (x, y), measured to terrain only. */
  aglAt(x, y, zMsl) { return zMsl - this.terrainFast(x, y); }
  isBlockedAt(x, y, z) { return this.blocked[this.nodeAt(x, y, z)] === 1; }

  /** Lowest flight level (index) usable directly above a vertiport/site cell, or -1. */
  firstFreeLevel(ix, iy) {
    for (let k = 0; k < this.nz; k++) if (!this.blocked[this.idx(ix, iy, k)]) return k;
    return -1;
  }
  vertiportById(id) { return this.vertiports.find((v) => v.id === id); }
  siteById(id) { return this.vertiports.find((v) => v.id === id) || this.emergencySites.find((v) => v.id === id); }

  // airspace
  restrictedAt(x, y, z, t = 0) {
    for (const r of this.restricted) {
      if (!r.active || t < r.t0 || t > r.t1) continue;
      if (Math.hypot(x - r.x, y - r.y) < r.r) return r;
    }
    return null;
  }
  /** Node indices (all levels) inside a restricted zone — used to turn a TFR into spatiotemporal reservations. */
  nodesInZone(zone) {
    const out = [];
    for (let iy = 0; iy < this.ny; iy++) for (let ix = 0; ix < this.nx; ix++) {
      if (Math.hypot((ix + 0.5) * this.cell - zone.x, (iy + 0.5) * this.cell - zone.y) < zone.r + this.cell * 0.5) {
        for (let iz = 0; iz < this.nz; iz++) out.push(this.idx(ix, iy, iz));
      }
    }
    return out;
  }
  zone(id) { return this.restricted.find((r) => r.id === id); }

  // real-data hook
  /**
   * Rasterise GeoJSON building footprints (OSM style) into the cell height map.
   * Heights come from properties.height or properties['building:levels'] * 3.2 (default 12 m).
   * Map centre (lat0, lon0) sits at the centre of the simulated rectangle.
   */
  applyBuildingsGeoJSON(geojson) {
    const feats = geojson.type === 'FeatureCollection' ? geojson.features : [geojson];
    const polys = [];
    for (const f of feats) {
      const g = f.geometry || f;
      if (!g) continue;
      const rings = g.type === 'Polygon' ? [g.coordinates] : g.type === 'MultiPolygon' ? g.coordinates : [];
      const p = f.properties || {};
      let h = parseFloat(p.height);
      if (!(h > 0)) { const lv = parseFloat(p['building:levels']); h = lv > 0 ? lv * 3.2 : 12; }
      for (const poly of rings) {
        const ring = poly[0].map(([lon, lat]) => {
          const e = this.frame.toENU(lat, lon, this.cfg.baseGroundMsl + GEOID_UNDULATION_PIT);
          return [e[0] + this.width / 2, e[1] + this.height / 2];
        });
        polys.push({ ring, h });
      }
    }
    this.bldg.fill(0);
    for (const { ring, h } of polys) {
      let minx = Infinity, maxx = -Infinity, miny = Infinity, maxy = -Infinity;
      for (const [x, y] of ring) { minx = Math.min(minx, x); maxx = Math.max(maxx, x); miny = Math.min(miny, y); maxy = Math.max(maxy, y); }
      const x0 = Math.max(0, Math.floor(minx / this.cell)), x1 = Math.min(this.nx - 1, Math.floor(maxx / this.cell));
      const y0 = Math.max(0, Math.floor(miny / this.cell)), y1 = Math.min(this.ny - 1, Math.floor(maxy / this.cell));
      for (let iy = y0; iy <= y1; iy++) for (let ix = x0; ix <= x1; ix++) {
        const cx = (ix + 0.5) * this.cell, cy = (iy + 0.5) * this.cell;
        if (pointInRing(cx, cy, ring) || (x0 === x1 && y0 === y1)) {
          const i = iy * this.nx + ix;
          if (!this.water[i]) this.bldg[i] = Math.max(this.bldg[i], h);
        }
      }
    }
    for (const v of this.vertiports) this.bldg[v.iy * this.nx + v.ix] = (VERTIPORT_DEFS.find((d) => d.id === v.id) || {}).roof || 0;
    for (const s of this.emergencySites) this.bldg[s.iy * this.nx + s.ix] = 0;
    this.finalize();
    return polys.length;
  }
}

export function pointInRing(x, y, ring) {
  let inside = false;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
    const [xi, yi] = ring[i], [xj, yj] = ring[j];
    if ((yi > y) !== (yj > y) && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}
