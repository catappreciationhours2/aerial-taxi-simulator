// Pixel-art renderer. Everything is drawn at a low logical resolution (360 x 240, 20 m per pixel) with 1x1 fillRect calls and
// scaled up by CSS (image-rendering: pixelated), so it works with any CanvasRenderingContext2D-like object (also a test stub).

export const S = 20; // metres per pixel
export const VIEWS = [
  { id: 'traffic', label: 'Traffic', icon: '✈', hint: 'Cars as aerial taxis with dotted planned routes' },
  { id: 'battery', label: 'Battery', icon: '🔋', hint: 'State of charge of every aircraft (green → pink = low)' },
  { id: 'wind', label: 'Wind', icon: '🌬', hint: 'True wind at 380 m MSL drawn as flowing currents; hazard zones in pink' },
  { id: 'airspace', label: 'Airspace', icon: '⛔', hint: 'Restricted zones, flight-level blockage, external traffic' },
  { id: 'safety', label: 'Safety', icon: '🛡', hint: 'Protected radii, close pairs, CBF interventions' },
  { id: 'noise', label: 'Noise', icon: '🔊', hint: 'Noise-sensitivity (population) map the planner avoids' },
  { id: 'windmap', label: 'Wind map', icon: '🧭', hint: 'Planner wind-estimate error (dark = accurate, pink = wrong)' },
];

const COL = {
  bg: [10, 15, 36], land: [14, 21, 48], hill: [26, 33, 70], water: [13, 36, 78], water2: [18, 46, 94], bldg: [28, 40, 84], bldgTop: [52, 68, 124],
  ink: '#e8ecff', dim: '#7c86b8', pink: '#ffb3c7', mint: '#b6f0d0', sky: '#a8d8ff', lav: '#d2b8ff', peach: '#ffd6a5', yellow: '#fff2a8', red: '#ff7a9c', amber: '#ffcf70',
};
const hex = (r, g, b) => '#' + [r, g, b].map((v) => Math.max(0, Math.min(255, v | 0)).toString(16).padStart(2, '0')).join('');

function hsl(h, s, l) {
  h = ((h % 360) + 360) % 360 / 360;
  const f = (n) => { const k = (n + h * 12) % 12; const a = s * Math.min(l, 1 - l); return l - a * Math.max(-1, Math.min(k - 3, Math.min(9 - k, 1))); };
  return hex(f(0) * 255, f(8) * 255, f(4) * 255);
}
const hueCache = new Map();
export const pastel = (hue, l = 0.78) => { const k = (hue | 0) * 100 + ((l * 100) | 0); let c = hueCache.get(k); if (!c) { c = hsl(hue, 0.7, l); hueCache.set(k, c); } return c; };
export const socColor = (soc) => (soc > 0.6 ? COL.mint : soc > 0.35 ? COL.yellow : soc > 0.2 ? COL.peach : COL.red);

/** Cheap deterministic hash noise in [0,1). */
const hash2 = (x, y) => { let h = (x * 374761393 + y * 668265263) | 0; h = (h ^ (h >>> 13)) * 1274126177; return (((h ^ (h >>> 16)) >>> 0) % 1000) / 1000; };

// ---------------------------------------------------------------------------------------------- tiny 3x5 pixel font
const FONT_ROWS = {
  'A': '.#.|#.#|###|#.#|#.#',
  'B': '##.|#.#|##.|#.#|##.',
  'C': '.##|#..|#..|#..|.##',
  'D': '##.|#.#|#.#|#.#|##.',
  'E': '###|#..|##.|#..|###',
  'F': '###|#..|##.|#..|#..',
  'G': '.##|#..|#.#|#.#|.##',
  'H': '#.#|#.#|###|#.#|#.#',
  'I': '###|.#.|.#.|.#.|###',
  'J': '..#|..#|..#|#.#|.#.',
  'K': '#.#|#.#|##.|#.#|#.#',
  'L': '#..|#..|#..|#..|###',
  'M': '#.#|###|###|#.#|#.#',
  'N': '##.|#.#|#.#|#.#|#.#',
  'O': '.#.|#.#|#.#|#.#|.#.',
  'P': '##.|#.#|##.|#..|#..',
  'Q': '.#.|#.#|#.#|##.|.##',
  'R': '##.|#.#|##.|#.#|#.#',
  'S': '.##|#..|.#.|..#|##.',
  'T': '###|.#.|.#.|.#.|.#.',
  'U': '#.#|#.#|#.#|#.#|###',
  'V': '#.#|#.#|#.#|#.#|.#.',
  'W': '#.#|#.#|###|###|#.#',
  'X': '#.#|#.#|.#.|#.#|#.#',
  'Y': '#.#|#.#|.#.|.#.|.#.',
  'Z': '###|..#|.#.|#..|###',
  '0': '###|#.#|#.#|#.#|###',
  '1': '.#.|##.|.#.|.#.|###',
  '2': '##.|..#|.#.|#..|###',
  '3': '##.|..#|.#.|..#|##.',
  '4': '#.#|#.#|###|..#|..#',
  '5': '###|#..|##.|..#|##.',
  '6': '.##|#..|###|#.#|###',
  '7': '###|..#|.#.|.#.|.#.',
  '8': '###|#.#|###|#.#|###',
  '9': '###|#.#|###|..#|##.',
  '-': '...|...|###|...|...',
  '.': '...|...|...|...|.#.',
  ':': '...|.#.|...|.#.|...',
  '%': '#.#|..#|.#.|#..|#.#',
  '/': '..#|..#|.#.|#..|#..',
};
const FONT = {};
for (const [k, v] of Object.entries(FONT_ROWS)) FONT[k] = v.split('|').join('');

export function text(ctx, str, x, y, color = COL.ink) {
  ctx.fillStyle = color;
  let cx = x | 0;
  for (const ch of String(str).toUpperCase()) {
    const g = FONT[ch];
    if (g) for (let i = 0; i < 15; i++) if (g[i] === '#') ctx.fillRect(cx + (i % 3), (y | 0) + ((i / 3) | 0), 1, 1);
    cx += 4;
  }
  return cx;
}

// ---------------------------------------------------------------------------------------------- primitives
function line(ctx, x0, y0, x1, y1, dotted = 0) {
  x0 |= 0; y0 |= 0; x1 |= 0; y1 |= 0;
  const dx = Math.abs(x1 - x0), dy = -Math.abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
  let err = dx + dy, n = 0;
  for (let g = 0; g < 600; g++) {
    if (!dotted || (n++ % dotted) === 0) ctx.fillRect(x0, y0, 1, 1);
    if (x0 === x1 && y0 === y1) break;
    const e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}
function circle(ctx, cx, cy, r, step = 1) {
  cx |= 0; cy |= 0; r = Math.round(r);
  let x = r, y = 0, err = 1 - r, n = 0;
  while (x >= y) {
    if (n++ % step === 0) for (const [a, b] of [[x, y], [y, x], [-y, x], [-x, y], [-x, -y], [-y, -x], [y, -x], [x, -y]]) ctx.fillRect(cx + a, cy + b, 1, 1);
    y++;
    if (err < 0) err += 2 * y + 1; else { x--; err += 2 * (y - x) + 1; }
  }
}
function disc(ctx, cx, cy, r, hatch = 0) {
  cx |= 0; cy |= 0; r = Math.round(r);
  for (let dy = -r; dy <= r; dy++) { const w = Math.floor(Math.sqrt(r * r - dy * dy)); for (let dx = -w; dx <= w; dx++) if (!hatch || ((cx + dx + cy + dy) % hatch === 0)) ctx.fillRect(cx + dx, cy + dy, 1, 1); }
}

export const toPx = (x, y, H) => [x / S, H - 1 - y / S];

// ---------------------------------------------------------------------------------------------- background (built once)
export function buildBackground(world, W, H, createImageData) {
  const img = createImageData(W, H), d = img.data;
  const hMin = world.cfg.baseGroundMsl ?? 222;
  for (let py = 0; py < H; py++) for (let px = 0; px < W; px++) {
    const x = (px + 0.5) * S, y = (H - 1 - py + 0.5) * S;
    const ci = world.cellIndex(x, y);
    let c = COL.land;
    const ground = world.terrainFast(x, y);
    const k = Math.max(0, Math.min(1, (ground - hMin) / 130));
    c = [COL.land[0] + (COL.hill[0] - COL.land[0]) * k, COL.land[1] + (COL.hill[1] - COL.land[1]) * k, COL.land[2] + (COL.hill[2] - COL.land[2]) * k];
    const ri = world.riverInfo(x, y);
    if (ri.dw <= 0) { const sh = hash2(px >> 1, py) < 0.12 ? COL.water2 : COL.water; c = sh; }
    else if (world.bldg[ci] > 0) {
      const hk = Math.min(1, world.bldg[ci] / 140);
      const edgeUp = world.bldg[world.cellIndex(x, Math.min(world.height - 1, y + S * 1.2))] < world.bldg[ci] - 1; // pixel-art lit roof edge
      c = edgeUp ? COL.bldgTop : [COL.bldg[0] + hk * 12, COL.bldg[1] + hk * 15, COL.bldg[2] + hk * 22];
    } else if (hash2(px, py) < 0.035) c = [c[0] + 12, c[1] + 14, c[2] + 24];
    const o = (py * W + px) * 4;
    d[o] = c[0]; d[o + 1] = c[1]; d[o + 2] = c[2]; d[o + 3] = 255;
  }
  return img;
}

// ---------------------------------------------------------------------------------------------- sprites
const SPRITES = {
  compact: { w: 9, rows: ['.BBBBBB..', 'BWWBBWWB.', 'BBBBBBBBB', '.TT..TT..'], rotors: [2, 6] },
  standard: { w: 11, rows: ['..BBBBBB...', '.BWWBBWWB..', 'BBBBBBBBBB.', 'BBBBBBBBBBB', '.TT....TT..'], rotors: [3, 8] },
  large: { w: 14, rows: ['.BBBBBBBBBBB..', 'BWWBWWBWWBWWB.', 'BBBBBBBBBBBBBB', 'BBBBBBBBBBBBBB', 'BBBBBBBBBBBBBB', '.TT........TT.'], rotors: [2, 6, 11] },
};
function shade(hexColor, k) {
  const n = parseInt(hexColor.slice(1), 16);
  return hex(((n >> 16) & 255) * k, ((n >> 8) & 255) * k, (n & 255) * k);
}
export function drawSprite(ctx, cls, cx, cy, body, flip, frame, alpha = 1) {
  const sp = SPRITES[cls] || SPRITES.standard, w = sp.w, h = sp.rows.length;
  const x0 = (cx - (w >> 1)) | 0, y0 = (cy - (h >> 1)) | 0;
  const dark = shade(body, 0.72);
  if (alpha < 1) ctx.globalAlpha = alpha;
  for (let r = 0; r < h; r++) {
    const row = sp.rows[r];
    for (let c = 0; c < w; c++) {
      const ch = row[c];
      if (ch === '.') continue;
      ctx.fillStyle = ch === 'B' ? (r >= h - 2 && h > 4 ? dark : body) : ch === 'W' ? '#eaf6ff' : '#1a2040';
      ctx.fillRect(x0 + (flip ? w - 1 - c : c), y0 + r, 1, 1);
    }
  }
  ctx.fillStyle = '#ffffff';
  for (const rx of sp.rotors) {
    const x = x0 + (flip ? w - 1 - rx : rx);
    if (frame & 1) ctx.fillRect(x - 1, y0 - 1, 3, 1); else ctx.fillRect(x, y0 - 1, 1, 1);
  }
  if (alpha < 1) ctx.globalAlpha = 1;
  return { x0, y0, w, h };
}

// ---------------------------------------------------------------------------------------------- layers
function drawZones(ctx, engine, H, frame) {
  for (const z of engine.world.restricted) {
    const [cx, cy] = toPx(z.x, z.y, H), r = z.r / S;
    if (z.active) { ctx.fillStyle = 'rgba(255,122,156,0.28)'; disc(ctx, cx, cy, r, 3); ctx.fillStyle = COL.red; circle(ctx, cx, cy, r, 2); }
    else { ctx.fillStyle = 'rgba(255,207,112,0.45)'; circle(ctx, cx, cy, r, 5); }
    text(ctx, z.id, cx - 5, cy - 2, z.active ? COL.red : COL.amber);
  }
  void frame;
}

function drawVertiports(ctx, engine, H, frame, showLabels = true) {
  const w = engine.world;
  for (const s of w.emergencySites) { const [x, y] = toPx(s.x, s.y, H); ctx.fillStyle = 'rgba(182,240,208,0.55)'; ctx.fillRect(x | 0, (y | 0) - 1, 1, 3); ctx.fillRect((x | 0) - 1, y | 0, 3, 1); }
  for (const v of w.vertiports) {
    const rt = engine.vertiports.get(v.id), [x, y] = toPx(v.x, v.y, H);
    const closed = rt && rt.closed, busy = rt ? rt.padBookings.some((l) => l.some((b) => b.s <= engine.t && b.e > engine.t)) : false;
    ctx.fillStyle = closed ? COL.red : busy ? COL.yellow : COL.sky;
    ctx.fillRect((x | 0) - 3, (y | 0) - 3, 7, 7);
    ctx.fillStyle = '#0a0f24'; ctx.fillRect((x | 0) - 2, (y | 0) - 2, 5, 5);
    ctx.fillStyle = closed ? COL.red : COL.sky; ctx.fillRect(x | 0, (y | 0) - 1, 1, 3); ctx.fillRect((x | 0) - 1, y | 0, 3, 1);
    if (closed && frame % 20 < 10) { ctx.fillStyle = COL.red; line(ctx, (x | 0) - 4, (y | 0) - 4, (x | 0) + 4, (y | 0) + 4); line(ctx, (x | 0) - 4, (y | 0) + 4, (x | 0) + 4, (y | 0) - 4); }
    if (showLabels) text(ctx, v.id, (x | 0) - 5, (y | 0) + 5, closed ? COL.red : COL.dim);
  }
}

function aircraftColor(ac) { return pastel(ac.hue); }
function pathColor(ac) { return ac.emergencyMode ? COL.pink : ac.frozen ? COL.amber : pastel(ac.hue, 0.7); }

function drawPath(ctx, ac, t, H, maxDots = 90) {
  const plan = ac.activePlan ? ac.activePlan() : ac.plan;
  if (!plan || plan.empty) return;
  ctx.fillStyle = pathColor(ac);
  let n = 0, px = ac.p[0] / S, py = H - 1 - ac.p[1] / S;
  for (const s of plan.segs) {
    if (s.t1 <= t) continue;
    const x1 = s.p1[0] / S, y1 = H - 1 - s.p1[1] / S;
    const dist = Math.hypot(x1 - px, y1 - py), steps = Math.floor(dist / 3);
    for (let k = 1; k <= steps && n < maxDots; k++, n++) ctx.fillRect(Math.round(px + ((x1 - px) * k) / (steps + 1)), Math.round(py + ((y1 - py) * k) / (steps + 1)), 1, 1);
    px = x1; py = y1;
    if (n >= maxDots) break;
  }
  const [dx, dy] = [px | 0, py | 0];
  ctx.fillRect(dx - 1, dy, 3, 1); ctx.fillRect(dx, dy - 1, 1, 3);
}

function drawAircraft(ctx, engine, H, frame, opts) {
  const { mode, selectedId } = opts;
  const list = engine.aircraft;
  for (const ac of list) {
    if (!ac.airborne) continue;
    const [x, y] = toPx(ac.p[0], ac.p[1], H);
    const agl = ac.p[2] - engine.world.terrainFast(ac.p[0], ac.p[1]);
    const lift = Math.max(0, Math.min(5, Math.round(agl / 60)));
    const cls = ac.spec.classKey || ac.spec.key || 'standard';
    ctx.fillStyle = 'rgba(0,0,0,0.38)';
    ctx.fillRect((x | 0) - 3, y | 0, 7, 1);
    let body = aircraftColor(ac);
    if (mode === 'battery') body = socColor(ac.soc);
    if (ac.mode === 'EMERGENCY' && frame % 16 < 8) body = COL.pink;
    const dim = mode === 'wind' || mode === 'windmap' || mode === 'noise' ? 0.75 : 1;
    const box = drawSprite(ctx, cls, x, y - lift - 2, body, ac.v[0] < -0.5, frame + ac.id, dim);
    if (mode === 'battery') {
      const wbar = Math.max(1, Math.round(box.w * ac.soc));
      ctx.fillStyle = '#1a2040'; ctx.fillRect(box.x0, box.y0 - 4, box.w, 2);
      ctx.fillStyle = socColor(ac.soc); ctx.fillRect(box.x0, box.y0 - 4, wbar, 2);
    }
    if (ac.id === selectedId) { ctx.fillStyle = '#ffffff'; circle(ctx, x, y - lift - 2, 9, 2); }
    if (ac.health && (ac.health.commLoss || ac.health.gps !== 'ok' || ac.health.thrustFactor < 1 || ac.health.drainMult > 1 || ac.health.sensor !== 'ok') && frame % 12 < 8) { ctx.fillStyle = COL.amber; ctx.fillRect(box.x0 + box.w, box.y0 - 2, 2, 2); }
  }
}

function drawGroundAircraft(ctx, engine, H) {
  for (const v of engine.world.vertiports) {
    const rt = engine.vertiports.get(v.id);
    const n = rt ? rt.hangar.size : 0;
    if (!n) continue;
    const [x, y] = toPx(v.x, v.y, H);
    ctx.fillStyle = COL.dim;
    for (let i = 0; i < Math.min(8, n); i++) ctx.fillRect((x | 0) - 4 + i, (y | 0) + 8, 1, 1);
  }
}

function drawExternal(ctx, engine, H, frame) {
  for (const e of engine.ext) {
    const [x, y] = toPx(e.p[0], e.p[1], H);
    ctx.fillStyle = COL.peach;
    ctx.fillRect((x | 0) - 1, y | 0, 3, 1); ctx.fillRect(x | 0, (y | 0) - 1, 1, 3);
    ctx.fillStyle = 'rgba(255,214,165,0.5)'; line(ctx, x, y, x - (e.v[0] * 25) / S, y + (e.v[1] * 25) / S, 2);
    if (frame % 20 < 12) { ctx.fillStyle = COL.peach; circle(ctx, x, y, 4, 2); }
  }
}

// -- wind
export class WindField {
  constructor(engine, W, H, nParticles = 900) {
    this.W = W; this.H = H; this.engine = engine; this.nx = 72; this.ny = 48;
    this.ux = new Float32Array(this.nx * this.ny); this.uy = new Float32Array(this.nx * this.ny); this.nextUpdate = -1;
    this.parts = Array.from({ length: nParticles }, (_, i) => ({ x: hash2(i, 1) * W, y: hash2(i, 2) * H, age: (hash2(i, 3) * 80) | 0, hx: [0, 0, 0], hy: [0, 0, 0] }));
  }
  refresh(t, z = 380) {
    if (t < this.nextUpdate) return;
    this.nextUpdate = t + 2;
    const w = this.engine.world, tmp = [0, 0, 0];
    for (let iy = 0; iy < this.ny; iy++) for (let ix = 0; ix < this.nx; ix++) {
      const v = this.engine.truth.at((ix + 0.5) * w.cell, (iy + 0.5) * w.cell, z, t, tmp);
      this.ux[iy * this.nx + ix] = v[0]; this.uy[iy * this.nx + ix] = v[1];
    }
  }
  sample(px, py) { // pixel -> wind (m/s)
    const gx = Math.max(0, Math.min(this.nx - 1.001, (px * S) / 100 - 0.5)), gy = Math.max(0, Math.min(this.ny - 1.001, ((this.H - 1 - py) * S) / 100 - 0.5));
    const ix = gx | 0, iy = gy | 0, fx = gx - ix, fy = gy - iy, i = iy * this.nx + ix;
    const bl = (a) => a[i] * (1 - fx) * (1 - fy) + a[i + 1] * fx * (1 - fy) + a[i + this.nx] * (1 - fx) * fy + a[i + this.nx + 1] * fx * fy;
    return [bl(this.ux), bl(this.uy)];
  }
  step(dt) {
    for (const p of this.parts) {
      const [ux, uy] = this.sample(p.x, p.y);
      p.hx.unshift(p.x); p.hy.unshift(p.y); p.hx.length = 3; p.hy.length = 3;
      p.x += (ux * dt * 10) / S; p.y -= (uy * dt * 10) / S;
      if (++p.age > 90 || p.x < 0 || p.y < 0 || p.x >= this.W || p.y >= this.H) { p.x = Math.random() * this.W; p.y = Math.random() * this.H; p.age = 0; p.hx = [p.x, p.x, p.x]; p.hy = [p.y, p.y, p.y]; }
    }
  }
}
const windRamp = (s) => (s < 3 ? '#7fb6ff' : s < 6 ? '#a8d8ff' : s < 9 ? '#b6f0d0' : s < 12 ? '#fff2a8' : s < 15 ? '#ffd6a5' : '#ffb3c7');

function drawWind(ctx, engine, wf, W, H, frame) {
  wf.refresh(engine.t);
  ctx.fillStyle = 'rgba(6,9,24,0.6)'; ctx.fillRect(0, 0, W, H);
  // current arrows on a coarse grid
  for (let gy = 9; gy < H; gy += 18) for (let gx = 9; gx < W; gx += 18) {
    const [ux, uy] = wf.sample(gx, gy), sp = Math.hypot(ux, uy);
    if (sp < 0.3) continue;
    const len = Math.min(13, 3 + sp * 1.0), dx = (ux / sp) * len, dy = (-uy / sp) * len;
    ctx.fillStyle = windRamp(sp); line(ctx, gx, gy, gx + dx, gy + dy);
    ctx.fillStyle = '#ffffff'; ctx.fillRect(Math.round(gx + dx), Math.round(gy + dy), 1, 1);
  }
  // particles
  for (const p of wf.parts) {
    const [ux, uy] = wf.sample(p.x, p.y), c = windRamp(Math.hypot(ux, uy));
    const fade = p.age < 8 ? p.age / 8 : p.age > 75 ? (90 - p.age) / 15 : 1;
    ctx.globalAlpha = Math.max(0.1, fade);
    ctx.fillStyle = c; ctx.fillRect(p.x | 0, p.y | 0, 1, 1);
    ctx.globalAlpha = Math.max(0.05, fade * 0.45); ctx.fillRect(p.hx[1] | 0, p.hy[1] | 0, 1, 1); ctx.fillRect(p.hx[2] | 0, p.hy[2] | 0, 1, 1);
  }
  ctx.globalAlpha = 1;
  // hazard zones broadcast by aircraft
  const hz = engine.wind.hazards || [];
  ctx.fillStyle = COL.pink;
  for (const h of hz) { const [x, y] = toPx(h.x, h.y, H); circle(ctx, x, y, h.r / S, 2); }
  void frame;
}

let heatCache = null;
function drawNoise(ctx, engine, W, H) {
  const w = engine.world;
  for (let iy = 0; iy < w.ny; iy++) for (let ix = 0; ix < w.nx; ix++) {
    const v = Math.min(1, w.noiseSens[iy * w.nx + ix] / 1.3);
    if (v < 0.08) continue;
    ctx.fillStyle = `rgba(${(180 + 75 * v) | 0},${(140 - 60 * v) | 0},${(230 - 90 * v) | 0},${(0.1 + 0.5 * v).toFixed(2)})`;
    ctx.fillRect(ix * 5, H - (iy + 1) * 5, 5, 5);
  }
  void W; void heatCache;
}

export class WindErrorField {
  constructor(engine) { this.engine = engine; this.next = -1; this.err = new Float32Array(36 * 24); }
  refresh(t, z = 380) {
    if (t < this.next) return; this.next = t + 3;
    const e = this.engine, a = [0, 0, 0], b = [0, 0, 0];
    for (let iy = 0; iy < 24; iy++) for (let ix = 0; ix < 36; ix++) {
      const x = (ix + 0.5) * 200, y = (iy + 0.5) * 200;
      e.truth.at(x, y, z, t, a); e.wind.at(x, y, z, t, b);
      this.err[iy * 36 + ix] = Math.hypot(a[0] - b[0], a[1] - b[1]);
    }
  }
}
function drawWindMap(ctx, engine, wef, H) {
  wef.refresh(engine.t);
  for (let iy = 0; iy < 24; iy++) for (let ix = 0; ix < 36; ix++) {
    const v = Math.min(1, wef.err[iy * 36 + ix] / 4);
    ctx.fillStyle = `rgba(${(120 + 135 * v) | 0},${(200 - 90 * v) | 0},${(230 - 70 * v) | 0},${(0.08 + 0.55 * v).toFixed(2)})`;
    ctx.fillRect(ix * 10, H - (iy + 1) * 10, 10, 10);
  }
  // probe observations from the fleet
  ctx.fillStyle = COL.mint;
  for (const ac of engine.aircraft) if (ac.airborne) { const [x, y] = toPx(ac.p[0], ac.p[1], H); ctx.fillRect(x | 0, y | 0, 1, 1); }
}

function drawSafety(ctx, engine, H, frame) {
  const air = engine.aircraft.filter((a) => a.airborne);
  for (const ac of air) {
    const [x, y] = toPx(ac.p[0], ac.p[1], H);
    ctx.fillStyle = ac.cbfActive ? COL.red : 'rgba(168,216,255,0.35)';
    circle(ctx, x, y, Math.max(2, (45 + (ac.nav ? ac.nav.sigma : 0) * 1.5) / S), ac.cbfActive ? 1 : 3);
  }
  for (let i = 0; i < air.length; i++) for (let j = i + 1; j < air.length; j++) {
    const a = air[i], b = air[j];
    const dx = a.p[0] - b.p[0], dy = a.p[1] - b.p[1];
    if (Math.abs(dx) > 250 || Math.abs(dy) > 250) continue;
    const d = Math.hypot(dx, dy, a.p[2] - b.p[2]);
    if (d > 250) continue;
    ctx.fillStyle = d < 70 ? COL.red : d < 130 ? COL.amber : 'rgba(182,240,208,0.5)';
    const [x0, y0] = toPx(a.p[0], a.p[1], H), [x1, y1] = toPx(b.p[0], b.p[1], H);
    line(ctx, x0, y0, x1, y1, 2);
  }
  for (const e of engine.ext) { const [x, y] = toPx(e.p[0], e.p[1], H); ctx.fillStyle = COL.peach; circle(ctx, x, y, (e.sigma + 60) / S, 2); }
  void frame;
}

function drawAirspace(ctx, engine, H) {
  const w = engine.world, k = 1; // level 380 MSL
  ctx.fillStyle = 'rgba(210,184,255,0.22)';
  for (let iy = 0; iy < w.ny; iy++) for (let ix = 0; ix < w.nx; ix++) if (w.blocked[k * w.layer + iy * w.nx + ix]) ctx.fillRect(ix * 5, H - (iy + 1) * 5, 5, 5);
}

// -- altitude side view
export function drawSide(ctx, engine, W, SH, selectedId) {
  const w = engine.world, zMin = 230, zMax = 540, ys = (z) => SH - 1 - Math.round(((z - zMin) / (zMax - zMin)) * (SH - 1));
  ctx.fillStyle = '#080c1e'; ctx.fillRect(0, 0, W, SH);
  ctx.fillStyle = '#16204a';
  for (let px = 0; px < W; px++) {
    let top = 0;
    for (let iy = 0; iy < w.ny; iy += 3) { const ci = iy * w.nx + Math.min(w.nx - 1, Math.floor((px * S) / w.cell)); top = Math.max(top, w.ground[ci] + w.bldg[ci]); }
    const y = ys(top); ctx.fillRect(px, y, 1, SH - y);
  }
  ctx.fillStyle = 'rgba(168,216,255,0.35)';
  for (const z of w.levels) line(ctx, 0, ys(z), W - 1, ys(z), 3);
  for (const ac of engine.aircraft) {
    if (!ac.airborne) continue;
    ctx.fillStyle = ac.id === selectedId ? '#ffffff' : ac.emergencyMode ? COL.pink : pastel(ac.hue);
    const x = Math.round(ac.p[0] / S), y = ys(ac.p[2]);
    ctx.fillRect(x, y, 2, 2);
  }
  for (const e of engine.ext) { ctx.fillStyle = COL.peach; ctx.fillRect(Math.round(e.p[0] / S), ys(e.p[2]), 2, 1); }
  text(ctx, 'ALT', 2, 2, COL.dim);
}

// ---------------------------------------------------------------------------------------------- frame
export class Renderer {
  constructor(engine, { W = 360, H = 240, createImageData }) {
    this.engine = engine; this.W = W; this.H = H; this.frame = 0;
    this.bg = buildBackground(engine.world, W, H, createImageData);
    this.wf = new WindField(engine, W, H); this.wef = new WindErrorField(engine);
  }
  setEngine(engine) { this.engine = engine; this.wf = new WindField(engine, this.W, this.H); this.wef = new WindErrorField(engine); }
  draw(ctx, { view = 'traffic', selectedId = null, showPaths = true } = {}) {
    const e = this.engine, { W, H } = this; this.frame++;
    ctx.putImageData(this.bg, 0, 0);
    if (view === 'noise') drawNoise(ctx, e, W, H);
    if (view === 'airspace') drawAirspace(ctx, e, H);
    drawZones(ctx, e, H, this.frame);
    if (view === 'windmap') drawWindMap(ctx, e, this.wef, H);
    if (view === 'wind') { this.wf.step(0.1); drawWind(ctx, e, this.wf, W, H, this.frame); }
    drawVertiports(ctx, e, H, this.frame, view !== 'wind');
    drawGroundAircraft(ctx, e, H);
    if (showPaths && (view === 'traffic' || view === 'airspace' || view === 'safety')) {
      const air = e.aircraft.filter((a) => a.airborne);
      const sel = selectedId !== null ? e.aircraft[selectedId] : null;
      const many = air.length > 90;
      for (const ac of air) if (!many || ac === sel || ac.emergencyMode || ac.frozen) drawPath(ctx, ac, e.t, H, ac === sel ? 400 : 70);
      if (many && sel && sel.airborne) drawPath(ctx, sel, e.t, H, 400);
    } else if (selectedId !== null && e.aircraft[selectedId] && e.aircraft[selectedId].airborne) drawPath(ctx, e.aircraft[selectedId], e.t, H, 400);
    if (view === 'safety') drawSafety(ctx, e, H, this.frame);
    drawExternal(ctx, e, H, this.frame);
    drawAircraft(ctx, e, H, this.frame, { mode: view, selectedId });
  }
  /** Return aircraft id nearest to canvas pixel (px, py) within r pixels, else null. */
  pick(px, py, r = 10) {
    let best = null, bd = r * r;
    for (const ac of this.engine.aircraft) {
      if (!ac.airborne) continue;
      const [x, y] = toPx(ac.p[0], ac.p[1], this.H), d = (x - px) ** 2 + (y - py) ** 2;
      if (d < bd) { bd = d; best = ac.id; }
    }
    return best;
  }
}
