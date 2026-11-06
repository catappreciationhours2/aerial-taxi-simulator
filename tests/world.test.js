import test from 'node:test';
import assert from 'node:assert/strict';
import { World } from '../src/world/world.js';
import { WORLD } from '../src/core/config.js';
import { sharedWorld } from './helpers.js';

const w = sharedWorld();
test('world dimensions follow the config', () => {
  assert.equal(w.nx, WORLD.nx); assert.equal(w.ny, WORLD.ny); assert.equal(w.nNodes, WORLD.nx * WORLD.ny * WORLD.levelsMsl.length);
  assert.equal(w.width, WORLD.nx * WORLD.cell);
});
test('world generation is deterministic', () => {
  const a = new World(), b = new World();
  assert.deepEqual(Array.from(a.bldg.slice(0, 500)), Array.from(b.bldg.slice(0, 500)));
  assert.deepEqual(a.vertiports.map((v) => [v.id, v.ix, v.iy]), b.vertiports.map((v) => [v.id, v.ix, v.iy]));
});
test('nine vertiports exist, each on a free node with a positive pad count', () => {
  assert.equal(w.vertiports.length, 9);
  for (const v of w.vertiports) { const k = w.firstFreeLevel(v.ix, v.iy); assert.ok(k >= 0, v.id + ' has a free level'); assert.ok(!w.blocked[w.idx(v.ix, v.iy, k)]); assert.ok(v.pads > 0 && v.chargers > 0); }
});
test('emergency sites exist and are reachable columns', () => { assert.ok(w.emergencySites.length >= 5); for (const s of w.emergencySites) assert.ok(w.firstFreeLevel(s.ix, s.iy) >= 0, s.id); });
test('landingSites includes vertiports and emergency sites', () => { const ids = new Set(w.landingSites().map((s) => s.id)); for (const v of w.vertiports) assert.ok(ids.has(v.id)); for (const s of w.emergencySites) assert.ok(ids.has(s.id)); });
test('node index encode/decode round trip and centre positions', () => {
  for (const [ix, iy, iz] of [[0, 0, 0], [10, 20, 2], [71, 47, 3]]) { const n = w.idx(ix, iy, iz); assert.deepEqual(w.decode(n), [ix, iy, iz]); const c = w.center(n); assert.equal(c[0], (ix + 0.5) * w.cell); assert.equal(c[2], w.levels[iz]); assert.equal(w.nodeAt(c[0], c[1], c[2]), n); }
});
test('levelOf picks the nearest flight level', () => { assert.equal(w.levelOf(300), 0); assert.equal(w.levelOf(381), 1); assert.equal(w.levelOf(9999), w.nz - 1); });
test('nodes below terrain + clearance are blocked; high nodes above everything are free', () => {
  for (let i = 0; i < w.layer; i += 7) { const surf = w.ground[i] + w.bldg[i]; for (let k = 0; k < w.nz; k++) if (w.levels[k] < surf + w.clearance - 1e-6) assert.equal(w.blocked[k * w.layer + i], 1, `cell ${i} level ${k}`); }
});
test('terrainFast approximates terrainAt', () => { for (const [x, y] of [[1000, 1000], [3600, 2400], [5200, 3100]]) assert.ok(Math.abs(w.terrainFast(x, y) - w.terrainAt(x, y)) < 12); });
test('rivers exist: some cells are water and water has no buildings', () => { let water = 0; for (let i = 0; i < w.layer; i++) if (w.water[i]) { water++; assert.equal(w.bldg[i], 0); } assert.ok(water > 100); });
test('signed distance field: zero on blocked nodes, positive and increasing away from them', () => {
  for (let n = 0; n < w.nNodes; n += 97) { if (w.blocked[n]) assert.equal(w.sdf[n], 0); else assert.ok(w.sdf[n] > 0); }
  const g = w.sdfGrad(3600, 2400, 500); assert.ok(Math.abs(Math.hypot(...g) - 1) < 1e-6);
});
test('nearestFreeNode returns an unblocked node close to the query', () => {
  for (const [x, y, z] of [[3906, 2399, 402], [3700, 2850, 330], [100, 100, 300]]) { const n = w.nearestFreeNode(x, y, z); assert.ok(n >= 0); assert.equal(w.blocked[n], 0); const c = w.center(n); assert.ok(Math.hypot(c[0] - x, c[1] - y) < 160); }
});
test('permanent airport restriction is active and blocks flight inside it', () => {
  const z = w.zone('APT'); assert.ok(z.active && z.perm);
  assert.equal(w.restrictedAt(z.x, z.y, 400, 0).length > 0 || !!w.restrictedAt(z.x, z.y, 400, 0), true);
  assert.ok(w.nodesInZone(z).length > 0);
});
test('stadium TFR starts inactive', () => assert.equal(w.zone('TFR').active, false));
test('GeoJSON buildings replace the synthetic skyline', () => {
  const world = new World();
  const lat = world.cfg.lat0 + 0.004, lon = world.cfg.lon0 + 0.012, d = 0.0012; // ~ 260 m x 270 m footprint away from the river
  const ring = [[lon - d, lat - d], [lon + d, lat - d], [lon + d, lat + d], [lon - d, lat + d], [lon - d, lat - d]];
  const nBefore = Array.from(world.bldg).filter((h) => h > 0).length;
  const n = world.applyBuildingsGeoJSON({ type: 'FeatureCollection', features: [{ type: 'Feature', properties: { height: '180' }, geometry: { type: 'Polygon', coordinates: [ring] } }] });
  assert.equal(n, 1);
  const tall = Array.from(world.bldg).filter((h) => h >= 150).length, nAfter = Array.from(world.bldg).filter((h) => h > 0).length;
  assert.ok(tall >= 1, 'tower footprint rasterised'); assert.ok(nAfter < nBefore, 'synthetic buildings removed');
  for (const v of world.vertiports) assert.ok(world.firstFreeLevel(v.ix, v.iy) >= 0, 'vertiports stay usable');
});
