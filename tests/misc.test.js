import test from 'node:test';
import assert from 'node:assert/strict';
import { parseOpenMeteo, parseOpenSky, overpassToGeoJSON, overpassQuery, OPEN_METEO_URL } from '../src/data/loaders.js';
import { VertiportRuntime } from '../src/fleet/vertiports.js';
import { EventLog, Metrics } from '../src/sim/events.js';
import { TrafficGenerator, ExtAircraft } from '../src/sim/traffic.js';
import { RNG } from '../src/core/util.js';
import { Renderer, VIEWS, drawSide, buildBackground, text, pastel, socColor, S } from '../web/render.js';
import { createApp } from '../scripts/serve.js';
import { sharedWorld, engineWithFlights, stubCtx, makeEngine } from './helpers.js';

test('Open-Meteo parser maps current wind to simulator settings', () => {
  const p = parseOpenMeteo({ current: { wind_speed_10m: 4.2, wind_direction_10m: 255, wind_gusts_10m: 9.5 } }); assert.equal(p.speed10, 4.2); assert.equal(p.dirFrom, 255); assert.ok(p.gust > 0.5 && p.gust <= 3);
  assert.equal(parseOpenMeteo({}), null); assert.equal(parseOpenMeteo({ current: { wind_speed_10m: 'x' } }), null); assert.equal(parseOpenMeteo({ current: { wind_speed_10m: 3, wind_direction_10m: -90 } }).dirFrom, 270);
  assert.match(OPEN_METEO_URL(40.44, -79.99), /latitude=40.44.*longitude=-79.99/);
});
test('OpenSky parser keeps airborne aircraft inside the map and converts units', () => {
  const w = sharedWorld(), c = w.cfg;
  const state = (icao, lat, lon, alt, ground) => [icao, 'CALL', 'US', 0, 0, lon, lat, alt, ground, 50, 90, 2];
  const out = parseOpenSky({ states: [state('a1', c.lat0, c.lon0, 400, false), state('b2', c.lat0, c.lon0, 0, true), state('c3', c.lat0 + 5, c.lon0, 400, false), [null]] }, w);
  assert.equal(out.length, 1); assert.ok(out[0] instanceof ExtAircraft); assert.ok(Math.abs(out[0].v[0] - 50) < 1e-6 && Math.abs(out[0].v[1]) < 1e-6, 'track 090 => due east'); assert.ok(w.inBounds(out[0].p[0], out[0].p[1]));
});
test('Overpass JSON converts to GeoJSON polygons and the query is well formed', () => {
  const g = overpassToGeoJSON({ elements: [{ type: 'way', tags: { building: 'yes', height: '30' }, geometry: [{ lat: 1, lon: 1 }, { lat: 1, lon: 2 }, { lat: 2, lon: 2 }, { lat: 1, lon: 1 }] }, { type: 'node' }, { type: 'way', geometry: [{ lat: 1, lon: 1 }] }] });
  assert.equal(g.features.length, 1); assert.equal(g.features[0].properties.height, '30'); assert.match(overpassQuery(1, 2, 3, 4), /way\["building"\]\(1,2,3,4\)/);
});
test('vertiport runtime: single operations column, kinds, closure and charging', () => {
  const def = { id: 'T', name: 'T', ix: 1, iy: 1, x: 150, y: 150, padElev: 300, pads: 4, chargers: 2, closed: false }, v = new VertiportRuntime(def);
  assert.equal(v.fatos, 1); assert.deepEqual(v.findSlot(0, 10), { pad: 0, start: 0 }); v.book(1, 0, 0, 10, 'takeoff'); assert.equal(v.findSlot(5, 10).start, 10); assert.equal(v.findSlot(5, 10, 1).start, 5, 'ignoring own booking');
  v.book(1, 0, 20, 30, 'land'); v.release(1, 'takeoff'); assert.equal(v.padBookings[0].length, 1); v.release(1); assert.equal(v.padBookings[0].length, 0);
  v.closed = true; assert.equal(v.findSlot(0, 5), null); v.closed = false;
  assert.ok(v.tryStartCharging(1) && v.tryStartCharging(2) && !v.tryStartCharging(3)); v.stopCharging(1); assert.ok(v.tryStartCharging(3));
});
test('event log keeps order, filters by level and notifies listeners', () => { const l = new EventLog(5), seen = []; l.on((e) => seen.push(e.text)); for (let i = 0; i < 8; i++) l.add(i, i % 2 ? 'warn' : 'info', 'e' + i); assert.equal(l.items.length, 5); assert.equal(seen.length, 8); assert.deepEqual(l.recent(10, ['warn']).map((e) => e.text), ['e3', 'e5', 'e7']); });
test('metrics summary handles an empty run', () => { const s = new Metrics().summary(); assert.equal(s.completed, 0); assert.equal(s.minSeparation, null); });
test('traffic generator spawns external aircraft that cross the map and intruders aim at a target', () => {
  const w = sharedWorld(), g = new TrafficGenerator(w, new RNG(4), 1 / 5); let ext = []; for (let t = 0; t < 120; t++) { const n = g.tick(t); if (n) ext.push(n); }
  assert.ok(ext.length > 3); assert.ok(ext.every((x) => x instanceof ExtAircraft)); const i = g.intruderAt([3000, 2000, 400], 20, 40, null); assert.ok(i instanceof ExtAircraft);
  const q = [...i.p]; for (let k = 0; k < 20; k++) i.step(1); assert.ok(Math.hypot(i.p[0] - 3000, i.p[1] - 2000, i.p[2] - 400) < Math.hypot(q[0] - 3000, q[1] - 2000, q[2] - 400));
});
const imgData = (w, h) => ({ width: w, height: h, data: new Uint8ClampedArray(w * h * 4) });
test('renderer draws every view without NaNs or exceptions, with a selection and with faults', () => {
  const e = engineWithFlights(5, { seed: 6 }); e.injectFault('gps_bias'); e.injectFault('storm'); e.advance(20);
  const r = new Renderer(e, { createImageData: imgData }), sel = e.aircraft.find((a) => a.airborne).id;
  for (const v of VIEWS) { const c = stubCtx(); for (let i = 0; i < 3; i++) r.draw(c, { view: v.id, selectedId: sel }); assert.ok(c.n > 200, v.id); }
  const c = stubCtx(); drawSide(c, e, 360, 64, sel); assert.ok(c.n > 100);
});
test('renderer: seven views, background fills the canvas opaque, picking finds aircraft', () => {
  assert.equal(VIEWS.length, 7); const e = engineWithFlights(3, { seed: 6 }), r = new Renderer(e, { createImageData: imgData });
  for (let i = 3; i < r.bg.data.length; i += 4 * 997) assert.equal(r.bg.data[i], 255);
  const ac = e.aircraft.find((a) => a.airborne); const px = ac.p[0] / S, py = r.H - 1 - ac.p[1] / S; assert.equal(r.pick(px, py, 3), ac.id); assert.equal(r.pick(-100, -100, 3), null);
});
test('pixel font covers every character used in vertiport IDs and the colour helpers behave', () => {
  const c = stubCtx(); for (const v of [...sharedWorld().vertiports, ...sharedWorld().emergencySites]) text(c, v.id, 0, 0); assert.ok(c.n > 100);
  assert.match(pastel(120), /^#[0-9a-f]{6}$/); assert.notEqual(socColor(0.9), socColor(0.1));
  assert.equal(imgData(2, 2).data.length, 16); assert.ok(buildBackground(sharedWorld(), 36, 24, imgData).data.length === 36 * 24 * 4);
});
test('static server: serves the UI and modules, redirects /, blocks traversal, 404s unknown paths', async () => {
  const app = createApp(); await new Promise((r) => app.listen(0, r)); const base = 'http://127.0.0.1:' + app.address().port;
  try {
    const root = await fetch(base + '/', { redirect: 'manual' }); assert.equal(root.status, 302); assert.equal(root.headers.get('location'), '/web/');
    const html = await fetch(base + '/web/'); assert.equal(html.status, 200); assert.match(await html.text(), /Swagmo/);
    const js = await fetch(base + '/src/sim/engine.js'); assert.equal(js.status, 200); assert.match(js.headers.get('content-type'), /javascript/);
    assert.equal((await fetch(base + '/nope.txt')).status, 404);
    const trav = await fetch(base + '/..%2f..%2fetc%2fpasswd'); assert.ok([403, 404].includes(trav.status));
  } finally { app.close(); }
});
test('every browser module the UI imports exists and parses', async () => { for (const f of ['../web/main.js', '../web/render.js']) { const { readFileSync } = await import('node:fs'); assert.ok(readFileSync(new URL(f, import.meta.url), 'utf8').length > 1000); } });
