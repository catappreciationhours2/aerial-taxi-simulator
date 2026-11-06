import test from 'node:test';
import assert from 'node:assert/strict';
import { geodeticToEcef, ecefToGeodetic, ENUFrame, ellipsoidToMsl, mslToEllipsoid, aglToMsl, mslToAgl, PITTSBURGH, GEOID_UNDULATION_PIT } from '../src/core/geo.js';
import { near } from './helpers.js';

test('geodetic <-> ECEF round trip (WGS84)', () => {
  for (const [lat, lon, h] of [[40.44, -79.99, 300], [0, 0, 0], [-33.9, 151.2, 50], [89, 10, 1000]]) {
    const [x, y, z] = geodeticToEcef(lat, lon, h);
    const g = ecefToGeodetic(x, y, z); const arr = Array.isArray(g) ? g : [g.lat, g.lon, g.h];
    near(arr[0], lat, 1e-6); near(arr[1], lon, 1e-6); near(arr[2], h, 1e-2);
  }
});
test('ECEF radius at the equator equals the WGS84 semi-major axis', () => { const [x] = geodeticToEcef(0, 0, 0); near(x, 6378137, 1e-3); });
test('ENU frame: origin is zero, north/east axes are right-handed, round trip', () => {
  const f = new ENUFrame(PITTSBURGH.lat, PITTSBURGH.lon, 0);
  const o = f.toENU(PITTSBURGH.lat, PITTSBURGH.lon, 0); near(o[0], 0, 1e-6); near(o[1], 0, 1e-6); near(o[2], 0, 1e-6);
  const n = f.toENU(PITTSBURGH.lat + 0.01, PITTSBURGH.lon, 0); assert.ok(n[1] > 1000 && Math.abs(n[0]) < 5);
  const e = f.toENU(PITTSBURGH.lat, PITTSBURGH.lon + 0.01, 0); assert.ok(e[0] > 800 && Math.abs(e[1]) < 5);
  const u = f.toENU(PITTSBURGH.lat, PITTSBURGH.lon, 100); near(u[2], 100, 0.01);
  const back = f.toGeodetic(...e); const arr = Array.isArray(back) ? back : [back.lat, back.lon, back.h];
  near(arr[0], PITTSBURGH.lat, 1e-7); near(arr[1], PITTSBURGH.lon + 0.01, 1e-7);
});
test('1 degree of latitude is about 111 km', () => { const f = new ENUFrame(40, -80, 0); near(f.toENU(41, -80, 0)[1], 111000, 600); });
test('altitude datum conversions (ellipsoid / MSL / AGL)', () => {
  near(ellipsoidToMsl(mslToEllipsoid(300)), 300, 1e-9); assert.equal(GEOID_UNDULATION_PIT, -33);
  near(mslToEllipsoid(300), 300 + GEOID_UNDULATION_PIT, 1e-9);
  assert.equal(aglToMsl(100, 250), 350); assert.equal(mslToAgl(350, 250), 100);
});
