// Pittsburgh's lat/lon and conversions between degrees and local metres as used by the data loaders

// Geodesy: WGS-84 geodetic <-> ECEF <-> local East-North-Up (ENU), plus the three
// altitude references:
//   ellipsoidal height  (what a raw GNSS receiver reports)
//   MSL (orthometric)   (what terrain data / altimeters / airspace floors use)
//   AGL                 (what clearance rules use)
// Mixing them silently moves an aircraft by tens of metres (Pittsburgh geoid undulation ~ -33 m).

import { DEG } from './util.js';

const A = 6378137.0; // WGS-84 semi-major axis
const F = 1 / 298.257223563;
const E2 = F * (2 - F);

/** Approximate EGM96 geoid undulation N near Pittsburgh (m). h_ellipsoid = h_msl + N. Assumed constant over the map. */
export const GEOID_UNDULATION_PIT = -33.0;

export function geodeticToEcef(latDeg, lonDeg, h) {
  const lat = latDeg * DEG, lon = lonDeg * DEG;
  const s = Math.sin(lat), c = Math.cos(lat);
  const N = A / Math.sqrt(1 - E2 * s * s);
  return [(N + h) * c * Math.cos(lon), (N + h) * c * Math.sin(lon), (N * (1 - E2) + h) * s];
}

// Iterative (fixed-point) ECEF -> geodetic; converges to < 1e-9 deg in a few iterations for near-surface points.
export function ecefToGeodetic(x, y, z) {
  const lon = Math.atan2(y, x);
  const p = Math.hypot(x, y);
  let lat = Math.atan2(z, p * (1 - E2));
  let h = 0;
  for (let i = 0; i < 8; i++) {
    const s = Math.sin(lat);
    const N = A / Math.sqrt(1 - E2 * s * s);
    h = p / Math.cos(lat) - N;
    lat = Math.atan2(z, p * (1 - (E2 * N) / (N + h)));
  }
  return { lat: lat / DEG, lon: lon / DEG, h };
}

export class ENUFrame {
  /** @param {number} lat0 deg  @param {number} lon0 deg  @param {number} h0 ellipsoidal height of origin (m) */
  constructor(lat0, lon0, h0 = 0) {
    this.lat0 = lat0; this.lon0 = lon0; this.h0 = h0;
    this.ecef0 = geodeticToEcef(lat0, lon0, h0);
    const la = lat0 * DEG, lo = lon0 * DEG;
    const sl = Math.sin(la), cl = Math.cos(la), so = Math.sin(lo), co = Math.cos(lo);
    this.R = [
      [-so, co, 0],
      [-sl * co, -sl * so, cl],
      [cl * co, cl * so, sl],
    ];
  }
  /** geodetic (deg, deg, ellipsoidal metres) -> [east, north, up] metres */
  toENU(lat, lon, h = 0) {
    const e = geodeticToEcef(lat, lon, h);
    const d = [e[0] - this.ecef0[0], e[1] - this.ecef0[1], e[2] - this.ecef0[2]];
    const R = this.R;
    return [
      R[0][0] * d[0] + R[0][1] * d[1] + R[0][2] * d[2],
      R[1][0] * d[0] + R[1][1] * d[1] + R[1][2] * d[2],
      R[2][0] * d[0] + R[2][1] * d[1] + R[2][2] * d[2],
    ];
  }
  toGeodetic(x, y, z) {
    const R = this.R;
    const d = [
      R[0][0] * x + R[1][0] * y + R[2][0] * z,
      R[0][1] * x + R[1][1] * y + R[2][1] * z,
      R[0][2] * x + R[1][2] * y + R[2][2] * z,
    ];
    return ecefToGeodetic(d[0] + this.ecef0[0], d[1] + this.ecef0[1], d[2] + this.ecef0[2]);
  }
}

// ----- altitude reference conversions (explicit names so call sites cannot mix them up) -----
export const ellipsoidToMsl = (hEll, undulation = GEOID_UNDULATION_PIT) => hEll - undulation;
export const mslToEllipsoid = (hMsl, undulation = GEOID_UNDULATION_PIT) => hMsl + undulation;
export const aglToMsl = (agl, groundMsl) => agl + groundMsl;
export const mslToAgl = (msl, groundMsl) => msl - groundMsl;

/** Downtown Pittsburgh (the Point) — origin of the simulator map. */
export const PITTSBURGH = { lat: 40.4406, lon: -79.9959 };
