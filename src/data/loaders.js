// Optional real-data loaders. The simulator runs entirely offline on its synthetic world; these helpers let you
// plug live/real data in. They are pure parsers (no network inside) so they can be unit-tested offline.

import { ExtAircraft } from '../sim/traffic.js';

export const OPEN_METEO_URL = (lat, lon) =>
  `https://api.open-meteo.com/v1/forecast?latitude=${lat}&longitude=${lon}&current=wind_speed_10m,wind_direction_10m,wind_gusts_10m&wind_speed_unit=ms`;

/**
 * Parse an Open-Meteo "current" response (https://open-meteo.com/en/docs) into simulator wind settings.
 * @returns {{speed10:number, dirFrom:number, gust:number}|null}
 */
export function parseOpenMeteo(json) {
  const c = json && json.current;
  if (!c || !Number.isFinite(c.wind_speed_10m) || !Number.isFinite(c.wind_direction_10m)) return null;
  const speed10 = Math.max(0, c.wind_speed_10m);
  const gustRatio = Number.isFinite(c.wind_gusts_10m) && speed10 > 0.5 ? Math.min(3, Math.max(0.5, c.wind_gusts_10m / (speed10 * 1.5))) : 1;
  return { speed10, dirFrom: ((c.wind_direction_10m % 360) + 360) % 360, gust: gustRatio };
}

/**
 * Parse an OpenSky Network /states/all response (https://openskynetwork.github.io/opensky-api/rest.html) into external
 * traffic tracks inside the simulator's local frame. Aircraft outside the map or on the ground are dropped.
 * State vector indices: 0 icao24, 5 lon, 6 lat, 7 baro_altitude, 8 on_ground, 9 velocity, 10 true_track, 11 vertical_rate.
 */
export function parseOpenSky(json, world, baseMsl = 0) {
  const out = [];
  for (const s of (json && json.states) || []) {
    const [icao, , , , , lon, lat, alt, onGround, vel, track, vr] = [s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8], s[9], s[10], s[11]];
    if (onGround || lon == null || lat == null || alt == null) continue;
    const e = world.frame.toENU(lat, lon, alt);
    const x = e[0] + world.width / 2, y = e[1] + world.height / 2;
    if (!world.inBounds(x, y)) continue;
    const th = ((track || 0) * Math.PI) / 180, sp = vel || 0;
    out.push(new ExtAircraft(`OS-${icao}`, [x, y, alt + baseMsl], [sp * Math.sin(th), sp * Math.cos(th), vr || 0], 'helicopter'));
  }
  return out;
}

/** Overpass QL for building footprints in a lat/lon box (used by scripts/fetch-osm.js). */
export const overpassQuery = (south, west, north, east) =>
  `[out:json][timeout:90];(way["building"](${south},${west},${north},${east}););out geom;`;

/** Convert an Overpass JSON response (out geom) to a GeoJSON FeatureCollection of building polygons. */
export function overpassToGeoJSON(json) {
  const features = [];
  for (const el of (json && json.elements) || []) {
    if (el.type !== 'way' || !el.geometry || el.geometry.length < 4) continue;
    const ring = el.geometry.map((g) => [+g.lon.toFixed(5), +g.lat.toFixed(5)]); // ~1 m precision keeps the file small enough to commit
    const t = el.tags || {}, properties = {};
    for (const k of ['height', 'building:levels', 'building']) if (t[k] !== undefined) properties[k] = t[k]; // only the tags the simulator reads
    features.push({ type: 'Feature', properties, geometry: { type: 'Polygon', coordinates: [ring] } });
  }
  return { type: 'FeatureCollection', features };
}
