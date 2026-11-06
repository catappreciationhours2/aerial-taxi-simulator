// Optional: download real Pittsburgh building footprints (OpenStreetMap via Overpass) to data/buildings.geojson.
// The web UI loads that file automatically if it exists. Needs network access; the simulator works without it.
//   node scripts/fetch-osm.js
import { writeFileSync, mkdirSync } from 'node:fs';
import { WORLD } from '../src/core/config.js';
import { overpassQuery, overpassToGeoJSON } from '../src/data/loaders.js';

const dLat = (WORLD.ny * WORLD.cell) / 2 / 111320, dLon = (WORLD.nx * WORLD.cell) / 2 / (111320 * Math.cos((WORLD.lat0 * Math.PI) / 180));
const q = overpassQuery(WORLD.lat0 - dLat, WORLD.lon0 - dLon, WORLD.lat0 + dLat, WORLD.lon0 + dLon);
console.log('Querying Overpass for building footprints...');
// Overpass rejects clients with no User-Agent / Accept header (HTTP 406), so identify ourselves; try a mirror if one is down.
const ENDPOINTS = ['https://overpass-api.de/api/interpreter', 'https://overpass.kumi.systems/api/interpreter'];
let res;
for (const url of ENDPOINTS) {
  res = await fetch(url, { method: 'POST', body: 'data=' + encodeURIComponent(q), headers: { 'Content-Type': 'application/x-www-form-urlencoded', Accept: 'application/json', 'User-Agent': 'swagmo-simulator/1.0 (open-source research simulator)' } });
  if (res.ok) break;
  console.error('Overpass error', res.status, 'from', url);
}
if (!res.ok) process.exit(1);
const gj = overpassToGeoJSON(await res.json());
mkdirSync(new URL('../data/', import.meta.url), { recursive: true });
writeFileSync(new URL('../data/buildings.geojson', import.meta.url), JSON.stringify(gj));
console.log(`Saved ${gj.features.length} buildings to data/buildings.geojson (reload the web UI to use them).`);
