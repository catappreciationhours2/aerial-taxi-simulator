// Headless benchmark: ablation of planner presets A-D across seeds + scalability sweep.
//   npm run bench          (full: ~10-20 min)       npm run bench:quick  (~2 min)
import { Engine } from '../src/sim/engine.js';
import { PRESETS } from '../src/core/config.js';
import { writeFileSync, mkdirSync } from 'node:fs';

const quick = process.argv.includes('--quick');
const SEEDS = quick ? [1, 2] : [1, 2, 3, 4, 5];
const FLEET = quick ? 40 : 80, T = quick ? 420 : 900;
const mean = (a) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
const fmt = (x, d = 1) => (Number.isFinite(x) ? x.toFixed(d) : '-');

function run(preset, seed, fleet, T, extra = {}) {
  const e = new Engine({ fleetSize: fleet, seed, features: PRESETS[preset].features, demand: extra.demand ?? 2, wind: { speed10: extra.wind ?? 5, dirFrom: 250, gust: 1.3 } });
  e.setFeature('stadiumTfr', true);
  const t0 = performance.now();
  e.advance(T);
  const s = e.summary();
  return { s, wall: (performance.now() - t0) / 1000, e };
}

const out = ['# Swagmo benchmark', '', `Fleet ${FLEET}, ${T} s simulated, seeds ${SEEDS.join(',')}, wind 5 m/s (10 m) with 1.3x gusts, stadium TFR active, demand 2x.`, ''];
out.push('## Ablation (means over seeds)', '', '| preset | completed | loss of sep. | collisions | crashes | emergencies | delay ratio | energy/trip kWh | plan ms p95 |', '|---|---|---|---|---|---|---|---|---|');
for (const p of Object.keys(PRESETS)) {
  const rs = SEEDS.map((sd) => run(p, sd, FLEET, T).s);
  out.push(`| ${PRESETS[p].label} | ${fmt(mean(rs.map((r) => r.completed)))} | ${fmt(mean(rs.map((r) => r.lossOfSeparation)))} | ${fmt(mean(rs.map((r) => r.collisions)))} | ${fmt(mean(rs.map((r) => r.crashes)))} | ${fmt(mean(rs.map((r) => r.emergencyLandings)))} | ${fmt(mean(rs.map((r) => r.delayRatioMean)), 2)} | ${fmt(mean(rs.map((r) => r.energyPerTripMean)), 1)} | ${fmt(mean(rs.map((r) => r.planMsP95)), 0)} |`);
  console.error('ablation', p, 'done');
}
out.push('', '## Scalability (preset D)', '', '| fleet | airborne (peak) | completed | loss of sep. | crashes | wall s per sim-min | plan ms p95 |', '|---|---|---|---|---|---|---|');
for (const n of quick ? [20, 100] : [20, 50, 100, 200, 350, 500]) {
  const Tn = quick ? 240 : 420;
  const { s, wall, e } = run('D', 1, n, Tn, { demand: 3 });
  out.push(`| ${n} | ${Math.max(...e.metrics.series.airborne, 0)} | ${s.completed} | ${s.lossOfSeparation} | ${s.crashes} | ${fmt((wall / Tn) * 60, 1)} | ${fmt(s.planMsP95, 0)} |`);
  console.error('scale', n, 'done');
}
const text = out.join('\n') + '\n';
console.log(text);
mkdirSync(new URL('../results/', import.meta.url), { recursive: true });
writeFileSync(new URL('../results/bench' + (quick ? '-quick' : '') + '.md', import.meta.url), text);
