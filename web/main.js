// Web UI wiring: simulation loop (time-budgeted), controls, fault buttons, event log, selected-aircraft card.
import { Engine } from '../src/sim/engine.js';
import { World } from '../src/world/world.js';
import { FEATURE_LABELS, PRESETS, SIM, WORLD } from '../src/core/config.js';
import { FAULT_TYPES } from '../src/sim/faults.js';
import { OPEN_METEO_URL, parseOpenMeteo } from '../src/data/loaders.js';
import { Renderer, VIEWS, drawSide, socColor } from './render.js';
import { Tutorial } from './tutorial.js';

const $ = (id) => document.getElementById(id);
const W = 360, H = 240;
const mapCv = $('map'), sideCv = $('side'), sparkCv = $('spark');
const ctx = mapCv.getContext('2d'), sctx = sideCv.getContext('2d'), kctx = sparkCv.getContext('2d');
for (const c of [ctx, sctx]) c.imageSmoothingEnabled = false;

const state = { view: 'traffic', paused: false, speed: 2, selectedId: null, engine: null, renderer: null, world: null, simAccum: 0, lastFrame: performance.now(), lastUi: 0, achieved: 1, params: { fleet: 60, demand: 1, seed: 7, wind: 3.5, dir: 250, gust: 1 }, features: null };

// ------------------------------------------------------------------ world (optionally with real OSM buildings if data/buildings.geojson exists)
async function loadWorld() {
  const world = new World();
  try {
    const r = await fetch('../data/buildings.geojson', { cache: 'no-store' });
    if (r.ok) { world.applyBuildingsGeoJSON(await r.json()); console.info('Loaded real building footprints from data/buildings.geojson'); }
  } catch { /* offline / not present: synthetic world */ }
  return world;
}

function newEngine(extra = {}) {
  const p = state.params;
  const eng = new Engine({ fleetSize: p.fleet, seed: p.seed, demand: p.demand, world: state.world, wind: { speed10: p.wind, dirFrom: p.dir, gust: p.gust }, features: state.features || undefined, planBudgetMs: 14, ...extra });
  state.features = eng.features;
  state.engine = eng;
  eng.events.on(onEvent);
  if (!state.renderer) state.renderer = new Renderer(eng, { W, H, createImageData: (w, h) => ctx.createImageData(w, h) });
  else state.renderer.setEngine(eng);
  state.selectedId = null; state.simAccum = 0;
  $('log').innerHTML = ''; renderToggles(); updateCard(true); syncStressUi();
}

// ------------------------------------------------------------------ controls
function buildViews() {
  const box = $('views');
  box.innerHTML = '';
  VIEWS.forEach((v, i) => {
    const b = document.createElement('button');
    b.innerHTML = `<span class="ic">${v.icon}</span>${v.label} <span class="dim small" style="margin-left:auto">${i + 1}</span>`;
    b.onclick = () => setView(v.id);
    b.dataset.id = v.id; box.appendChild(b);
  });
}
function setView(id) {
  state.view = id;
  for (const b of $('views').children) b.classList.toggle('on', b.dataset.id === id);
  $('viewhint').textContent = VIEWS.find((v) => v.id === id).hint;
}

function renderToggles() {
  const box = $('toggles');
  box.innerHTML = '';
  for (const [k, label] of Object.entries(FEATURE_LABELS)) {
    const l = document.createElement('label');
    const c = document.createElement('input'); c.type = 'checkbox'; c.checked = !!state.engine.features[k]; c.dataset.k = k;
    c.onchange = () => state.engine.setFeature(k, c.checked);
    l.dataset.k = k;
    l.append(c, document.createTextNode(label)); box.appendChild(l);
  }
}
function syncToggles() { for (const c of $('toggles').querySelectorAll('input')) c.checked = !!state.engine.features[c.dataset.k]; }

function buildFaults() {
  const box = $('faults');
  for (const [k, f] of Object.entries(FAULT_TYPES)) {
    const b = document.createElement('button');
    b.textContent = f.label; b.title = f.desc; b.dataset.fault = k;
    b.onclick = () => {
      const r = state.engine.injectFault(k);
      if (!r.ok) toast(r.reason || 'could not inject');
    };
    box.appendChild(b);
  }
}
let toastT = 0;
function toast(msg) { const t = $('toast'); t.textContent = msg; t.style.display = 'block'; clearTimeout(toastT); toastT = setTimeout(() => { t.style.display = 'none'; }, 2600); }

function bindSliders() {
  const s = state.params;
  $('rFleet').oninput = (e) => { s.fleet = +e.target.value; $('vFleet').textContent = s.fleet; };
  $('rDemand').oninput = (e) => { s.demand = +e.target.value; $('vDemand').textContent = s.demand.toFixed(2).replace(/0+$/, '').replace(/\.$/, '') + '×'; state.engine.cfg.demand = s.demand; };
  $('inSeed').onchange = (e) => { s.seed = Math.max(1, +e.target.value | 0); };
  const wind = () => { state.engine.setWind({ speed10: s.wind, dirFrom: s.dir, gust: s.gust }); $('vWind').textContent = s.wind.toFixed(1) + ' m/s'; $('vDir').textContent = s.dir + '°'; $('vGust').textContent = s.gust.toFixed(1) + '×'; };
  $('rWind').oninput = (e) => { s.wind = +e.target.value; wind(); };
  $('rDir').oninput = (e) => { s.dir = +e.target.value; wind(); };
  $('rGust').oninput = (e) => { s.gust = +e.target.value; wind(); };
  $('selSpeed').onchange = (e) => { state.speed = +e.target.value; };
  $('selPreset').onchange = (e) => {
    const f = PRESETS[e.target.value].features;
    for (const [k, v] of Object.entries(f)) state.engine.setFeature(k, v);
    syncToggles();
    state.engine.log('info', `Preset ${PRESETS[e.target.value].label} applied`);
  };
  $('btnPause').onclick = togglePause;
  $('btnStep').onclick = () => { state.engine.advance(1); };
  $('btnRestart').onclick = () => { newEngine(); };
  $('btnStress').onclick = toggleStress;
  $('btnLive').onclick = async () => {
    $('btnLive').textContent = '… loading';
    try {
      const r = await fetch(OPEN_METEO_URL(WORLD.lat0, WORLD.lon0));
      const p = parseOpenMeteo(await r.json());
      if (!p) throw new Error('unexpected response');
      Object.assign(s, { wind: Math.min(14, p.speed10), dir: Math.round(p.dirFrom / 5) * 5 % 360, gust: Math.min(3, p.gust) });
      $('rWind').value = s.wind; $('rDir').value = s.dir; $('rGust').value = s.gust; wind();
      state.engine.log('info', `Live Pittsburgh wind loaded: ${p.speed10.toFixed(1)} m/s from ${p.dirFrom.toFixed(0)}°`);
    } catch (e) { toast('Could not fetch live wind (offline?)'); }
    $('btnLive').textContent = '☁ load live Pittsburgh wind';
  };
  $('btnDivert').onclick = () => { const a = sel(); if (a && a.airborne) state.engine.divertAircraft(a, 'operator request'); };
  $('btnEmerg').onclick = () => { const a = sel(); if (a && a.airborne) state.engine.enterEmergency(a, 'operator request'); };
  $('btnDesel').onclick = () => { state.selectedId = null; state.engine.selectedId = null; updateCard(true); };
  window.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    if (e.code === 'Space') { e.preventDefault(); togglePause(); }
    else if (e.key === 'n') state.engine.advance(1);
    else if (e.key >= '1' && e.key <= String(VIEWS.length)) setView(VIEWS[+e.key - 1].id);
  });
  mapCv.addEventListener('click', (e) => {
    const r = mapCv.getBoundingClientRect();
    const id = state.renderer.pick(((e.clientX - r.left) / r.width) * W, ((e.clientY - r.top) / r.height) * H, 10);
    state.selectedId = id; state.engine.selectedId = id; updateCard(true);
  });
}
// ------------------------------------------------------------------ stress mode: every taxi in the fleet requests a ride at once, continuously
function syncStressUi() {
  const on = !!(state.engine && state.engine.stress);
  $('btnStress').textContent = on ? '⏹ end stress mode' : '💥 Stress: launch all';
  $('btnStress').classList.toggle('on', on);
}
function toggleStress() {
  if (state.engine.stress) { state.engine.stress = false; state.engine.log('info', 'Stress mode ended — demand back to the slider setting'); syncStressUi(); return; }
  const parallel = $('chkParallel').checked;
  newEngine({ stress: true, fatos: parallel ? 3 : 1 });
  state.engine.log('warn', `STRESS MODE: all ${state.params.fleet} taxis request rides now${parallel ? ' (parallel take-offs: UNSAFE, expect conflicts)' : ' (one take-off at a time per vertiport)'}`);
  toast(`Stress: launching ${state.params.fleet} taxis`);
}
/** Push state.params into the sliders/labels (used by the tutorial to set up and restore the scenario). */
function applyParamsToUi() {
  const p = state.params;
  $('rFleet').value = p.fleet; $('vFleet').textContent = p.fleet;
  $('rDemand').value = p.demand; $('vDemand').textContent = p.demand.toFixed(2).replace(/0+$/, '').replace(/\.$/, '') + '×';
  $('rWind').value = p.wind; $('vWind').textContent = p.wind.toFixed(1) + ' m/s';
  $('rDir').value = p.dir; $('vDir').textContent = p.dir + '°';
  $('rGust').value = p.gust; $('vGust').textContent = p.gust.toFixed(1) + '×';
  $('inSeed').value = p.seed; $('selSpeed').value = String(state.speed); $('selPreset').value = 'D';
  $('btnPause').textContent = state.paused ? '▶' : '⏸';
}
function togglePause() { state.paused = !state.paused; $('btnPause').textContent = state.paused ? '▶' : '⏸'; }
const sel = () => (state.selectedId === null ? null : state.engine.aircraft[state.selectedId]);

// ------------------------------------------------------------------ event log
function fmtT(t) { const m = Math.floor(t / 60), s = Math.floor(t % 60); return `${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`; }
function onEvent(e) {
  if (e.level === 'info' && /replanned/.test(e.text)) return; // keep the log readable
  const box = $('log');
  const d = document.createElement('div'); d.className = e.level;
  d.innerHTML = `<span class="t">${fmtT(e.t)}</span>`;
  d.append(document.createTextNode(e.text));
  box.appendChild(d);
  while (box.children.length > 120) box.removeChild(box.firstChild);
  box.scrollTop = box.scrollHeight;
}

// ------------------------------------------------------------------ side panels
function kpi(label, value, cls = '') { return `<div class="kpi ${cls}"><b>${value}</b><span>${label}</span></div>`; }
function updateKpis() {
  const e = state.engine, m = e.metrics, s = m.series;
  const air = e.airborneCount(), done = m.completed;
  const wind = e.truth.at(3600, 2400, 380, e.t);
  $('kpis').innerHTML = kpi('sim time', fmtT(e.t)) + kpi('airborne', `${air}/${e.aircraft.length}`) + kpi('trips done', done, 'good') + kpi('requests', m.requested)
    + kpi('loss of sep.', m.los, m.los ? 'bad' : '') + kpi('crashes', m.crashes + m.collisions, m.crashes + m.collisions ? 'bad' : '') + kpi('emergencies', m.emergencyLandings) + (e.stress ? kpi('mode', 'STRESS', 'bad') : '')
    + kpi('wind@380m', Math.hypot(wind[0], wind[1]).toFixed(1) + ' m/s') + kpi('sim speed', '×' + state.achieved.toFixed(1), state.achieved < state.speed * 0.9 ? 'bad' : '');
  void s;
}
function updateFaultList() {
  const recs = state.engine.faultRecords.slice(-8).reverse();
  $('faultlist').innerHTML = recs.length ? recs.map((r) => {
    const det = r.tDetect === null ? '—' : (r.tDetect - r.tInject).toFixed(1) + ' s', res = r.tResolve === null ? '—' : (r.tResolve - r.tInject).toFixed(0) + ' s';
    return `<div><span class="badge b-${r.status}">${r.status}</span><b>${r.label}</b> · ${r.acName}<br><span class="dim">detect ${det} · resolve ${res} ${r.outcome ? '· ' + r.outcome : ''}</span></div>`;
  }).join('') : '<span class="dim">No failures injected yet. Use the buttons on the left.</span>';
}
function updateSpark() {
  const s = state.engine.metrics.series, n = s.t.length;
  kctx.fillStyle = '#080c1e'; kctx.fillRect(0, 0, 240, 44);
  if (n < 2) return;
  const draw = (arr, color, max) => { kctx.strokeStyle = color; kctx.beginPath(); for (let i = 0; i < n; i++) { const x = (i / (n - 1)) * 238 + 1, y = 42 - (arr[i] / Math.max(1, max)) * 38; i ? kctx.lineTo(x, y) : kctx.moveTo(x, y); } kctx.stroke(); };
  draw(s.airborne, '#a8d8ff', Math.max(...s.airborne, 5)); draw(s.completed, '#b6f0d0', Math.max(...s.completed, 5)); draw(s.los, '#ff7a9c', Math.max(...s.los, 3));
  $('sparkLegend').innerHTML = '<span style="color:#a8d8ff">■</span> airborne <span style="color:#b6f0d0">■</span> completed <span style="color:#ff7a9c">■</span> separation losses';
}
let cardKey = '';
function updateCard(force = false) {
  const a = sel(), card = $('card');
  $('cardBtns').hidden = !a;
  $('targetHint').textContent = a ? `Target: ${a.name} (selected).` : 'Target: click a car to select it, otherwise a random airborne one.';
  if (!a) { if (force || cardKey !== 'none') { card.className = 'card dim'; card.textContent = 'Click a taxi on the map.'; cardKey = 'none'; } return; }
  cardKey = 'sel';
  const e = state.engine, v = Math.hypot(a.v[0], a.v[1]), plan = a.activePlan && a.activePlan();
  const eta = plan && !plan.empty ? Math.max(0, plan.t1 - e.t) : 0;
  const dest = a.destId ? (e.world.siteById(a.destId) || {}).name || a.destId : '—';
  const flags = [a.health.commLoss && 'link lost', a.health.gps !== 'ok' && 'GNSS ' + a.health.gps, a.health.thrustFactor < 1 && 'thrust ' + (a.health.thrustFactor * 100).toFixed(0) + '%', a.health.drainMult > 1 && 'battery fault', a.health.sensor !== 'ok' && 'sensor ' + a.health.sensor, a.frozen && 'plan frozen'].filter(Boolean);
  card.className = 'card';
  card.innerHTML = `<b>${a.name}</b> · ${a.spec.label || a.spec.key}<br>
    state <b>${a.phase}${a.emergencyMode ? ' · EMERGENCY' : ''}</b> · mode ${a.mode}<br>
    battery ${(a.soc * 100).toFixed(0)}% (${a.energy.toFixed(1)}/${a.spec.usableKwh.toFixed(0)} kWh)<div class="bar"><i style="width:${(a.soc * 100).toFixed(0)}%;background:${socColor(a.soc)}"></i></div>
    power ${a.powerKw.toFixed(0)} kW · ${v.toFixed(0)} m/s<br>
    alt ${a.p[2].toFixed(0)} m MSL<br>
    route ${a.siteId} → ${dest}<br>
    ETA ${eta.toFixed(0)} s · track err ${a.trackErr.toFixed(0)} m<br>
    CBF ${a.cbfActive ? '<b style="color:#ff7a9c">active</b>' : 'idle'} · nav σ ${a.nav.sigma.toFixed(1)} m<br>
    ${flags.length ? '<span style="color:#ffcf70">⚠ ' + flags.join(', ') + '</span>' : '<span class="dim">no faults</span>'}`;
}

// ------------------------------------------------------------------ main loop
function frame(now) {
  const dtReal = Math.min(0.1, (now - state.lastFrame) / 1000); state.lastFrame = now;
  const e = state.engine;
  if (!state.paused) {
    state.simAccum += dtReal * state.speed;
    const t0 = performance.now(), budget = 11, startT = e.t;
    while (state.simAccum >= SIM.dt && performance.now() - t0 < budget) { e.step(); state.simAccum -= SIM.dt; }
    if (state.simAccum >= SIM.dt) state.simAccum = 0; // fell behind: drop the backlog instead of spiralling
    const adv = e.t - startT;
    state.achieved = state.achieved * 0.9 + 0.1 * (dtReal > 0 ? adv / dtReal : 0);
  }
  state.renderer.draw(ctx, { view: state.view, selectedId: state.selectedId });
  drawSide(sctx, e, W, 64, state.selectedId);
  if (now - state.lastUi > 250) { state.lastUi = now; updateKpis(); updateCard(); updateFaultList(); updateSpark(); }
  requestAnimationFrame(frame);
}

// ------------------------------------------------------------------ tutorial
const tutorial = new Tutorial({
  $, state, setView,
  setPaused(v) { state.paused = !!v; $('btnPause').textContent = state.paused ? '▶' : '⏸'; },
  begin() { // known, lively scenario: 40 taxis, 3x demand, pre-run 150 s so some are already in the air
    tutorial.saved = { params: { ...state.params }, speed: state.speed, view: state.view };
    Object.assign(state.params, { fleet: 40, demand: 3, seed: 7, wind: 3.5, dir: 250, gust: 1 });
    state.features = null; state.paused = false; state.speed = 2; setView('traffic'); applyParamsToUi();
    newEngine(); state.engine.traffic.rate = 1 / 20; state.engine.advance(150); // more helicopters/planes so external traffic is easy to see
  },
  end() { // back to a fresh run with the user's own settings
    const sv = tutorial.saved; if (!sv) return;
    Object.assign(state.params, sv.params); state.speed = sv.speed; state.features = null; state.paused = false;
    applyParamsToUi(); setView('traffic'); newEngine();
    state.engine.log('info', 'Tutorial finished — replay it any time from the ? Tutorial button.');
  },
});

(async function init() {
  buildViews(); buildFaults(); setView('traffic');
  state.world = await loadWorld();
  const q = new URLSearchParams(location.search);
  newEngine();
  bindSliders();
  if (q.has('warm')) state.engine.advance(Math.min(900, +q.get('warm') || 0)); // ?warm=SECONDS pre-runs the sim (demos/screenshots)
  if (q.has('view') && VIEWS.some((v) => v.id === q.get('view'))) setView(q.get('view'));
  if (q.has('pause')) togglePause();
  state.engine.log('info', 'Swagmo ready — click a taxi to select it, then inject a failure from the left panel.');
  $('btnTutorial').onclick = () => tutorial.start();
  if (!Tutorial.seen() && !q.has('warm') && !q.has('pause') && !q.has('notutorial')) setTimeout(() => tutorial.start(), 400); // first visit
  requestAnimationFrame(frame);
  window.swagmo = state; // handy for debugging in the console
})();
