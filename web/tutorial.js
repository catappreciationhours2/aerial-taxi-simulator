// Guided tutorial: a spotlight overlay (everything except the focused control is dimmed), bouncing arrows that say
// "do this" / "watch here", and an input gate that only lets the user perform the action the current step asks for.
//
// Steps are plain data (STEPS) so they can be tested without a browser. Each step may have:
//   target  CSS selector (or fn(api, base) -> {left,top,width,height} | null): the thing to act on  -> "do this" arrow
//   see     array of CSS selectors: where the effect shows up                                       -> "watch here" arrows
//   allow   array of CSS selectors the user may interact with (clicks / keys / slider drags) during this step
//   keys    array of KeyboardEvent.code values allowed during this step
//   enter   fn(api) -> base: runs when the step starts (set up the sim state), returns a baseline for `ready`
//   ready   fn(api, base) -> bool: the user did the expected thing; the Next button unlocks (steps without `ready` are read-only)

import { S } from './render.js';

export const STORAGE_KEY = 'swagmo.tutorial.v1';
const W = 360, H = 240; // map canvas logical size

/** Screen rect of an airborne taxi (followed every frame) so the arrow can point at a concrete car to click. */
function taxiRect(api, base) {
  const e = api.state.engine;
  let ac = base && base.taxiId != null ? e.aircraft[base.taxiId] : null;
  if (!ac || !ac.airborne) {
    ac = e.aircraft.filter((a) => a.airborne && !a.emergencyMode).sort((a, b) => Math.hypot(a.p[0] - 3600, a.p[1] - 2400) - Math.hypot(b.p[0] - 3600, b.p[1] - 2400))[0];
    if (!ac) return null;
    if (base) base.taxiId = ac.id;
  }
  const r = api.$('map').getBoundingClientRect();
  const cx = r.left + ((ac.p[0] / S) / W) * r.width, cy = r.top + ((H - 1 - ac.p[1] / S) / H) * r.height;
  return { left: cx - 14, top: cy - 14, width: 28, height: 28 };
}

const deselect = (a) => { a.state.selectedId = null; a.state.engine.selectedId = null; };
const toggleSel = (k) => `#toggles label[data-k="${k}"]`;

export const STEPS = [
  { id: 'welcome', modal: true, title: 'Welcome to Swagmo', text: 'This is a live simulator of autonomous aerial taxis over Pittsburgh. In a few minutes you will pause it, change views, select a taxi, break things on purpose and switch algorithm features off to see what each one is worth. The tutorial only accepts the highlighted action, and you can skip it at any time.' },
  { id: 'map', title: 'The map', target: '#stage', text: 'Each little car is an aerial taxi; dotted lines are their planned routes. Blue squares are vertiports (take-off and landing sites), green crosses are emergency landing sites, and the red circles are restricted airspace. Taxis fly between vertiports to carry passengers.' },
  { id: 'side', title: 'Altitude strip', target: '#side', text: 'A side view of the same scene: left-right is the map\'s east-west axis, up is altitude. The dark silhouette is terrain and buildings, the faint lines are the four flight levels, and each square is an airborne taxi.' },
  { id: 'kpis', title: 'Live numbers', target: '#kpis', text: 'Sim time, taxis airborne out of the whole fleet, trips completed, ride requests, loss-of-separation events (two taxis too close), crashes, emergency landings, wind, and how fast the simulation is actually running.' },
  { id: 'pause', title: 'Pause the simulation', target: '#btnPause', allow: ['#btnPause'], keys: ['Space'], see: ['#kpis'], text: 'Click the pause button (or press Space). Watch the sim time stop.', ready: (a) => a.state.paused },
  { id: 'resume', title: 'Resume', target: '#btnPause', allow: ['#btnPause'], keys: ['Space'], text: 'Click it again to resume. While paused you can inspect anything in peace.', ready: (a) => !a.state.paused },
  { id: 'speed', title: 'Change the simulation speed', target: '#selSpeed', allow: ['#selSpeed'], see: ['#kpis'], enter: (a) => ({ speed: a.state.speed }), text: 'Pick a different speed. At large fleet sizes the computer may not keep up, so the "sim speed" tile will turn red.', ready: (a, b) => a.state.speed !== b.speed },
  { id: 'view-battery', title: 'Battery view', enter: (a) => { a.setView('traffic'); }, target: '#views button[data-id="battery"]', allow: ['#views button[data-id="battery"]'], keys: ['Digit2'], see: ['#stage'], text: 'The buttons on the right switch what the map shows. Click Battery: every taxi is now coloured by its charge.', ready: (a) => a.state.view === 'battery' },
  { id: 'battery-read', title: 'Reading a taxi\'s battery', target: taxiRect, enter: () => ({ taxiId: null }), see: ['#viewhint'], text: 'This highlighted taxi shows the colour code: green is well charged, yellow is middling, orange and pink are low. Taxis with low charge divert to charge or land at the nearest safe site.' },
  { id: 'view-wind', title: 'Wind view', target: '#views button[data-id="wind"]', allow: ['#views button[data-id="wind"]'], keys: ['Digit3'], see: ['#stage'], text: 'Now click Wind. The flowing currents are the true wind at flight altitude. Faster wind is shown in warmer colours, and gust hazards in pink.', ready: (a) => a.state.view === 'wind' },
  { id: 'view-others', title: 'More views', enter: (a) => { a.setView('traffic'); }, target: '#views', text: 'The other views: Airspace (restricted zones and external traffic), Safety (protected bubbles and close pairs), Noise (the population map the planner avoids) and Wind map (how wrong the planner\'s wind estimate is). Keys 1-7 switch views once the tutorial is over.' },
  { id: 'select', title: 'Select a taxi', target: taxiRect, allow: ['#map'], see: ['#card'], enter: (a) => { a.setPaused(true); deselect(a); return { taxiId: null }; }, text: 'I paused the simulation so the taxi holds still (you can always do this to pick a taxi in a busy scene). Click the highlighted taxi on the map and its details appear in the card on the right.', ready: (a) => a.state.selectedId !== null },
  { id: 'card', title: 'The selected-taxi card', target: '#card', see: ['#cardBtns'], text: 'Battery, power, speed, altitude, route and ETA, plus whether the safety filter (CBF) is acting. Below it, "divert" sends the taxi to another vertiport and "emergency land" makes it land at the nearest emergency site.' },
  { id: 'resume-after', title: 'Resume the simulation', target: '#btnPause', allow: ['#btnPause'], keys: ['Space'], see: ['#card'], text: 'Un-pause. The card follows the selected taxi as it flies; its battery bar and ETA update live.', ready: (a) => !a.state.paused },
  { id: 'fault', title: 'Inject a failure', target: '#faults button[data-fault="gps_bias"]', allow: ['#faults button[data-fault="gps_bias"]'], see: ['#log', '#faultwrap'], enter: (a) => ({ n: a.state.engine.faultRecords.length }), text: 'Click "GPS bias / spoof". It corrupts the selected taxi\'s position fix by 60 m. Watch the event log and the failure records.', ready: (a, b) => a.state.engine.faultRecords.length > b.n },
  { id: 'watch', title: 'Watch it get handled', target: '#faultwrap', see: ['#log', '#card'], text: 'Each failure is tracked through injected, detected, responding and resolved, with the detection delay. For GPS bias the navigation filter notices the disagreement within seconds, isolates GPS, widens the safety margin and diverts the taxi. The other eleven failures work the same way; try them after the tutorial.' },
  { id: 'features', title: 'Feature switches', target: '#toggles', text: 'Every algorithm component has a switch, and they can be flipped while the simulation runs. Switching one off shows what it is worth: reservations (taxis plan around each other), wind-aware planning, energy-aware speed, the fleet wind map, adaptive safety margins, MPC tracking, the CBF safety filter, emergency reachability, fault response, cluster repair, noise-aware routing, gusts, external traffic and the stadium restriction. Some effects are obvious on the map; others show up in the numbers. Let us try the clearest ones.' },
  { id: 'ext-off', title: 'External air traffic: off', enter: (a) => { a.setView('traffic'); return {}; }, target: toggleSel('externalTraffic'), allow: [toggleSel('externalTraffic')], see: ['#stage'], text: 'The orange dots with a streak are helicopters and small planes that are not part of the fleet. Untick "External air traffic" and watch them vanish from the map.', ready: (a) => !a.state.engine.features.externalTraffic },
  { id: 'ext-on', title: 'External air traffic: on', target: toggleSel('externalTraffic'), allow: [toggleSel('externalTraffic')], see: ['#stage'], text: 'Tick it again. New tracks cross in from the map edges. Taxis must route around them, which is why this is part of the airspace problem.', ready: (a) => a.state.engine.features.externalTraffic },
  { id: 'resp-off', title: 'Fault response: off', enter: (a) => { a.setView('traffic'); return {}; }, target: toggleSel('faultResponse'), allow: [toggleSel('faultResponse')], see: ['#log'], text: 'Now the opposite experiment. Untick "Fault detection + response": failures will still happen but nothing will detect or handle them.', ready: (a) => !a.state.engine.features.faultResponse },
  { id: 'resp-inject', title: 'Break a taxi with no safety net', enter: (a) => { deselect(a); return { n: a.state.engine.faultRecords.length }; }, target: '#faults button[data-fault="gps_bias"]', allow: ['#faults button[data-fault="gps_bias"]'], see: ['#log', '#faultwrap'], text: 'Inject GPS bias again (on a random airborne taxi this time). Compare with before: no detection, no isolation, no diversion.', ready: (a, b) => a.state.engine.faultRecords.length > b.n },
  { id: 'resp-see', title: 'Nothing happens', target: '#faultwrap', see: ['#log'], text: 'The failure record shows no detection time and no response, and if the fault later ends or the taxi lands it is marked "unhandled" rather than "resolved". That gap is exactly what the detection and response layer closes.' },
  { id: 'resp-on', title: 'Fault response: back on', target: toggleSel('faultResponse'), allow: [toggleSel('faultResponse')], text: 'Tick it again to restore protection.', ready: (a) => a.state.engine.features.faultResponse },
  { id: 'res-off', title: 'Reservations: off', target: toggleSel('reservations'), allow: [toggleSel('reservations')], see: ['#kpis', '#stage'], text: 'Untick "Multi-agent reservations". Taxis now plan as if alone, like running plain A* for each one. Over a few minutes the "loss of sep." tile counts the near misses this causes, especially near busy vertiports. This is the core idea of the planner.', ready: (a) => !a.state.engine.features.reservations },
  { id: 'res-on', title: 'Reservations: back on', target: toggleSel('reservations'), allow: [toggleSel('reservations')], text: 'Tick it again so taxis book space and time before they fly.', ready: (a) => a.state.engine.features.reservations },
  { id: 'toggle', title: 'Stadium restriction', enter: (a) => { a.setView('airspace'); return {}; }, target: toggleSel('stadiumTfr'), allow: [toggleSel('stadiumTfr')], see: ['#stage'], text: 'Tick "Stadium TFR": a temporary no-fly zone appears over the stadium, and taxis re-route around it.', ready: (a) => a.state.engine.features.stadiumTfr },
  { id: 'wind', title: 'Change the weather', enter: (a) => { a.setView('wind'); return { w: a.state.params.wind }; }, target: '#rWind', allow: ['#rWind'], see: ['#stage', '#kpis'], text: 'Drag the wind slider. The currents on the map and the wind tile change immediately, and the planner has to cope with stronger headwinds and gusts.', ready: (a, b) => a.state.params.wind !== b.w },
  { id: 'demand', title: 'Change ride demand', enter: (a) => { a.setView('traffic'); return { d: a.state.params.demand }; }, target: '#rDemand', allow: ['#rDemand'], see: ['#kpis'], text: 'Demand sets how often passengers request rides, and so how many taxis are in the air. Raise it and watch "airborne" climb.', ready: (a, b) => a.state.params.demand !== b.d },
  { id: 'fleet', title: 'Fleet size', target: '#rFleet', see: ['#btnRestart'], text: 'Fleet size is how many taxis exist (up to 500). It takes effect when you press restart. Not all of them fly at once: only taxis with a ride are airborne.' },
  { id: 'stress', title: 'Stress mode', target: '#btnStress', allow: ['#btnStress'], see: ['#kpis', '#stage'], text: 'Stress mode restarts with every taxi in the fleet requesting a ride immediately and keeps re-requesting. Click it: expect the vertiports to become the bottleneck, with requests rejected and queues forming.', ready: (a) => a.state.engine.stress },
  { id: 'done', modal: true, title: 'You are set', text: 'That is the whole loop: watch, inspect, break, switch things off, observe. The "? Tutorial" button at the top replays this any time. Finishing resets the simulation to a fresh run with your original settings.' },
];

/** True when a UI event should be let through during `step` (pure, so it is unit-tested). `match(sel)` tells whether the event target is inside `sel`. */
export function eventAllowed(step, match, code = null) {
  if (match('#tut-pop')) return true;
  if (code !== null) return (step.keys || []).includes(code) || (step.allow || []).some(match);
  return (step.allow || []).some(match);
}

export class Tutorial {
  constructor(api) {
    this.api = api; this.i = -1; this.active = false; this.base = null; this.raf = 0; this.ready = false; this.hintT = 0;
    this._gate = this._gate.bind(this); this._loop = this._loop.bind(this);
  }
  static seen() { try { return localStorage.getItem(STORAGE_KEY) === 'done'; } catch { return false; } }
  static markSeen() { try { localStorage.setItem(STORAGE_KEY, 'done'); } catch { /* private mode */ } }

  start() {
    if (this.active) return;
    this.active = true; this.api.begin();
    this._build();
    for (const t of ['pointerdown', 'mousedown', 'click', 'dblclick', 'touchstart', 'keydown', 'keyup', 'wheel', 'contextmenu']) window.addEventListener(t, this._gate, true);
    document.body.classList.add('tutorial-on');
    this.go(0); this.raf = requestAnimationFrame(this._loop);
  }
  finish(skipped) {
    if (!this.active) return;
    this.active = false; cancelAnimationFrame(this.raf);
    for (const t of ['pointerdown', 'mousedown', 'click', 'dblclick', 'touchstart', 'keydown', 'keyup', 'wheel', 'contextmenu']) window.removeEventListener(t, this._gate, true);
    this.root.remove(); document.body.classList.remove('tutorial-on');
    Tutorial.markSeen(); this.api.end(skipped);
  }

  get step() { return STEPS[this.i]; }
  go(i) {
    if (i >= STEPS.length) return this.finish(false);
    this.i = i; const s = this.step;
    this.base = (s.enter && s.enter(this.api)) || {}; this.ready = !s.ready;
    // narrow screens stack the panels and the page scrolls: bring the "watch here" areas, then (last, so it wins) the thing to act on, into view
    for (const sel of [...(s.see || []), typeof s.target === 'string' ? s.target : s.target ? '#map' : null]) { const el = sel && document.querySelector(sel); if (el && el.scrollIntoView) el.scrollIntoView({ block: 'nearest', inline: 'nearest' }); }
    this._renderPop(); this._hint('');
  }

  // ------------------------------------------------------------------ input gate: only the step's expected actions get through
  _gate(ev) {
    if (!this.active) return;
    const s = this.step, t = ev.target && ev.target.closest ? ev.target : document.body;
    if (ev.type === 'keydown' && ev.code === 'Escape') { ev.preventDefault(); ev.stopPropagation(); this.finish(true); return; }
    const isKey = ev.type === 'keydown' || ev.type === 'keyup';
    if (eventAllowed(s, (sel) => !!t.closest(sel), isKey ? ev.code : null)) return;
    if (ev.type === 'wheel') return; // scrolling the page is harmless
    ev.preventDefault(); ev.stopPropagation(); ev.stopImmediatePropagation();
    if (ev.type === 'click' || ev.type === 'keydown') this._hint(s.allow ? 'Only the highlighted control works right now.' : 'Press Next to continue.');
  }
  _hint(msg) {
    if (!this.pop) return;
    const h = this.pop.querySelector('.tut-hint'); if (!h) return;
    h.textContent = msg; clearTimeout(this.hintT);
    if (msg) { this.pop.classList.remove('shake'); void this.pop.offsetWidth; this.pop.classList.add('shake'); this.hintT = setTimeout(() => { h.textContent = ''; }, 2200); }
  }

  // ------------------------------------------------------------------ DOM
  _build() {
    const NS = 'http://www.w3.org/2000/svg';
    this.root = document.createElement('div'); this.root.id = 'tut';
    this.svg = document.createElementNS(NS, 'svg'); this.svg.id = 'tut-svg';
    this.svg.innerHTML = '<defs><mask id="tut-mask"><rect id="tut-full" fill="white"/><g id="tut-holes" fill="black"></g></mask></defs><rect id="tut-dim" mask="url(#tut-mask)"/><g id="tut-rings" fill="none"></g>';
    this.root.appendChild(this.svg);
    this.arrows = document.createElement('div'); this.root.appendChild(this.arrows);
    this.pop = document.createElement('div'); this.pop.id = 'tut-pop'; this.root.appendChild(this.pop);
    document.body.appendChild(this.root);
    this.holesG = this.svg.querySelector('#tut-holes'); this.ringsG = this.svg.querySelector('#tut-rings'); this.full = this.svg.querySelector('#tut-full'); this.dim = this.svg.querySelector('#tut-dim');
  }
  _renderPop() {
    const s = this.step, last = this.i === STEPS.length - 1;
    this.pop.className = s.modal ? 'modal' : '';
    this.pop.innerHTML = `<div class="tut-count">${this.i + 1} / ${STEPS.length}</div><h4>${s.title}</h4><p>${s.text}</p><div class="tut-hint"></div>
      <div class="tut-btns"><button id="tut-skip">Skip tutorial</button><button id="tut-next" class="primary"></button></div>`;
    this.pop.querySelector('#tut-skip').onclick = () => this.finish(true);
    this.pop.querySelector('#tut-next').onclick = () => { if (this.ready) this.go(this.i + 1); };
    this.nextBtn = this.pop.querySelector('#tut-next'); this.nextLabel = last ? 'Finish' : 'Next ▶';
    this._syncNext(true);
  }
  _syncNext(force = false) {
    const s = this.step, ok = this.ready, key = ok ? 'on' : 'off';
    if (!force && this._nk === key) return; this._nk = key;
    this.nextBtn.disabled = !ok; this.nextBtn.classList.toggle('pulse', ok && !!s.ready);
    this.nextBtn.textContent = ok ? (s.ready ? '✓ ' : '') + this.nextLabel : 'Do the highlighted action';
  }

  // ------------------------------------------------------------------ per-frame layout
  _rectOf(sel) {
    if (typeof sel === 'function') return sel(this.api, this.base);
    const el = document.querySelector(sel); if (!el) return null;
    const r = el.getBoundingClientRect(); return r.width || r.height ? { left: r.left, top: r.top, width: r.width, height: r.height } : null;
  }
  _loop() {
    if (!this.active) return;
    const s = this.step;
    if (s.ready && !this.ready && s.ready(this.api, this.base)) { this.ready = true; this._hint(''); }
    this._syncNext();
    const vw = window.innerWidth, vh = window.innerHeight, pad = 6;
    const holes = [];
    if (s.target) { const r = this._rectOf(s.target); if (r) holes.push({ ...r, kind: 'do' }); }
    for (const sel of s.see || []) { const r = this._rectOf(sel); if (r) holes.push({ ...r, kind: 'see' }); }
    this.full.setAttribute('width', vw); this.full.setAttribute('height', vh); this.dim.setAttribute('width', vw); this.dim.setAttribute('height', vh);
    this.svg.setAttribute('width', vw); this.svg.setAttribute('height', vh);
    this.holesG.innerHTML = this.ringsG.innerHTML = ''; this.arrows.innerHTML = '';
    const NS = 'http://www.w3.org/2000/svg', hard = [], soft = [];
    for (const h of holes) {
      const x = h.left - pad, y = h.top - pad, w = h.width + pad * 2, hh = h.height + pad * 2;
      for (const [g, cls] of [[this.holesG, ''], [this.ringsG, 'ring ' + h.kind]]) { const r = document.createElementNS(NS, 'rect'); r.setAttribute('x', x); r.setAttribute('y', y); r.setAttribute('width', w); r.setAttribute('height', hh); r.setAttribute('rx', 6); if (cls) r.setAttribute('class', cls); g.appendChild(r); }
      const above = y > 38, a = document.createElement('div');
      a.className = 'tut-arrow ' + h.kind + (above ? ' above' : ' below');
      a.innerHTML = above ? `<span>${h.kind === 'do' ? 'do this' : 'watch here'}</span><i>▼</i>` : `<i>▲</i><span>${h.kind === 'do' ? 'do this' : 'watch here'}</span>`;
      this.arrows.appendChild(a);
      const aw = a.offsetWidth || 90, ax = Math.max(4, Math.min(vw - aw - 4, x + w / 2 - aw / 2)), ay = above ? y - 34 : y + hh + 2;
      a.style.left = ax + 'px'; a.style.top = ay + 'px';
      (h.kind === 'do' ? hard : soft).push({ left: x, top: y, width: w, height: hh }, { left: ax, top: ay, width: aw, height: 32 });
    }
    this._placePop(hard, soft, holes, vw, vh);
    this.raf = requestAnimationFrame(this._loop);
  }
  /** Put the popover where it never covers the control the user must act on (hard) and overlaps the "watch here" areas as little as possible (soft), staying near the action. */
  _placePop(hard, soft, holes, vw, vh) {
    const pop = this.pop, W0 = pop.classList.contains('modal') ? 380 : 300, h = pop.offsetHeight || 160; pop.style.width = W0 + 'px';
    if (!holes.length) { pop.style.left = (vw - W0) / 2 + 'px'; pop.style.top = Math.max(8, (vh - h) / 2) + 'px'; return; }
    const ov = (a, b) => Math.max(0, Math.min(a.left + a.width, b.left + b.width) - Math.max(a.left, b.left)) * Math.max(0, Math.min(a.top + a.height, b.top + b.height) - Math.max(a.top, b.top));
    const a0 = (hard[0] && typeof this.step.target !== 'function') ? hard[0] : (soft[0] || hard[0]), ax = a0.left + a0.width / 2, ay = a0.top + a0.height / 2;
    const score = (x, y) => {
      const box = { left: x, top: y, width: W0, height: h }; let sc = 0;
      for (const r of hard) sc += ov(box, r) ? 1e9 + ov(box, r) : 0;
      for (const r of soft) sc += ov(box, r);
      return sc + Math.hypot(x + W0 / 2 - ax, y + h / 2 - ay) * 40;
    };
    let best = null;
    for (let x = 8; x <= Math.max(8, vw - W0 - 8); x += 20) for (let y = 8; y <= Math.max(8, vh - h - 8); y += 20) { const sc = score(x, y); if (!best || sc < best.sc) best = { x, y, sc }; }
    const prev = this._pp; if (prev && prev.step === this.i && score(prev.x, prev.y) <= best.sc + 4000) best = prev; // hysteresis: do not jitter
    this._pp = { x: best.x, y: best.y, step: this.i };
    pop.style.left = best.x + 'px'; pop.style.top = best.y + 'px';
  }
}
