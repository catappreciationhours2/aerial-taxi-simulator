// Safe Interval Path Planning (SIPP) over the 3-D lattice with continuous time, wind- and energy-aware
// edge costs, and a relaxed-problem heuristic.
//
// adapted from: M. Phillips and M. Likhachev, "SIPP: Safe Interval Path Planning for Dynamic Environments",
// IEEE ICRA 2011.  A search state is (node, safe interval); the earliest arrival time is propagated, and
// waiting inside a safe interval is implicit.
// Differences from the paper (all deliberate, for aerial taxis):
//   * edge durations depend on departure time (time-varying wind) and are re-evaluated when waiting shifts departure;
//   * the objective is a weighted cost (time + energy + risk + noise), so labels are compared on (cost, time)
//     with Pareto dominance (at most MAX_LABELS per state) instead of earliest-arrival alone;
//   * waiting means hovering and is charged hover power (multicopters cannot idle for free);
//   * a move occupies BOTH endpoint nodes (and the two shared corner nodes of a diagonal) for its whole duration,
//     which rules out swap and crossing conflicts without separate edge constraints;
//   * the goal needs a safe interval long enough for the final descent (landing slot).

import { MinHeap } from '../core/util.js';
import { PLANNER } from '../core/config.js';
import { H8 } from './heuristics.js';

const MAX_LABELS = 3;
const KEY = 512;

/**
 * @param req {id, startNode, t0, goalNode, tGoalMin, landDur, landCost, maxExpansions, deadlineMs}
 * @param ctx {world, cost, table, h}   h = Float64Array relaxed heuristic to goal
 */
export function sippSearch(req, ctx) {
  const { world, cost, table, h } = ctx;
  const { id, startNode, t0, goalNode, tGoalMin = 0, landDur = 0, landCost = 0 } = req;
  const eps = req.epsilon ?? 1;
  const maxExp = req.maxExpansions ?? PLANNER.maxExpansions;
  const L = world.layer;
  const siCache = new Map();
  const SI = (node) => {
    let a = siCache.get(node);
    if (!a) { a = table ? table.safeIntervals(node, id, t0 - 1) : [[t0 - 1, Infinity]]; siCache.set(node, a); }
    return a;
  };
  const stats = { expansions: 0, generated: 0 };
  if (world.blocked[startNode] || world.blocked[goalNode]) return { ok: false, reason: 'blocked-endpoint', ...stats };
  if (!(h[startNode] < Infinity)) return { ok: false, reason: 'unreachable', ...stats };

  const open = new MinHeap();
  const labels = new Map();
  const tmp = { ok: false, dur: 0, energy: 0, risk: 0, noise: 0, cost: 0, va: 0, g: 0 };
  const tmp2 = { ok: false, dur: 0, energy: 0, risk: 0, noise: 0, cost: 0, va: 0, g: 0 };

  function tryInsert(lab) {
    const key = lab.node * KEY + Math.min(lab.ii, KEY - 1);
    let arr = labels.get(key);
    if (!arr) { arr = []; labels.set(key, arr); }
    for (const o of arr) if (o.g <= lab.g + 1e-9 && o.t <= lab.t + 1e-9) return false; // dominated
    for (let i = arr.length - 1; i >= 0; i--) if (lab.g <= arr[i].g + 1e-9 && lab.t <= arr[i].t + 1e-9) { arr[i].dead = true; arr.splice(i, 1); }
    if (arr.length >= MAX_LABELS) {
      let worst = 0;
      for (let i = 1; i < arr.length; i++) if (arr[i].g > arr[worst].g) worst = i;
      if (arr[worst].g <= lab.g) return false;
      arr[worst].dead = true; arr.splice(worst, 1);
    }
    arr.push(lab);
    return true;
  }

  function terminalIfLandable(arrLab, J) {
    if (arrLab.node !== goalNode) return false;
    const tLand = Math.max(arrLab.t, tGoalMin);
    if (tLand + landDur > J[1]) return false;
    const gT = arrLab.g + cost.holdCost(tLand - arrLab.t) + landCost;
    open.push(gT, { term: true, base: arrLab, tLand, g: gT });
    return true;
  }

  // ---- start label
  const S0 = SI(startNode);
  let si0 = -1;
  for (let i = 0; i < S0.length; i++) if (S0[i][1] > t0 + 1e-6) { si0 = i; break; }
  if (si0 < 0) return { ok: false, reason: 'start-blocked', ...stats };
  const start = { node: startNode, ii: si0, t: Math.max(t0, S0[si0][0]), g: 0, parent: null, dep: 0, dur: 0 };
  tryInsert(start);
  if (!terminalIfLandable(start, S0[si0])) open.push(eps * h[startNode], start);

  const neighbours = [];
  while (open.size) {
    const cur = open.pop();
    if (cur.term) return finish(cur, stats);
    if (cur.dead) continue;
    if (++stats.expansions > maxExp) return { ok: false, reason: 'budget', ...stats };
    if (req.deadlineMs && (stats.expansions & 127) === 0 && performance.now() > req.deadlineMs) return { ok: false, reason: 'deadline', ...stats };
    const n = cur.node, t = cur.t, g = cur.g;
    const I = SI(n)[cur.ii];
    const iz = (n / L) | 0, c = n % L;
    neighbours.length = 0;
    for (let k = 0; k < 8; k++) {
      const r = cost.staticCorners(n, H8[k][0], H8[k][1]);
      if (r) neighbours.push(r);
    }
    for (const dz of [-1, 1]) {
      const jz = iz + dz;
      if (jz < 0 || jz >= world.nz) continue;
      const m = jz * L + c;
      if (!world.blocked[m]) neighbours.push({ m, corners: null });
    }
    for (const nb of neighbours) {
      const m = nb.m, hm = h[m];
      if (!(hm < Infinity)) continue;
      let info = cost.evalEdge(n, m, t, tmp);
      if (!info.ok) continue;
      const Js = SI(m);
      for (let j = 0; j < Js.length; j++) {
        const jb = Js[j][1];
        if (jb <= t) continue;
        let td = Math.max(t, Js[j][0]);
        let dur = info.dur;
        if (td + dur > I[1]) break;
        let inf = info;
        if (td - t > 3) {
          inf = cost.evalEdge(n, m, td, tmp2);
          if (!inf.ok) continue;
          dur = inf.dur;
          if (td + dur > I[1]) continue;
        }
        if (td + dur > jb) continue;
        if (nb.corners) {
          const limit = Math.min(I[1], jb) - dur;
          let ok = true;
          for (let guard = 0; guard < 8 && ok; guard++) {
            let bump = -1;
            for (const cn of nb.corners) { const e = table ? table.conflictEnd(cn, td, td + dur, id) : -1; if (e > bump) bump = e; }
            if (bump < 0) break;
            td = bump;
            if (td > limit) ok = false;
            if (guard === 7) ok = false;
          }
          if (!ok || td + dur > Math.min(I[1], jb)) continue;
        }
        const arr = td + dur;
        const gNew = g + cost.holdCost(td - t) + inf.cost;
        const lab = {
          node: m, ii: j, t: arr, g: gNew, parent: cur, dep: td, dur,
          energy: inf.energy, risk: inf.risk, noise: inf.noise, va: inf.va, gs: inf.g, dead: false,
        };
        stats.generated++;
        if (!tryInsert(lab)) continue;
        if (m === goalNode && terminalIfLandable(lab, Js[j])) continue;
        open.push(gNew + eps * hm, lab);
      }
    }
  }
  return { ok: false, reason: 'exhausted', ...stats };
}

function finish(term, stats) {
  const chain = [];
  for (let l = term.base; l; l = l.parent) chain.push(l);
  chain.reverse();
  const path = chain.map((l) => ({ node: l.node, t: l.t, dep: l.dep, dur: l.dur, energy: l.energy || 0, risk: l.risk || 0, noise: l.noise || 0, va: l.va || 0, gs: l.gs || 0 }));
  return { ok: true, path, tLand: term.tLand, cost: term.g, ...stats };
}

/**
 * Convert a SIPP path into trajectory segments.
 * opts: {world, cost, landDur, tLand, padElev, landEnergy}
 */
export function pathToSegments(path, tLand, { world, cost, landDur = 0, padElev = null, landEnergy = 0 }) {
  const segs = [];
  const pos = (n) => world.center(n);
  for (let i = 1; i < path.length; i++) {
    const a = path[i - 1], b = path[i];
    if (b.dep > a.t + 1e-6) segs.push({ kind: 'hold', t0: a.t, t1: b.dep, p0: pos(a.node), p1: pos(a.node), n0: a.node, n1: a.node, energy: cost.holdEnergy(b.dep - a.t) });
    const [ax, ay, az] = world.decode(a.node), [bx, by, bz] = world.decode(b.node);
    const diag = ax !== bx && ay !== by;
    segs.push({
      kind: az !== bz ? 'vmove' : 'move', t0: b.dep, t1: b.t, p0: pos(a.node), p1: pos(b.node), n0: a.node, n1: b.node,
      energy: b.energy, va: b.va, gs: b.gs, risk: b.risk, noise: b.noise,
      corners: diag ? [world.idx(bx, ay, az), world.idx(ax, by, az)] : null,
    });
  }
  const last = path[path.length - 1];
  if (tLand > last.t + 1e-6) segs.push({ kind: 'hold', t0: last.t, t1: tLand, p0: pos(last.node), p1: pos(last.node), n0: last.node, n1: last.node, energy: cost.holdEnergy(tLand - last.t) });
  if (padElev !== null && landDur > 0) {
    const p = pos(last.node);
    segs.push({ kind: 'land', t0: tLand, t1: tLand + landDur, p0: p, p1: [p[0], p[1], padElev], n0: last.node, n1: last.node, energy: landEnergy });
  }
  return segs;
}
