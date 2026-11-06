import test from 'node:test';
import assert from 'node:assert/strict';
import { clamp, lerp, smoothstep, RNG, MinHeap, erf, normalCdf, normalInv, makeNoise, makeFbm, hashStr, mulberry32 } from '../src/core/util.js';
import { near } from './helpers.js';

test('clamp / lerp / smoothstep basics', () => {
  assert.equal(clamp(5, 0, 3), 3); assert.equal(clamp(-1, 0, 3), 0); assert.equal(clamp(2, 0, 3), 2);
  assert.equal(lerp(2, 4, 0.5), 3);
  assert.equal(smoothstep(0, 1, -1), 0); assert.equal(smoothstep(0, 1, 2), 1); near(smoothstep(0, 1, 0.5), 0.5, 1e-9);
});
test('RNG is deterministic per seed and differs across seeds', () => {
  const a = new RNG(5), b = new RNG(5), c = new RNG(6);
  const sa = Array.from({ length: 20 }, () => a.float()), sb = Array.from({ length: 20 }, () => b.float()), sc = Array.from({ length: 20 }, () => c.float());
  assert.deepEqual(sa, sb); assert.notDeepEqual(sa, sc);
  assert.ok(sa.every((x) => x >= 0 && x < 1));
});
test('RNG.fork gives independent but reproducible streams', () => {
  const a = new RNG(9).fork('x'), b = new RNG(9).fork('x'), c = new RNG(9).fork('y');
  assert.equal(a.float(), b.float()); assert.notEqual(new RNG(9).fork('x').float(), c.float());
});
test('RNG range/int/chance/gauss/exp statistics', () => {
  const r = new RNG(1), n = 20000;
  let s = 0, s2 = 0, ints = new Set(), ch = 0, ex = 0;
  for (let i = 0; i < n; i++) { const g = r.gauss(); s += g; s2 += g * g; ints.add(r.int(5)); if (r.chance(0.3)) ch++; ex += r.exp(2); const v = r.range(3, 4); assert.ok(v >= 3 && v < 4); }
  near(s / n, 0, 0.03, 'gauss mean'); near(s2 / n, 1, 0.05, 'gauss var'); assert.equal(ints.size, 5); near(ch / n, 0.3, 0.02); near(ex / n, 0.5, 0.03);
});
test('MinHeap pops in key order', () => {
  const h = new MinHeap(), r = new RNG(3), keys = Array.from({ length: 500 }, () => r.float());
  keys.forEach((k, i) => h.push(k, i));
  let prev = -1;
  while (h.size) { const k = h.peekKey(), v = h.pop(); assert.ok(k >= prev); assert.equal(keys[v], k); prev = k; }
});
test('erf / normalCdf / normalInv', () => {
  near(erf(0), 0, 1e-6); near(erf(1), 0.8427, 1e-3); near(erf(-1), -0.8427, 1e-3);
  near(normalCdf(0), 0.5, 1e-6); near(normalCdf(1.96), 0.975, 1e-3);
  for (const p of [0.001, 0.05, 0.5, 0.9, 0.999]) near(normalCdf(normalInv(p)), p, 1e-3, 'inverse');
  near(normalInv(0.975), 1.96, 0.01);
});
test('value noise is deterministic, bounded and smooth', () => {
  const n1 = makeNoise(4), n2 = makeNoise(4), f = makeFbm(4, 3);
  for (let i = 0; i < 100; i++) { const x = i * 0.37, y = i * 0.11; assert.equal(n1(x, y), n2(x, y)); const v = n1(x, y); assert.ok(v >= -1.01 && v <= 1.01); assert.ok(Number.isFinite(f(x, y))); }
  assert.ok(Math.abs(n1(1.0, 1.0) - n1(1.001, 1.0)) < 0.05);
});
test('hashStr and mulberry32 are stable', () => { assert.equal(hashStr('AC-001'), hashStr('AC-001')); assert.notEqual(hashStr('a'), hashStr('b')); const m = mulberry32(2); assert.equal(typeof m(), 'number'); });
