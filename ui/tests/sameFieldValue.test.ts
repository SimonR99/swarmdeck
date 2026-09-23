import { test } from 'node:test';
import assert from 'node:assert/strict';
import { keepIfUnchanged, sameDrawnValue, sameFieldValue } from '../src/lib/stores/sameFieldValue.ts';

test('comparison distinguishes shape as well as value', () => {
  assert.equal(sameFieldValue(null, null), true);
  assert.equal(sameFieldValue(null, {}), false);
  assert.equal(sameFieldValue([1, 2], [1, 2]), true);
  assert.equal(sameFieldValue([1, 2], [2, 1]), false);
  assert.equal(sameFieldValue({ x: 1 }, { x: 1, y: 2 }), false);
  assert.equal(sameFieldValue({ x: 1, y: undefined }, { x: 1 }), false);
  assert.equal(sameFieldValue({ x: { y: [1, { z: 'a' }] } }, { x: { y: [1, { z: 'a' }] } }), true);
  assert.equal(sameFieldValue({ x: { y: [1, { z: 'a' }] } }, { x: { y: [1, { z: 'b' }] } }), false);
});

test('a poll that returned the same catalogue keeps the published reference', () => {
  const published = [{ scope: 'component:0', robots: ['robot_0'] }];
  assert.equal(keepIfUnchanged(published, [{ scope: 'component:0', robots: ['robot_0'] }]), published);

  const changed = [{ scope: 'component:0', robots: ['robot_0', 'robot_1'] }];
  assert.equal(keepIfUnchanged(published, changed), changed);
});

test('drawn values ignore last-digit recomputation noise but not a micrometre move', () => {
  // A parked robot's pose, resent in a keep-alive after passing through a transform.
  assert.equal(sameDrawnValue({ x: 3.9984943488578946, y: -0.642608897609956 }, { x: 3.9984943488578946, y: -0.6426088976099558 }), true);
  assert.equal(sameDrawnValue([[1.591072373352616, -1e-20]], [[1.5910723733526169, 0]]), true);
  assert.equal(sameDrawnValue({ x: 0 }, { x: 0.000002 }), false);
  assert.equal(sameDrawnValue({ x: 0.01 }, { x: 0 }), false);
  assert.equal(sameDrawnValue({ x: 1 }, { x: 1, y: 0 }), false);
  assert.equal(sameDrawnValue([1], [1, 1]), false);
  assert.equal(sameDrawnValue('idle', 'idle'), true);
  assert.equal(sameDrawnValue('idle', 'active'), false);
  assert.equal(sameDrawnValue(null, 0), false);
});

test('equal-within-noise values still differ as field values, so the store keeps the newest', () => {
  assert.equal(sameFieldValue(-0.642608897609956, -0.6426088976099558), false);
});
