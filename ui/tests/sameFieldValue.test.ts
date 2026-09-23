import { test } from 'node:test';
import assert from 'node:assert/strict';
import { keepIfUnchanged, sameFieldValue } from '../src/lib/stores/sameFieldValue.ts';

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
