import { test } from 'node:test';
import assert from 'node:assert/strict';
import { RobotPresenceTracker } from '../src/lib/components/map3d/robotPresence.ts';

test('transient missing robots retain their last pose only within one source frame', () => {
  const tracker = new RobotPresenceTracker(3);
  assert.deepEqual([...tracker.update(['r1'], 10, 'mission-a/component-a/frame-1').visible], ['r1']);
  assert.deepEqual([...tracker.update([], 12, 'mission-a/component-a/frame-1').visible], ['r1']);
  assert.deepEqual([...tracker.update([], 14, 'mission-a/component-a/frame-1').visible], []);
});

test('mission, component, or frame transition clears retained identities immediately', () => {
  const tracker = new RobotPresenceTracker(3);
  tracker.update(['r1'], 10, 'mission-a/component-a/frame-1');
  const changed = tracker.update([], 10.1, 'mission-a/component-b/frame-2');
  assert.equal(changed.reset, true);
  assert.deepEqual([...changed.visible], []);
});
