import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  TRAIL_MAX_POINTS,
  TRAIL_MIN_STEP_M,
  TRAIL_RESET_JUMP_M,
  TrailRecorder
} from '../src/lib/stores/trailRecorder.ts';

test('the first sample starts a trail and a repeated pose adds nothing', () => {
  const recorder = new TrailRecorder();
  assert.equal(recorder.record('robot_0', 1, 1), true);
  assert.equal(recorder.record('robot_0', 1, 1), false);
  assert.equal(recorder.record('robot_0', 1 + TRAIL_MIN_STEP_M / 2, 1), false);
  assert.deepEqual(recorder.points('robot_0'), [{ x: 1, y: 1 }]);
});

test('a parked robot records nothing however often its state is republished', () => {
  const recorder = new TrailRecorder();
  recorder.record('robot_0', 4, 4);
  for (let i = 0; i < 50; i++) assert.equal(recorder.record('robot_0', 4, 4), false);
  assert.equal(recorder.points('robot_0').length, 1);
});

test('a moving robot records a point per step, whatever the telemetry cadence', () => {
  const recorder = new TrailRecorder();
  // One second of movement arriving as one message, not five, still records it.
  recorder.record('robot_0', 0, 0);
  recorder.record('robot_0', 0.5, 0);
  recorder.record('robot_0', 1.0, 0);
  assert.deepEqual(recorder.points('robot_0'), [
    { x: 0, y: 0 },
    { x: 0.5, y: 0 },
    { x: 1, y: 0 }
  ]);
});

test('a relocalisation jump restarts the trail where the robot now is', () => {
  const recorder = new TrailRecorder();
  recorder.record('robot_0', 0, 0);
  recorder.record('robot_0', 0.5, 0);
  assert.equal(recorder.record('robot_0', 0.5 + TRAIL_RESET_JUMP_M + 0.1, 0), true);
  assert.deepEqual(recorder.points('robot_0'), [{ x: 3.6, y: 0 }]);
});

test('a trail is bounded and keeps the most recent points', () => {
  const recorder = new TrailRecorder();
  for (let i = 0; i < TRAIL_MAX_POINTS + 25; i++) recorder.record('robot_0', i * 0.5, 0);
  const points = recorder.points('robot_0');
  assert.equal(points.length, TRAIL_MAX_POINTS);
  assert.deepEqual(points[points.length - 1], { x: (TRAIL_MAX_POINTS + 24) * 0.5, y: 0 });
});

test('trails are per robot and can be retired one at a time', () => {
  const recorder = new TrailRecorder();
  recorder.record('robot_0', 0, 0);
  recorder.record('robot_1', 9, 9);
  recorder.clear('robot_0');
  assert.deepEqual(recorder.points('robot_0'), []);
  assert.deepEqual(recorder.points('robot_1'), [{ x: 9, y: 9 }]);
  recorder.clear();
  assert.equal(recorder.all().size, 0);
});

test('registration changes clear history even when the world pose stays still', () => {
  const recorder = new TrailRecorder();
  const source = { x: 0, y: 0, yaw: 0 };
  recorder.record('robot_0', 0, 0, source);
  recorder.record('robot_0', .5, 0, source);
  recorder.record('robot_0', .5, 0, { ...source, yaw: .01 });
  assert.deepEqual(recorder.points('robot_0'), [{ x: .5, y: 0 }]);
  recorder.record('robot_0', .6, 0);
  assert.deepEqual(recorder.points('robot_0'), [{ x: .6, y: 0 }]);
});

test('sub-tolerance registration noise preserves history but cumulative drift clears it', () => {
  const recorder = new TrailRecorder();
  recorder.record('robot_0', 0, 0, { x: 0, y: 0, yaw: Math.PI });
  recorder.record('robot_0', .1, 0, { x: .0005, y: 0, yaw: -Math.PI });
  assert.equal(recorder.points('robot_0').length, 2);
  recorder.record('robot_0', .2, 0, { x: .002, y: 0, yaw: Math.PI });
  assert.deepEqual(recorder.points('robot_0'), [{ x: .2, y: 0 }]);
});

test('telemetry without a finite pose is ignored', () => {
  const recorder = new TrailRecorder();
  assert.equal(recorder.record('robot_0', Number.NaN, 0), false);
  assert.equal(recorder.all().size, 0);
});
