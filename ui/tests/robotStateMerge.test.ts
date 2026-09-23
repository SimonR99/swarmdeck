import { test } from 'node:test';
import assert from 'node:assert/strict';
import { mergeRobotState, sameFieldValue } from '../src/lib/stores/robotStateMerge.ts';
import type { RobotState } from '../src/lib/types/protocol.ts';

function robotState(overrides: Partial<RobotState> = {}): RobotState {
  return {
    type: 'robot_state',
    robot_id: 'robot_0',
    t_wall: 1,
    pose: { x: 1, y: 2, yaw: 0.5 },
    battery_pct: 88,
    mode: 'idle',
    nav_status: 'idle',
    goal: null,
    planned_path: [{ x: 1, y: 2 }],
    capabilities: ['navigate'],
    unattended_s: 0,
    online: true,
    ...overrides
  } as RobotState;
}

test('a keep-alive that repeats the same telemetry performs no store write', () => {
  const previous = robotState();
  const merged = mergeRobotState(previous, robotState({ t_wall: 1 }));
  assert.equal(merged, previous);
});

test('a changed field is taken from the message and unchanged fields keep their reference', () => {
  const previous = robotState();
  const next = robotState({ pose: { x: 4, y: 2, yaw: 0.5 } });
  const merged = mergeRobotState(previous, next);
  assert.notEqual(merged, previous);
  assert.equal(merged.pose, next.pose);
  // The path did not change, so the renderers can still detect that by identity.
  assert.equal(merged.planned_path, previous.planned_path);
});

test('a path whose points changed replaces the array, an identical one does not', () => {
  const previous = robotState();
  const same = mergeRobotState(previous, robotState({ planned_path: [{ x: 1, y: 2 }] }));
  assert.equal(same, previous);
  const moved = robotState({ planned_path: [{ x: 1, y: 2 }, { x: 3, y: 4 }] });
  assert.equal(mergeRobotState(previous, moved).planned_path, moved.planned_path);
});

test('fields absent from the message keep their previous value', () => {
  const previous = robotState({ footprint_radius: 0.4 });
  const merged = mergeRobotState(previous, robotState({ battery_pct: 50 }));
  assert.equal(merged.footprint_radius, 0.4);
  assert.equal(merged.battery_pct, 50);
});

test('the first message for a robot is adopted whole', () => {
  const first = robotState();
  assert.equal(mergeRobotState(undefined, first), first);
});

test('field comparison distinguishes shape as well as value', () => {
  assert.equal(sameFieldValue(null, null), true);
  assert.equal(sameFieldValue(null, {}), false);
  assert.equal(sameFieldValue([1, 2], [1, 2]), true);
  assert.equal(sameFieldValue([1, 2], [2, 1]), false);
  assert.equal(sameFieldValue({ x: 1 }, { x: 1, y: 2 }), false);
  assert.equal(sameFieldValue({ x: 1, y: undefined }, { x: 1 }), false);
});
