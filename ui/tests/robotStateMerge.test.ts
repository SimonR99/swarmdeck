import { test } from 'node:test';
import assert from 'node:assert/strict';
import { mergeRobotState } from '../src/lib/stores/robotStateMerge.ts';
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
  const update = mergeRobotState(previous, robotState({ t_wall: 1 }));
  assert.equal(update.value, previous);
  assert.equal(update.changed, false);
  assert.equal(update.drawable, false);
});

test('a clock or unattended timer is stored but does not make the maps redraw', () => {
  const previous = robotState();
  const update = mergeRobotState(previous, robotState({ t_wall: 9, unattended_s: 12 }));
  assert.equal(update.changed, true);
  assert.equal(update.drawable, false);
  assert.equal(update.value.unattended_s, 12);
  assert.equal(update.value.pose, previous.pose);
});

test('a changed field is taken from the message and unchanged fields keep their reference', () => {
  const previous = robotState();
  const next = robotState({ pose: { x: 4, y: 2, yaw: 0.5 } });
  const update = mergeRobotState(previous, next);
  assert.notEqual(update.value, previous);
  assert.equal(update.drawable, true);
  assert.equal(update.value.pose, next.pose);
  // The path did not change, so the renderers can still detect that by identity.
  assert.equal(update.value.planned_path, previous.planned_path);
});

test('a path whose points changed replaces the array, an identical one does not', () => {
  const previous = robotState();
  const same = mergeRobotState(previous, robotState({ planned_path: [{ x: 1, y: 2 }] }));
  assert.equal(same.value, previous);
  const moved = robotState({ planned_path: [{ x: 1, y: 2 }, { x: 3, y: 4 }] });
  assert.equal(mergeRobotState(previous, moved).value.planned_path, moved.planned_path);
});

test('fields absent from the message keep their previous value', () => {
  const previous = robotState({ footprint_radius: 0.4 });
  const update = mergeRobotState(previous, robotState({ battery_pct: 50 }));
  assert.equal(update.value.footprint_radius, 0.4);
  assert.equal(update.value.battery_pct, 50);
});

test('the first message for a robot is adopted whole', () => {
  const first = robotState();
  assert.deepEqual(mergeRobotState(undefined, first), {
    value: first,
    changed: true,
    drawable: true
  });
});

test('a keep-alive with only recomputation noise in the pose is stored but does not redraw', () => {
  const previous = robotState({ pose: { x: 3.9984943488578946, y: -0.642608897609956, yaw: 1.5910723733526169 } });
  const update = mergeRobotState(
    previous,
    robotState({ pose: { x: 3.9984943488578946, y: -0.6426088976099558, yaw: 1.591072373352616 } })
  );
  assert.equal(update.changed, true);
  assert.equal(update.drawable, false);
  assert.equal(update.value.pose.y, -0.6426088976099558);
});

test('a one-centimetre move still redraws', () => {
  const update = mergeRobotState(robotState(), robotState({ pose: { x: 1.01, y: 2, yaw: 0.5 } }));
  assert.equal(update.drawable, true);
});

test('the live mapping authority age advances in every keep-alive without a redraw', () => {
  const liveMapping = (age: number, yaw: number) => ({ component_id: 'component:0', authority_age_s: age, pose: { x: 0, y: 0, yaw } });
  const previous = robotState({ live_mapping: liveMapping(0.228, -1.550527626647384) } as Partial<RobotState>);
  const aged = mergeRobotState(previous, robotState({ live_mapping: liveMapping(0.233, -1.5505276266473842) } as Partial<RobotState>));
  assert.equal(aged.changed, true);
  assert.equal(aged.drawable, false);
  const moved = mergeRobotState(previous, robotState({ live_mapping: { ...liveMapping(0.233, 0), component_id: 'component:1' } } as Partial<RobotState>));
  assert.equal(moved.drawable, true);
});
