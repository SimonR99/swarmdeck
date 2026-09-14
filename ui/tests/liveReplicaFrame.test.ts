import assert from 'node:assert/strict';
import test from 'node:test';
import {
  LIVE_REPLICA_FRESHNESS_BUDGET_S,
  liveRobotToMapRobot,
  liveReplicaMatchesSelection,
  parseLiveReplicaFrame,
  postLiveReplicaGoal
} from '../src/lib/components/map3d/liveReplicaFrame.ts';

const session = '12345678-1234-4234-8234-567812345678';
const component = 'component:merged';

function payload() {
  return {
    version: 1,
    mission_id: session,
    session_id: session,
    component_id: component,
    frame_id: 'component_frame',
    solution_order: [2, 7],
    robots: [{
      robot_id: 'robot-a',
      mission_id: session,
      component_id: component,
      navigation_frame: 'nav_a',
      T_component_navigation: [
        [0, -1, 0, 10],
        [1, 0, 0, 20],
        [0, 0, 1, 1],
        [0, 0, 0, 1]
      ],
      pose: { x: 1, y: 2, z: 0, yaw: 0 },
      goal: { x: 2, y: 2, z: 0, yaw: 0 },
      planned_path: [{ x: 1, y: 2 }, { x: 2, y: 2 }],
      global_planned_path: [{ x: 1, y: 2 }, { x: 3, y: 2 }],
      local_planned_path: [{ x: 1, y: 2 }, { x: 1, y: 3 }],
      freshness: { pose_s: 0, goal_s: 0, path_s: 0 },
      nav_status: 'active',
      mode: 'nav'
    }]
  };
}

test('live telemetry validates frame identity and transforms navigation data into component coordinates', () => {
  const frame = parseLiveReplicaFrame(payload());
  const robot = liveRobotToMapRobot(frame.robots[0], undefined);
  assert.deepEqual(robot.pose, { x: 8, y: 21, yaw: Math.PI / 2 });
  assert.deepEqual(robot.goal, { x: 8, y: 22 });
  assert.deepEqual(robot.global_planned_path, [{ x: 8, y: 21 }, { x: 8, y: 23 }]);
  assert.equal(robot.nav_status, 'active');
});

test('stale goal and paths are suppressed while pose freshness remains independently bounded', () => {
  const frame = parseLiveReplicaFrame(payload());
  frame.robots[0].freshness.goal_s = LIVE_REPLICA_FRESHNESS_BUDGET_S;
  frame.robots[0].freshness.path_s = LIVE_REPLICA_FRESHNESS_BUDGET_S;
  const robot = liveRobotToMapRobot(frame.robots[0], undefined, 0.01);
  assert.equal(robot.goal, null);
  assert.deepEqual(robot.planned_path, []);
  assert.deepEqual(robot.pose, { x: 8, y: 21, yaw: Math.PI / 2 });
});

test('reuses projected geometry while freshness changes independently', () => {
  const frame = parseLiveReplicaFrame(payload());
  const robot = frame.robots[0];
  const first = liveRobotToMapRobot(robot, undefined, 0);
  const second = liveRobotToMapRobot(robot, undefined, 1);
  assert.equal(first.planned_path, second.planned_path);
  assert.equal(first.global_planned_path, second.global_planned_path);
  assert.equal(first.local_planned_path, second.local_planned_path);
  robot.freshness.path_s = LIVE_REPLICA_FRESHNESS_BUDGET_S;
  robot.freshness.goal_s = LIVE_REPLICA_FRESHNESS_BUDGET_S;
  const expired = liveRobotToMapRobot(robot, undefined, 0.01);
  assert.deepEqual(expired.planned_path, []);
  assert.deepEqual(expired.goal, null);
  const freshAgain = liveRobotToMapRobot(robot, undefined, 0);
  assert.equal(freshAgain.planned_path, first.planned_path);
  assert.equal(freshAgain.goal, first.goal);
});

test('telemetry rejects non-rigid transforms and mismatched robot component identity', () => {
  const invalidTransform = payload();
  invalidTransform.robots[0].T_component_navigation[0][0] = 2;
  assert.throws(() => parseLiveReplicaFrame(invalidTransform), /rotation is invalid/);
  const mismatch = payload();
  mismatch.robots[0].component_id = 'component:other';
  assert.throws(() => parseLiveReplicaFrame(mismatch), /frame identity/);
});

test('telemetry from an old mission or component cannot serve the current selection', () => {
  const frame = parseLiveReplicaFrame(payload());
  const live = { frame, receivedAt: 1 };
  const selection = { scope: 'fleet' as const, robotId: 'fleet', sessionId: session, componentId: component };
  assert.equal(liveReplicaMatchesSelection(live, selection, [2, 7]), true);
  assert.equal(liveReplicaMatchesSelection(live, selection, [2, 8]), false);
  assert.equal(liveReplicaMatchesSelection(live, selection, null), false);
  assert.equal(liveReplicaMatchesSelection(live, { ...selection, componentId: 'component:old' }, [2, 7]), false);
  assert.equal(liveReplicaMatchesSelection(live, { ...selection, sessionId: '99999999-9999-4999-8999-999999999999' }, [2, 7]), false);
});

test('live goal endpoint posts component coordinates to the bounded route', async () => {
  const oldFetch = globalThis.fetch;
  let request: { url: string; init?: RequestInit } | null = null;
  globalThis.fetch = async (input, init) => {
    request = { url: String(input), init };
    return Response.json({ ok: true });
  };
  try {
    await postLiveReplicaGoal(
      { scope: 'fleet', robotId: 'fleet', sessionId: session, componentId: component },
      'robot-a',
      [2, 7],
      { x: 8, y: 22, z: 0.4, yaw: 0 }
    );
  } finally {
    globalThis.fetch = oldFetch;
  }
  assert.equal(request?.url, `/api/autonomy/replicas/components/live/${session}/goal`);
  assert.deepEqual(JSON.parse(String(request?.init?.body)), {
    robot_id: 'robot-a', component_id: component, solution_order: [2, 7],
    goal: { x: 8, y: 22, z: 0.4, yaw: 0 }
  });
});

test('initial frame sentinel is explicit and missing solution order is rejected', () => {
  const initial = payload();
  initial.solution_order = [0, -1];
  const frame = parseLiveReplicaFrame(initial);
  const selection = { scope: 'robot' as const, robotId: 'robot-a', sessionId: session, componentId: component };
  assert.equal(liveReplicaMatchesSelection({ frame, receivedAt: 1 }, selection, null), true);

  const missing = payload();
  delete (missing as { solution_order?: unknown }).solution_order;
  assert.throws(() => parseLiveReplicaFrame(missing), /solution_order/);
});
