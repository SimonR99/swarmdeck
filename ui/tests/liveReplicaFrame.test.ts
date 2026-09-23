import assert from 'node:assert/strict';
import test from 'node:test';
import {
  LIVE_REPLICA_FRESHNESS_BUDGET_S,
  liveRobotToMapRobot,
  liveReplicaDrawChanged,
  liveReplicaFreshnessDeadline,
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
  assert.deepEqual(robot.pose, { x: 8, y: 21, z: 1, yaw: Math.PI / 2 });
  assert.deepEqual(robot.goal, { x: 8, y: 22, z: 1 });
  assert.deepEqual(robot.global_planned_path, [{ x: 8, y: 21, z: 1 }, { x: 8, y: 23, z: 1 }]);
  assert.equal(robot.nav_status, 'active');
});

test('stale goal and paths are suppressed while pose freshness remains independently bounded', () => {
  const frame = parseLiveReplicaFrame(payload());
  frame.robots[0].freshness.goal_s = LIVE_REPLICA_FRESHNESS_BUDGET_S;
  frame.robots[0].freshness.path_s = LIVE_REPLICA_FRESHNESS_BUDGET_S;
  const robot = liveRobotToMapRobot(frame.robots[0], undefined, 0.01);
  assert.equal(robot.goal, null);
  assert.deepEqual(robot.planned_path, []);
  assert.deepEqual(robot.pose, { x: 8, y: 21, z: 1, yaw: Math.PI / 2 });
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
    goal: { x: 8, y: 22, z: 0.4, yaw: 0 },
    // The route takes the operator's "explore if unknown" choice with the
    // goal; not sending it at all is not the same as sending false.
    explore_if_unknown: false
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

test('a live frame that repeats a parked robot one poll later does not redraw', () => {
  const previous = { frame: parseLiveReplicaFrame(payload()), receivedAt: 1000 };
  const repeated = payload();
  // Only the freshness ages advance, and the pose comes back with last-digit noise.
  repeated.robots[0].freshness = { pose_s: 0.82, goal_s: 0.9, path_s: 0.9 };
  repeated.robots[0].pose = { x: 1.0000000000000002, y: 2, z: 0, yaw: 0 };
  const next = { frame: parseLiveReplicaFrame(repeated), receivedAt: 2000 };
  assert.equal(liveReplicaDrawChanged(previous, next, 2000), false);
});

test('a live frame that moves a robot, or changes the frame, redraws', () => {
  const previous = { frame: parseLiveReplicaFrame(payload()), receivedAt: 1000 };
  const moved = payload();
  moved.robots[0].pose = { x: 1.01, y: 2, z: 0, yaw: 0 };
  assert.equal(liveReplicaDrawChanged(previous, { frame: parseLiveReplicaFrame(moved), receivedAt: 2000 }, 2000), true);
  const reframed = payload();
  reframed.frame_id = 'component_frame_2';
  assert.equal(liveReplicaDrawChanged(previous, { frame: parseLiveReplicaFrame(reframed), receivedAt: 2000 }, 2000), true);
  const joined = payload();
  joined.robots.push({ ...joined.robots[0], robot_id: 'robot-b' });
  assert.equal(liveReplicaDrawChanged(previous, { frame: parseLiveReplicaFrame(joined), receivedAt: 2000 }, 2000), true);
});

test('a live frame whose pose, goal or path went stale redraws', () => {
  const previous = { frame: parseLiveReplicaFrame(payload()), receivedAt: 1000 };
  for (const field of ['pose_s', 'goal_s', 'path_s'] as const) {
    const stale = payload();
    stale.robots[0].freshness = { pose_s: 0, goal_s: 0, path_s: 0, [field]: LIVE_REPLICA_FRESHNESS_BUDGET_S + 0.5 };
    const next = { frame: parseLiveReplicaFrame(stale), receivedAt: 2000 };
    assert.equal(liveReplicaDrawChanged(previous, next, 2000), true, `${field} going stale did not redraw`);
  }
});

test('a live frame appearing or being dropped redraws', () => {
  const live = { frame: parseLiveReplicaFrame(payload()), receivedAt: 1000 };
  assert.equal(liveReplicaDrawChanged(null, live, 1000), true);
  assert.equal(liveReplicaDrawChanged(live, null, 1000), true);
  assert.equal(liveReplicaDrawChanged(null, null, 1000), false);
});

test('a live robot that went stale since its last frame redraws whether or not it is fresh again', () => {
  // Drawn fresh when it arrived; four seconds later it would be drawn stale.
  const previous = { frame: parseLiveReplicaFrame(payload()), receivedAt: 1000 };
  const recovered = { frame: parseLiveReplicaFrame(payload()), receivedAt: 5000 };
  assert.equal(liveReplicaDrawChanged(previous, recovered, 5000), true);
  const stillStale = payload();
  stillStale.robots[0].freshness = { pose_s: 4, goal_s: 4, path_s: 4 };
  const next = { frame: parseLiveReplicaFrame(stillStale), receivedAt: 5000 };
  assert.equal(liveReplicaDrawChanged(previous, next, 5000), true);
});

test('the next freshness deadline is the first drawn pose, goal or path to go stale', () => {
  const frame = parseLiveReplicaFrame(payload());
  frame.robots[0].freshness = { pose_s: 0.5, goal_s: 1, path_s: 2 };
  const live = { frame, receivedAt: 1000 };
  const budgetMs = LIVE_REPLICA_FRESHNESS_BUDGET_S * 1000;
  // Stale strictly after the budget, so the wake-up lands just past each deadline.
  assert.equal(liveReplicaFreshnessDeadline(live, 1000), 1000 + budgetMs - 2000 + 1);
  assert.equal(liveReplicaFreshnessDeadline(live, 2001), 1000 + budgetMs - 1000 + 1);
  assert.equal(liveReplicaFreshnessDeadline(live, 3001), 1000 + budgetMs - 500 + 1);
  assert.equal(liveReplicaFreshnessDeadline(live, 3501), null);
  assert.equal(liveReplicaFreshnessDeadline(null, 1000), null);
});

test('a missing goal or path has no freshness deadline', () => {
  const frame = parseLiveReplicaFrame({ ...payload(), robots: [{ ...payload().robots[0], goal: null }] });
  frame.robots[0].freshness = { pose_s: 0.5, goal_s: 0, path_s: null };
  assert.equal(liveReplicaFreshnessDeadline({ frame, receivedAt: 0 }, 0), LIVE_REPLICA_FRESHNESS_BUDGET_S * 1000 - 500 + 1);
});
