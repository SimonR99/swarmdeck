import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  componentGoal,
  postGlobalRasterGoal,
  usesLiveComponentGoal
} from '../src/lib/components/map2d/globalGoal.ts';

test('only a verified component raster in the global view takes the live goal path', () => {
  assert.equal(usesLiveComponentGoal('global', 'component:abc'), true);
  assert.equal(usesLiveComponentGoal('global', 'deployment:session'), false);
  assert.equal(usesLiveComponentGoal('global', null), false);
  assert.equal(usesLiveComponentGoal('local', 'component:abc'), false);
});

test('a raster click becomes a component-frame goal at the robot height, heading toward it', () => {
  // Navigation frame translated by (10, -2, 0.3) into the component frame.
  const robot = {
    robot_id: 'robot_2',
    pose: { x: 1, y: 1, z: 0.1, yaw: 0 },
    T_component_navigation: [
      [1, 0, 0, 10],
      [0, 1, 0, -2],
      [0, 0, 1, 0.3],
      [0, 0, 0, 1]
    ]
  };
  const goal = componentGoal(robot, { x: 11, y: 4 });
  assert.equal(goal.x, 11);
  assert.equal(goal.y, 4);
  assert.ok(Math.abs(goal.z - 0.4) < 1e-9);
  // Robot at (11, -1) in the component frame, goal straight up in y.
  assert.ok(Math.abs(goal.yaw - Math.PI / 2) < 1e-9);
  // A flat 16-element matrix reads the same.
  const flat = { ...robot, T_component_navigation: robot.T_component_navigation.flat() };
  assert.deepEqual(componentGoal(flat, { x: 11, y: 4 }), goal);
});

test('the goal is posted with the displayed solution order and component id', async () => {
  const calls: { url: string; body?: unknown }[] = [];
  const fetchImpl = (async (input: string | URL | Request, init?: RequestInit) => {
    const url = String(input);
    calls.push({ url, body: init?.body ? JSON.parse(String(init.body)) : undefined });
    const json = url.endsWith('/components')
      ? { active_session_id: 'mission-1' }
      : url.includes('/goal')
        ? { ok: true }
        : {
            session_id: 'mission-1',
            solution_order: [42, 0],
            robots: [
              {
                robot_id: 'robot_2',
                pose: { x: 0, y: 0, z: 0, yaw: 0 },
                T_component_navigation: [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
              }
            ]
          };
    return new Response(JSON.stringify(json), { status: 200 });
  }) as typeof fetch;
  // postLiveReplicaGoal uses the global fetch; route it through the same fake.
  const original = globalThis.fetch;
  globalThis.fetch = fetchImpl;
  try {
    await postGlobalRasterGoal('robot_2', 'component:abc', { x: 3, y: 4 }, fetchImpl);
  } finally {
    globalThis.fetch = original;
  }
  const posted = calls.find((call) => call.url.endsWith('/goal'));
  assert.ok(posted);
  assert.equal(posted.url, '/api/autonomy/replicas/components/live/mission-1/goal');
  assert.deepEqual(posted.body, {
    robot_id: 'robot_2',
    component_id: 'component:abc',
    solution_order: [42, 0],
    goal: { x: 3, y: 4, z: 0, yaw: Math.atan2(4, 3) }
  });
});
