import { test } from 'node:test';
import assert from 'node:assert/strict';
import type { MapRobot } from '../src/lib/components/map2d/mapLayers.ts';

/*
 * Which route and goal the 2D canvas and the 3D scene draw for a robot. The
 * two views used to decide this separately; these cases pin what each decided
 * before the rule was shared, using the views' own code as it stood.
 */

type Path = { x: number; y: number; z?: number }[];
interface Drawn { global: Path | null; local: Path | null; goal: MapRobot['goal'] | null }

/** mapLayers.ts drawRobots at 20279c1, with showPlans on. */
function legacy2D(robot: MapRobot): Drawn {
  const isNavActive = robot.nav_status === 'active' || robot.mode === 'nav' || Boolean(robot.goal);
  const hasSplitPaths =
    Boolean((robot.global_planned_path && robot.global_planned_path.length > 0) ||
    (robot.local_planned_path && robot.local_planned_path.length > 0));
  const globalPath = isNavActive && hasSplitPaths
    ? (robot.global_planned_path && robot.global_planned_path.length > 0 ? robot.global_planned_path : robot.planned_path)
    : isNavActive ? robot.planned_path : undefined;
  const localPath = isNavActive && robot.local_planned_path && robot.local_planned_path.length > 0 ? robot.local_planned_path : undefined;
  const drawn = (path: Path | undefined) => (isNavActive && path && path.length >= 2 ? path : null);
  return {
    global: drawn(globalPath),
    local: drawn(localPath),
    goal: isNavActive && robot.goal ? robot.goal : null
  };
}

/** map3dLayers.ts updatePaths and updateGoals at 20279c1, with showPlans on. */
function legacy3D(robot: MapRobot): Drawn {
  const isNavActive = robot.nav_status === 'active' || robot.mode === 'nav' || Boolean(robot.goal);
  if (!isNavActive) return { global: null, local: null, goal: null };
  const hasSplitPaths = Boolean(
    (robot.global_planned_path && robot.global_planned_path.length > 0) ||
    (robot.local_planned_path && robot.local_planned_path.length > 0)
  );
  const globalPath = hasSplitPaths
    ? robot.global_planned_path && robot.global_planned_path.length > 0
      ? robot.global_planned_path
      : robot.planned_path
    : robot.planned_path;
  const localPath =
    robot.local_planned_path && robot.local_planned_path.length > 0 ? robot.local_planned_path : undefined;
  return {
    global: globalPath && globalPath.length >= 2 ? globalPath : null,
    local: localPath && localPath.length >= 2 ? localPath : null,
    goal: robot.goal ? robot.goal : null
  };
}

const planned = [{ x: 0, y: 0 }, { x: 1, y: 0 }];
const globalRoute = [{ x: 0, y: 0, z: 0.1 }, { x: 2, y: 0, z: 0.2 }, { x: 3, y: 1 }];
const localRoute = [{ x: 0, y: 0 }, { x: 0.5, y: 0.1 }];
const goal = { x: 3, y: 1 };
const base: MapRobot = { robot_id: 'r0', pose: { x: 0, y: 0, yaw: 0 } };

const routeCases: { name: string; robot: MapRobot; expected: Drawn }[] = [
  {
    name: 'an idle robot draws nothing, even with a stale route',
    robot: { ...base, nav_status: 'idle', mode: 'teleop', planned_path: planned, global_planned_path: globalRoute },
    expected: { global: null, local: null, goal: null }
  },
  {
    name: 'an active robot without split routes draws its planned path',
    robot: { ...base, nav_status: 'active', planned_path: planned },
    expected: { global: planned, local: null, goal: null }
  },
  {
    name: 'nav mode counts as navigating',
    robot: { ...base, mode: 'nav', planned_path: planned },
    expected: { global: planned, local: null, goal: null }
  },
  {
    name: 'a goal alone counts as navigating',
    robot: { ...base, goal, planned_path: [] },
    expected: { global: null, local: null, goal }
  },
  {
    name: 'the global route wins over the planned path',
    robot: { ...base, goal, planned_path: planned, global_planned_path: globalRoute, local_planned_path: localRoute },
    expected: { global: globalRoute, local: localRoute, goal }
  },
  {
    name: 'a local route alone keeps the planned path as the global one',
    robot: { ...base, goal, planned_path: planned, global_planned_path: [], local_planned_path: localRoute },
    expected: { global: planned, local: localRoute, goal }
  },
  {
    name: 'a single-vertex route is not drawn',
    robot: { ...base, nav_status: 'active', planned_path: [{ x: 0, y: 0 }], local_planned_path: [{ x: 1, y: 1 }] },
    expected: { global: null, local: null, goal: null }
  },
  {
    name: 'an absent planned path draws no global route',
    robot: { ...base, nav_status: 'active' },
    expected: { global: null, local: null, goal: null }
  }
];

for (const { name, robot, expected } of routeCases) {
  test(`2D and 3D routes: ${name}`, () => {
    assert.deepEqual(legacy2D(robot), expected, '2D');
    assert.deepEqual(legacy3D(robot), expected, '3D');
  });
}
