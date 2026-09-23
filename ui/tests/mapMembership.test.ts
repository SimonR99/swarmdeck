import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  globalMapMembers,
  projectRobotToRaster,
  type FrameTransforms
} from '../src/lib/components/map2d/mapFrames.ts';
import type { RobotState } from '../src/lib/types/protocol.ts';

/*
 * Which robots the 2D canvas and the 3D scene draw. The two views used to
 * decide this separately; these cases pin what each decided before the rule
 * was shared, using the views' own code as it stood.
 */

function robot(robot_id: string, navigation_transform?: { x: number; y: number; yaw: number }): RobotState {
  return {
    robot_id,
    pose: { x: 1, y: 2, yaw: 0 },
    goal: null,
    planned_path: [],
    ...(navigation_transform ? { navigation_transform } : {})
  } as unknown as RobotState;
}

interface ViewInput {
  viewMode: 'global' | 'local';
  viewRobot: string | null;
  robots: RobotState[];
  disabled?: string[];
  transforms?: FrameTransforms;
  optimizedRobots?: string[] | null;
  globalMembers?: string[] | null;
}

const enabled = (input: ViewInput) => (id: string) => !(input.disabled ?? []).includes(id);

/** MapView.svelte's robotsOnMap at 20279c1, with the stores passed in. */
function legacy2D(input: ViewInput): string[] {
  const isEnabled = enabled(input);
  const fleetRobots = input.robots.filter((r) => isEnabled(r.robot_id));
  const project = (r: RobotState) => projectRobotToRaster(r, input.transforms);
  if (input.viewMode === 'local' && input.viewRobot) {
    if (!isEnabled(input.viewRobot)) return [];
    const r = input.robots.find((candidate) => candidate.robot_id === input.viewRobot);
    const shown = r ? project(r) : null;
    return shown ? [shown.robot_id] : [];
  }
  const members = globalMapMembers({
    showingOptimizedGrid: true,
    optimizedRobots: input.optimizedRobots,
    transforms: input.transforms,
    globalMembers: input.globalMembers
  });
  if (members.length === 0) return [];
  return fleetRobots
    .filter((r) => members.includes(r.robot_id) && isEnabled(r.robot_id))
    .map(project)
    .filter((r): r is RobotState => r !== null)
    .map((r) => r.robot_id);
}

/** Map3D.svelte's robotsOnMap at 20279c1 without a replica selected. */
function legacy3D(input: ViewInput): string[] {
  const isEnabled = enabled(input);
  const fleetRobots = input.robots.filter((r) => isEnabled(r.robot_id));
  if (input.viewMode === 'local' && input.viewRobot) {
    if (!isEnabled(input.viewRobot)) return [];
    const r = input.robots.find((candidate) => candidate.robot_id === input.viewRobot);
    return r ? [r.robot_id] : [];
  }
  const members = input.globalMembers;
  if (members && members.length > 0) {
    return fleetRobots.filter((r) => members.includes(r.robot_id) && isEnabled(r.robot_id)).map((r) => r.robot_id);
  }
  return fleetRobots.filter((r) => isEnabled(r.robot_id)).map((r) => r.robot_id);
}

interface TacticalInput {
  frameRobots: { robot_id: string; fresh: boolean }[];
  disabled?: string[];
  scope: 'robot' | 'fleet';
  robotId: string;
}

/** Map3D.svelte's robotsOnMap at 20279c1 on a live replica frame. */
function legacyTactical(input: TacticalInput): string[] {
  const isEnabled = (id: string) => !(input.disabled ?? []).includes(id);
  return input.frameRobots
    .filter((r) => r.fresh)
    .filter((r) => isEnabled(r.robot_id))
    .filter((r) => input.scope !== 'robot' || r.robot_id === input.robotId)
    .map((r) => r.robot_id);
}

const fleet3 = [robot('r0'), robot('r1'), robot('r2')];
const placed = { r0: { x: 0, y: 0, yaw: 0 }, r1: { x: 1, y: 0, yaw: 0 } };

const viewCases: { name: string; input: ViewInput; view2D: string[]; view3D: string[] }[] = [
  {
    name: 'a local view shows its robot alone',
    input: { viewMode: 'local', viewRobot: 'r1', robots: fleet3, globalMembers: ['r0'] },
    view2D: ['r1'],
    view3D: ['r1']
  },
  {
    name: 'a local view of a disabled robot shows nobody',
    input: { viewMode: 'local', viewRobot: 'r1', robots: fleet3, disabled: ['r1'] },
    view2D: [],
    view3D: []
  },
  {
    name: 'a local view of an unknown robot shows nobody',
    input: { viewMode: 'local', viewRobot: 'r9', robots: fleet3 },
    view2D: [],
    view3D: []
  },
  {
    name: 'a local view the raster cannot place shows nobody in 2D',
    input: {
      viewMode: 'local',
      viewRobot: 'r1',
      robots: [robot('r0'), robot('r1', { x: 0, y: 0, yaw: 0 })],
      transforms: {}
    },
    view2D: [],
    view3D: ['r1']
  },
  {
    name: 'local mode without a robot is the global view',
    input: { viewMode: 'local', viewRobot: null, robots: fleet3, optimizedRobots: ['r2'], globalMembers: ['r0'] },
    view2D: ['r2'],
    view3D: ['r0']
  },
  {
    name: 'the global raster shows its catalogue scope in fleet order',
    input: { viewMode: 'global', viewRobot: null, robots: fleet3, optimizedRobots: ['r2', 'r0'], globalMembers: ['r1'] },
    view2D: ['r0', 'r2'],
    view3D: ['r1']
  },
  {
    name: 'without a scope the raster shows who its transforms placed',
    input: { viewMode: 'global', viewRobot: null, robots: fleet3, optimizedRobots: [], transforms: placed },
    view2D: ['r0', 'r1'],
    view3D: ['r0', 'r1', 'r2']
  },
  {
    name: 'nothing known places nobody in 2D and everybody in 3D',
    input: { viewMode: 'global', viewRobot: null, robots: fleet3 },
    view2D: [],
    view3D: ['r0', 'r1', 'r2']
  },
  {
    name: 'disabled robots are never drawn globally',
    input: {
      viewMode: 'global', viewRobot: null, robots: fleet3, disabled: ['r0'],
      optimizedRobots: ['r0', 'r1'], globalMembers: ['r0', 'r2']
    },
    view2D: ['r1'],
    view3D: ['r2']
  },
  {
    name: 'a member the raster cannot place is dropped in 2D only',
    input: {
      viewMode: 'global', viewRobot: null,
      robots: [robot('r0', { x: 0, y: 0, yaw: 0 }), robot('r1', { x: 0, y: 0, yaw: 0 })],
      optimizedRobots: ['r0', 'r1'], transforms: { r0: { x: 0, y: 0, yaw: 0 } }, globalMembers: ['r0', 'r1']
    },
    view2D: ['r0'],
    view3D: ['r0', 'r1']
  }
];

const tacticalCases: { name: string; input: TacticalInput; expected: string[] }[] = [
  {
    name: 'a fleet replica shows every fresh enabled robot in frame order',
    input: {
      scope: 'fleet', robotId: 'fleet', disabled: ['r2'],
      frameRobots: [{ robot_id: 'r1', fresh: true }, { robot_id: 'r0', fresh: false },
        { robot_id: 'r2', fresh: true }, { robot_id: 'r3', fresh: true }]
    },
    expected: ['r1', 'r3']
  },
  {
    name: 'a robot replica shows only its robot',
    input: {
      scope: 'robot', robotId: 'r1',
      frameRobots: [{ robot_id: 'r0', fresh: true }, { robot_id: 'r1', fresh: true }]
    },
    expected: ['r1']
  },
  {
    name: 'a robot replica of a disabled robot shows nobody',
    input: { scope: 'robot', robotId: 'r1', disabled: ['r1'], frameRobots: [{ robot_id: 'r1', fresh: true }] },
    expected: []
  }
];

for (const { name, input, view2D, view3D } of viewCases) {
  test(`2D and 3D membership: ${name}`, () => {
    assert.deepEqual(legacy2D(input), view2D, '2D');
    assert.deepEqual(legacy3D(input), view3D, '3D');
  });
}

for (const { name, input, expected } of tacticalCases) {
  test(`live replica membership: ${name}`, () => {
    assert.deepEqual(legacyTactical(input), expected);
  });
}
