import { test } from 'node:test';
import assert from 'node:assert/strict';
import { projectRobotToRaster, type FrameTransforms } from '../src/lib/components/map2d/mapFrames.ts';
import { globalMapMembers, localRobotOf, membersOnMap } from '../src/lib/components/map/mapMembership.ts';
import type { RobotState } from '../src/lib/types/protocol.ts';

/*
 * Which robots the 2D canvas and the 3D scene draw. Both take the global
 * map's members from one source, the displayed map (`mapStore.globalMapMembers`),
 * and with no merge information both show every enabled robot. Before this was
 * shared the 2D canvas showed nobody in that case and the 3D scene read the
 * SLAM merge instead; the operator chose the 3D behaviour, since hiding robots
 * is the worse failure. The one remaining difference is that the 2D canvas
 * cannot draw a robot its raster has no transform for.
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

/** mapStore.globalMapMembers with the stores passed in. */
function displayedMembers(input: ViewInput) {
  return globalMapMembers({
    showingOptimizedGrid: true,
    optimizedRobots: input.optimizedRobots,
    transforms: input.transforms,
    globalMembers: input.globalMembers
  });
}

/** The robots either view has on its map, before the 2D canvas projects them. */
function sharedMembers(input: ViewInput): RobotState[] {
  return membersOnMap(input.robots, {
    localRobot: localRobotOf(input.viewMode, input.viewRobot),
    members: displayedMembers(input),
    isEnabled: enabled(input)
  });
}

/** MapView.svelte's robotsOnMap: the shared members its raster can place. */
function view2D(input: ViewInput): string[] {
  return sharedMembers(input)
    .map((r) => projectRobotToRaster(r, input.transforms))
    .filter((r): r is RobotState => r !== null)
    .map((r) => r.robot_id);
}

/** Map3D.svelte's robotsOnMap without a replica selected. */
function view3D(input: ViewInput): string[] {
  return sharedMembers(input).map((r) => r.robot_id);
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

/** Both views draw `shown`, except where the 2D raster cannot place a member. */
const viewCases: { name: string; input: ViewInput; shown: string[]; only2D?: string[] }[] = [
  {
    name: 'a local view shows its robot alone',
    input: { viewMode: 'local', viewRobot: 'r1', robots: fleet3, globalMembers: ['r0'] },
    shown: ['r1']
  },
  {
    name: 'a local view of a disabled robot shows nobody',
    input: { viewMode: 'local', viewRobot: 'r1', robots: fleet3, disabled: ['r1'] },
    shown: []
  },
  {
    name: 'a local view of an unknown robot shows nobody',
    input: { viewMode: 'local', viewRobot: 'r9', robots: fleet3 },
    shown: []
  },
  {
    name: 'local mode without a robot is the global view',
    input: { viewMode: 'local', viewRobot: null, robots: fleet3, optimizedRobots: ['r2'], globalMembers: ['r0'] },
    shown: ['r2']
  },
  {
    name: 'the global map shows its catalogue scope in fleet order',
    input: { viewMode: 'global', viewRobot: null, robots: fleet3, optimizedRobots: ['r2', 'r0'], globalMembers: ['r1'] },
    shown: ['r0', 'r2']
  },
  {
    name: 'without a scope the global map shows who its transforms placed',
    input: { viewMode: 'global', viewRobot: null, robots: fleet3, optimizedRobots: [], transforms: placed },
    shown: ['r0', 'r1']
  },
  {
    name: 'nothing known places every robot',
    input: { viewMode: 'global', viewRobot: null, robots: fleet3 },
    shown: ['r0', 'r1', 'r2']
  },
  {
    name: 'empty merge information places every robot',
    input: { viewMode: 'global', viewRobot: null, robots: fleet3, optimizedRobots: [], transforms: {}, globalMembers: [] },
    shown: ['r0', 'r1', 'r2']
  },
  {
    name: 'the SLAM merge alone does not name the displayed map\'s members',
    input: { viewMode: 'global', viewRobot: null, robots: fleet3, globalMembers: ['r1'] },
    shown: ['r0', 'r1', 'r2']
  },
  {
    name: 'disabled robots are never drawn globally',
    input: {
      viewMode: 'global', viewRobot: null, robots: fleet3, disabled: ['r0'],
      optimizedRobots: ['r0', 'r1'], globalMembers: ['r0', 'r2']
    },
    shown: ['r1']
  },
  {
    name: 'disabled robots are never drawn when nothing is known',
    input: { viewMode: 'global', viewRobot: null, robots: fleet3, disabled: ['r2'] },
    shown: ['r0', 'r1']
  }
];

/* The exception: the canvas cannot draw a robot its raster has no transform for. */
const rasterCases: typeof viewCases = [
  {
    name: 'a local view the raster cannot place',
    input: {
      viewMode: 'local',
      viewRobot: 'r1',
      robots: [robot('r0'), robot('r1', { x: 0, y: 0, yaw: 0 })],
      transforms: {}
    },
    shown: ['r1'],
    only2D: []
  },
  {
    name: 'a member the raster cannot place',
    input: {
      viewMode: 'global', viewRobot: null,
      robots: [robot('r0', { x: 0, y: 0, yaw: 0 }), robot('r1', { x: 0, y: 0, yaw: 0 })],
      optimizedRobots: ['r0', 'r1'], transforms: { r0: { x: 0, y: 0, yaw: 0 } }, globalMembers: ['r0', 'r1']
    },
    shown: ['r0', 'r1'],
    only2D: ['r0']
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

/** Map3D.svelte's robotsOnMap on a live replica frame. */
function sharedTactical(input: TacticalInput): string[] {
  return membersOnMap(input.frameRobots.filter((r) => r.fresh), {
    localRobot: input.scope === 'robot' ? input.robotId : null,
    members: null,
    isEnabled: (id) => !(input.disabled ?? []).includes(id)
  }).map((r) => r.robot_id);
}

for (const { name, input, shown } of viewCases) {
  test(`2D and 3D show the same robots: ${name}`, () => {
    assert.deepEqual(view2D(input), shown, '2D');
    assert.deepEqual(view3D(input), shown, '3D');
  });
}

for (const { name, input, shown, only2D } of rasterCases) {
  test(`only the 2D canvas drops a robot it cannot place: ${name}`, () => {
    assert.deepEqual(view3D(input), shown, '3D');
    assert.deepEqual(view2D(input), only2D, '2D');
  });
}

for (const { name, input, expected } of tacticalCases) {
  test(`live replica membership: ${name}`, () => {
    assert.deepEqual(legacyTactical(input), expected);
    assert.deepEqual(sharedTactical(input), expected, 'shared');
  });
}
