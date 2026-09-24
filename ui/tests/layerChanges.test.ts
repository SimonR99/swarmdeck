import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  LayerDependencies,
  goalDependencies,
  loopDependencies,
  pathDependencies,
  trailDependencies
} from '../src/lib/components/map3d/layerChanges.ts';
import type { MapRobot } from '../src/lib/components/map/mapRobot.ts';

function robot(overrides: Partial<MapRobot> = {}): MapRobot {
  return {
    robot_id: 'robot_0',
    pose: { x: 1, y: 2, yaw: 0 },
    planned_path: [],
    goal: null,
    nav_status: 'idle',
    mode: 'idle',
    ...overrides
  } as MapRobot;
}

test('a layer rebuilds on the first update and then only when a value changes', () => {
  const deps = new LayerDependencies();
  assert.equal(deps.changed('goals', [true, 1]), true);
  assert.equal(deps.changed('goals', [true, 1]), false);
  assert.equal(deps.changed('goals', [false, 1]), true);
  assert.equal(deps.changed('goals', [false, 1, 2]), true);
});

test('each layer keeps its own dependencies', () => {
  const deps = new LayerDependencies();
  deps.changed('goals', [1]);
  assert.equal(deps.changed('paths', [1]), true);
  assert.equal(deps.changed('goals', [1]), false);
});

test('invalidation forces every layer to rebuild once', () => {
  const deps = new LayerDependencies();
  deps.changed('goals', [1]);
  deps.clear();
  assert.equal(deps.changed('goals', [1]), true);
});

// The fleet store hands back the same array while the selection is unchanged.
const noSelection: string[] = [];

test('a repeated pose leaves goals, paths and closures alone', () => {
  const path = [{ x: 0, y: 0 }];
  const before = [robot({ planned_path: path })];
  const graphs = {};
  const deps = new LayerDependencies();
  deps.changed('goals', goalDependencies(before, true));
  deps.changed('paths', pathDependencies(before, true, noSelection));
  deps.changed('loops', loopDependencies(before, true, graphs));

  // The store hands back the same references when telemetry repeats itself.
  const after = [robot({ planned_path: path })];
  assert.equal(deps.changed('goals', goalDependencies(after, true)), false);
  assert.equal(deps.changed('paths', pathDependencies(after, true, noSelection)), false);
  assert.equal(deps.changed('loops', loopDependencies(after, true, graphs)), false);
});

test('a moved robot rebuilds its goal guide and closure lines but not its route', () => {
  const path = [{ x: 0, y: 0 }];
  const graphs = {};
  const deps = new LayerDependencies();
  const before = [robot({ planned_path: path })];
  deps.changed('goals', goalDependencies(before, true));
  deps.changed('paths', pathDependencies(before, true, noSelection));
  deps.changed('loops', loopDependencies(before, true, graphs));

  const moved = [robot({ pose: { x: 9, y: 2, yaw: 0 }, planned_path: path })];
  assert.equal(deps.changed('goals', goalDependencies(moved, true)), true);
  assert.equal(deps.changed('paths', pathDependencies(moved, true, noSelection)), false);
  assert.equal(deps.changed('loops', loopDependencies(moved, true, graphs)), true);
});

test('a new planner route replaces the drawn route', () => {
  const deps = new LayerDependencies();
  deps.changed('paths', pathDependencies([robot({ planned_path: [] })], true, noSelection));
  const replanned = [robot({ planned_path: [{ x: 1, y: 1 }] })];
  assert.equal(deps.changed('paths', pathDependencies(replanned, true, noSelection)), true);
});

test('a trail is redrawn when a point is appended, including at its length cap', () => {
  const deps = new LayerDependencies();
  const robots = [robot()];
  const trail = [{ x: 0, y: 0 }, { x: 1, y: 0 }];
  const trails = new Map([['robot_0', trail]]);
  assert.equal(deps.changed('trails', trailDependencies(robots, trails, true)), true);
  assert.equal(deps.changed('trails', trailDependencies(robots, trails, true)), false);

  trail.push({ x: 2, y: 0 });
  assert.equal(deps.changed('trails', trailDependencies(robots, trails, true)), true);

  // At the cap the trail drops its oldest point as it takes a new one, so the
  // length alone would report no change.
  trail.shift();
  trail.push({ x: 3, y: 0 });
  assert.equal(deps.changed('trails', trailDependencies(robots, trails, true)), true);

  trails.delete('robot_0');
  assert.equal(deps.changed('trails', trailDependencies(robots, trails, true)), true);
});

test('changing the selection restyles the routes', () => {
  const deps = new LayerDependencies();
  const robots = [robot()];
  deps.changed('paths', pathDependencies(robots, true, noSelection));
  assert.equal(deps.changed('paths', pathDependencies(robots, true, ['robot_0'])), true);
});
