import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  optimizedScopeLabel,
  rankGlobalOptimizedScopes,
  selectGlobalOptimizedScope,
  unmergedRobotIds,
  type OptimizedScope
} from '../src/lib/stores/optimizedScopes.ts';

const session = '12345678-1234-4234-8234-567812345678';

function scope(name: string, robots: string[], width = 10, height = 10): OptimizedScope {
  return { scope: name, robots, resolution: 0.2, width, height, origin: { x: 0, y: 0 } };
}

test('a verified two-robot component outranks a larger deployment composite', () => {
  const ranked = rankGlobalOptimizedScopes([
    scope(`deployment:${session}`, ['robot_0', 'robot_1', 'robot_2'], 500, 500),
    scope('component:0', ['robot_0', 'robot_1'], 20, 20)
  ]);
  assert.deepEqual(
    ranked.map((entry) => entry.scope),
    ['component:0', `deployment:${session}`]
  );
});

test('with merges off only the deployment composite qualifies for the global view', () => {
  const scopes = [
    scope('robot:robot_0', ['robot_0']),
    scope('robot:robot_1', ['robot_1']),
    scope('component:0', ['robot_0']),
    scope('component:1', ['robot_1']),
    scope(`deployment:${session}`, ['robot_0', 'robot_1'])
  ];
  assert.equal(selectGlobalOptimizedScope(scopes)?.scope, `deployment:${session}`);
});

test('single-robot and unknown scopes never reach the global view', () => {
  assert.equal(selectGlobalOptimizedScope([scope('component:0', ['robot_0'])]), undefined);
  assert.equal(selectGlobalOptimizedScope([scope(`deployment:${session}`, ['robot_0'])]), undefined);
  assert.equal(selectGlobalOptimizedScope([scope('fleet:x', ['robot_0', 'robot_1'])]), undefined);
  assert.equal(selectGlobalOptimizedScope([]), undefined);
});

test('components are ordered by robots, then grid area, then name, unchanged', () => {
  const ranked = rankGlobalOptimizedScopes([
    scope('component:2', ['robot_0', 'robot_1'], 10, 10),
    scope('component:1', ['robot_0', 'robot_1'], 10, 10),
    scope('component:3', ['robot_0', 'robot_1'], 30, 30),
    scope('component:0', ['robot_0', 'robot_1', 'robot_2'], 5, 5)
  ]);
  assert.deepEqual(
    ranked.map((entry) => entry.scope),
    ['component:0', 'component:3', 'component:1', 'component:2']
  );
});

test('the unmerged hint skips the members of a displayed deployment composite', () => {
  const scopes = [
    scope('robot:robot_0', ['robot_0']),
    scope('component:0', ['robot_0']),
    scope('component:1', ['robot_1']),
    scope('component:2', ['robot_2']),
    scope('component:3', ['robot_3', 'robot_4']),
    scope(`deployment:${session}`, ['robot_0', 'robot_1'])
  ];
  // No fleet map on show, or a verified component on show: every lone robot.
  assert.deepEqual(unmergedRobotIds(scopes, null), ['robot_0', 'robot_1', 'robot_2']);
  assert.deepEqual(unmergedRobotIds(scopes, 'component:3'), ['robot_0', 'robot_1', 'robot_2']);
  // The composite on show places robot_0 and robot_1; robot_2 is still off it.
  assert.deepEqual(unmergedRobotIds(scopes, `deployment:${session}`), ['robot_2']);
  // A composite that is not listed places nobody.
  assert.deepEqual(unmergedRobotIds(scopes, 'deployment:other'), ['robot_0', 'robot_1', 'robot_2']);
});

test('the composite is named by its role, other scopes verbatim', () => {
  assert.equal(optimizedScopeLabel(`deployment:${session}`), 'deployment composite');
  assert.equal(optimizedScopeLabel('component:0'), 'component:0');
  assert.equal(optimizedScopeLabel('robot:robot_0'), 'robot:robot_0');
});
