import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  CanvasInteraction,
  pickRobot,
  qualifiedNavigateTargets,
  type CanvasInteractionHost,
  type ReviewedObjectHit
} from '../src/lib/components/map2d/canvasInteraction.ts';
import { CanvasViewport } from '../src/lib/components/map2d/canvasViewport.ts';
import type { RobotState } from '../src/lib/types/protocol.ts';

type XY = { x: number; y: number };
const origin = () => ({ x: 10, y: 20 });

/** Identity viewport and a world that is the grid: a world point is drawn at its own px. */
function harness({ goalMode = false, ...overrides }: Partial<Omit<CanvasInteractionHost, 'goalMode'>> & { goalMode?: boolean } = {}) {
  const calls: string[] = [];
  const log = (entry: string) => calls.push(entry);
  const viewport = new CanvasViewport({ scale: 1, tx: 0, ty: 0, rotation: 0, initialised: true });
  const state = { goalMode, reviewSelected: null as string | null };
  const host: CanvasInteractionHost = {
    goalMode: () => state.goalMode,
    robotsOnMap: () => [
      { robot_id: 'r0', pose: { x: 100, y: 100 } },
      { robot_id: 'r1', pose: { x: 110, y: 100 } }
    ],
    worldToGrid: (x, y) => ({ gx: x, gy: y }),
    gridToWorld: (gx, gy) => ({ x: gx, y: gy }),
    reviewedObjectAt: () => null,
    reviewSelected: () => state.reviewSelected,
    selectReview: (id) => log(`review ${id}`),
    focusRobot: (id) => log(`focus ${id}`),
    selectRobot: (id, additive) => log(`select ${id}${additive ? ' +' : ''}`),
    goalTargets: () => ['r0', 'r1'],
    goalScope: () => 'component:ab',
    sendGoal: async (id, scope, world) => log(`goal ${id} ${scope} ${world.x},${world.y}`),
    goalRefused: (id, reason) => log(`refused ${id} ${(reason as Error).message}`),
    cancelGoalMode: () => log('cancel'),
    finishGoal: (world) => log(`finish ${world.x},${world.y}`),
    stopFollowing: () => log('stop following'),
    setCursor: (world) => log(`cursor ${world?.x},${world?.y}`),
    ...overrides
  };
  const interaction = new CanvasInteraction(viewport, host);
  const click = (at: XY, additive = false) => {
    interaction.pointerDown(1, at);
    interaction.pointerUp(1, at, origin, additive);
  };
  return { calls, viewport, state, interaction, click };
}

const settle = () => new Promise((resolve) => setImmediate(resolve));

test('a click picks the nearest robot within the pick radius', () => {
  const { calls, click } = harness();
  click({ x: 10 + 107, y: 20 + 100 });
  click({ x: 10 + 101, y: 20 + 101 }, true);
  click({ x: 10 + 100, y: 20 + 140 });
  assert.deepEqual(calls, ['select r1', 'select r0 +']);
});

test('robot picking skips robots the raster cannot place and respects the radius', () => {
  const robots = [{ robot_id: 'a', pose: { x: 0, y: 0 } }, { robot_id: 'b', pose: { x: 30, y: 0 } }];
  const screen = (x: number, y: number) => (x === 0 ? null : { sx: x, sy: y });
  assert.equal(pickRobot(robots, { x: 1, y: 0 }, screen), null);
  assert.equal(pickRobot(robots, { x: 12, y: 0 }, screen), 'b');
  assert.equal(pickRobot(robots, { x: 11.9, y: 0 }, screen), null);
});

test('a reviewed object under the click wins over robots and toggles its selection', () => {
  const hit: ReviewedObjectHit = { id: 'det1', robotId: null, robotIds: ['r2'] };
  const { calls, click, state } = harness({ reviewedObjectAt: () => hit });
  click({ x: 110, y: 120 });
  state.reviewSelected = 'det1';
  click({ x: 110, y: 120 });
  assert.deepEqual(calls, ['review det1', 'focus r2', 'review null']);
});

test('a drag pans and stops following instead of clicking', () => {
  const { calls, interaction, viewport } = harness();
  interaction.pointerDown(1, { x: 110, y: 120 });
  interaction.pointerMove(1, { x: 113, y: 120 }, origin);
  assert.deepEqual(calls, [], 'within the drag threshold');
  interaction.pointerMove(1, { x: 120, y: 125 }, origin);
  interaction.pointerUp(1, { x: 120, y: 125 }, origin, false);
  assert.deepEqual(calls, ['stop following']);
  assert.deepEqual([viewport.view.tx, viewport.view.ty], [10, 5]);
});

test('hovering reports the world point under the cursor', () => {
  const { calls, interaction } = harness();
  interaction.pointerMove(7, { x: 60, y: 70 }, origin);
  interaction.pointerMove(7, { x: 60, y: 70 }, () => null);
  assert.deepEqual(calls, ['cursor 50,50']);
});

test('a second finger pinches, and lifting both ends the gesture without a click', () => {
  const { calls, interaction, viewport } = harness();
  interaction.pointerDown(1, { x: 100, y: 100 });
  interaction.pointerDown(2, { x: 200, y: 100 });
  interaction.pointerMove(2, { x: 300, y: 100 }, origin);
  assert.ok(Math.abs(viewport.view.scale - 2) < 1e-12);
  interaction.pointerUp(2, { x: 300, y: 100 }, origin, false);
  interaction.pointerUp(1, { x: 100, y: 100 }, origin, false);
  assert.deepEqual(calls, ['stop following']);
});

test('in goal mode a click sends the goal to every qualified robot and finishes', async () => {
  const { calls, click } = harness({ goalMode: true });
  click({ x: 60, y: 80 });
  await settle();
  assert.deepEqual(calls, ['goal r0 component:ab 50,60', 'goal r1 component:ab 50,60', 'finish 50,60']);
});

test('a refused goal is reported per robot', async () => {
  const { calls, click } = harness({
    goalMode: true,
    sendGoal: async (id) => {
      if (id === 'r1') throw new Error('no fresh telemetry');
    }
  });
  click({ x: 60, y: 80 });
  await settle();
  assert.deepEqual(calls, ['finish 50,60', 'refused r1 no fresh telemetry']);
});

test('goal mode without a shown scope cancels; without targets it stays armed', async () => {
  const noScope = harness({ goalMode: true, goalScope: () => null });
  noScope.click({ x: 60, y: 80 });
  assert.deepEqual(noScope.calls, ['cancel']);
  const noTargets = harness({ goalMode: true, goalTargets: () => [] });
  noTargets.click({ x: 60, y: 80 });
  await settle();
  assert.deepEqual(noTargets.calls, []);
  const offRaster = harness({ goalMode: true, gridToWorld: () => null });
  offRaster.click({ x: 60, y: 80 });
  assert.deepEqual(offRaster.calls, []);
});

test('goal qualification needs navigation and a raster frame for a transformed robot', () => {
  const robot = (id: string, transformed: boolean) =>
    ({ robot_id: id, ...(transformed ? { navigation_transform: { x: 0, y: 0, yaw: 0 } } : {}) }) as unknown as RobotState;
  const robots: Record<string, RobotState> = {
    plain: robot('plain', false),
    placed: robot('placed', true),
    unplaced: robot('unplaced', true),
    driver: robot('driver', false)
  };
  const targets = qualifiedNavigateTargets(
    ['plain', 'placed', 'unplaced', 'driver', 'unknown'],
    (id) => id !== 'driver',
    (id) => robots[id],
    { placed: { x: 1, y: 2, yaw: 0 } }
  );
  assert.deepEqual(targets, ['plain', 'placed', 'unknown']);
});
