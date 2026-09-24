import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  liveGoalTargets,
  SceneInteraction,
  type InteractionScene,
  type LiveGoalContext,
  type SceneInteractionHost
} from '../src/lib/components/map3d/sceneInteraction.ts';

type Live = LiveGoalContext<string>;

interface Options {
  goalMode?: boolean;
  replica?: 'none' | 'live' | 'readonly';
  solutionOrderKnown?: boolean;
  ground?: { x: number; y: number; z: number } | null;
  robotAt?: string | null;
  detectionAt?: string | null;
  detectionSelected?: string | null;
  selected?: string[];
  drawn?: string[];
  refuse?: string;
}

function harness(options: Options = {}) {
  const calls: string[] = [];
  const log = (entry: string) => calls.push(entry);
  const reticlePositions: number[][] = [];
  const scene: InteractionScene = {
    yaw: 0,
    pitch: 1,
    panBy: (dx, dy) => log(`pan ${dx},${dy}`),
    groundAt: () => (options.ground === undefined ? { x: 1, y: 2, z: 0.5 } : options.ground),
    robotAt: () => options.robotAt ?? null,
    detectionAt: () => options.detectionAt ?? null,
    reticle: { visible: false, position: { set: (x, y, z) => reticlePositions.push([x, y, z]) } }
  };
  const replica = options.replica ?? 'none';
  const host: SceneInteractionHost<string> = {
    scene: () => scene,
    requestRender: () => log('render'),
    cameraMoved: () => log('camera'),
    setDragging: (dragging) => log(`dragging ${dragging}`),
    setCursor: (point, planar) => log(`cursor ${point ? `${point.x},${point.y},${point.z}` : 'none'} ${planar ? 'planar' : '-'}`),
    goalMode: () => options.goalMode ?? false,
    replicaShown: () => replica !== 'none',
    liveReplica: () =>
      replica === 'live'
        ? { selection: 'component:ab', solutionOrder: [2, 7], solutionOrderKnown: options.solutionOrderKnown ?? true }
        : null,
    drawnRobotIds: () => options.drawn ?? ['r0', 'r1'],
    selected: () => options.selected ?? ['r0', 'r1'],
    canNavigate: (id) => id !== 'r1',
    sendGoal: async (live: Live, id, goal) => {
      if (id === options.refuse) throw new Error('stale solution order');
      log(`goal ${live.selection} ${id} ${goal.x},${goal.y},${goal.z},${goal.yaw}`);
    },
    goalFailed: (message) => log(`failed ${message}`),
    cancelGoalMode: () => log('cancel'),
    finishGoal: (goal) => log(`finish ${goal.x},${goal.y}`),
    selectRobot: (id, additive) => log(`select ${id}${additive ? ' +' : ''}`),
    selectDetection: (id) => log(`detection ${id}`),
    detectionSelected: () => options.detectionSelected ?? null
  };
  const interaction = new SceneInteraction(host);
  const ndc = () => ({ x: 0, y: 0 });
  const click = async (additive = false) => {
    interaction.pointerDown({ x: 50, y: 50 }, 0, false);
    await interaction.pointerUp(0, false, ndc, additive);
  };
  return { calls, scene, reticlePositions, interaction, click };
}

test('a left-drag orbits, a right-drag pans, and neither clicks', async () => {
  const orbit = harness({ robotAt: 'r0' });
  orbit.interaction.pointerDown({ x: 0, y: 0 }, 0, false);
  orbit.interaction.pointerMove({ x: 3, y: 0 }, { x: 0, y: 0 });
  orbit.interaction.pointerMove({ x: 10, y: 100 }, { x: 0, y: 0 });
  await orbit.interaction.pointerUp(0, false, () => ({ x: 0, y: 0 }), false);
  assert.deepEqual(orbit.calls, ['dragging true', 'render', 'camera', 'render', 'dragging false']);
  assert.ok(Math.abs(orbit.scene.yaw - -0.07) < 1e-12);
  assert.equal(orbit.scene.pitch, 1.48, 'pitch is clamped');

  const pan = harness();
  pan.interaction.pointerDown({ x: 0, y: 0 }, 2, false);
  pan.interaction.pointerMove({ x: 10, y: 5 }, { x: 0, y: 0 });
  assert.deepEqual(pan.calls, ['dragging true', 'camera', 'pan -8,-4', 'render']);
});

test('hovering moves the cursor, and the reticle only in goal mode on a live replica', () => {
  const plain = harness();
  plain.interaction.pointerMove({ x: 1, y: 1 }, { x: 0, y: 0 });
  assert.deepEqual(plain.calls, ['cursor 1,2,0.5 planar'], 'a still scene is not redrawn for a hover');

  const live = harness({ goalMode: true, replica: 'live' });
  live.interaction.pointerMove({ x: 1, y: 1 }, { x: 0, y: 0 });
  assert.deepEqual(live.calls, ['cursor 1,2,0.5 -', 'render']);
  assert.deepEqual(live.reticlePositions, [[1, 2, 0.515]]);

  const off = harness({ goalMode: true, replica: 'live', ground: null });
  off.scene.reticle.visible = true;
  off.interaction.pointerMove({ x: 1, y: 1 }, { x: 0, y: 0 });
  assert.deepEqual(off.calls, ['cursor none -', 'render']);
  assert.equal(off.scene.reticle.visible, false);
});

test('a click selects the robot under it, else a detection, else clears the detection', async () => {
  const robot = harness({ robotAt: 'r2', detectionAt: 'det1' });
  await robot.click(true);
  assert.deepEqual(robot.calls.slice(2), ['select r2 +', 'render']);
  const detection = harness({ detectionAt: 'det1' });
  await detection.click();
  assert.deepEqual(detection.calls.slice(2), ['detection det1', 'render']);
  const empty = harness({ detectionSelected: 'det1' });
  await empty.click();
  assert.deepEqual(empty.calls.slice(2), ['detection null']);
});

test('a read-only replica ignores clicks', async () => {
  const { calls, click } = harness({ replica: 'readonly', robotAt: 'r0' });
  await click();
  assert.deepEqual(calls, ['dragging true', 'dragging false']);
});

test('a goal on a live replica goes to the drawn robots that can navigate, then finishes', async () => {
  const { calls, click, scene } = harness({ goalMode: true, replica: 'live', selected: ['r0', 'r1', 'r9'] });
  scene.reticle.visible = true;
  await click();
  assert.deepEqual(calls.slice(2), ['goal component:ab r0 1,2,0.5,0', 'finish 1,2', 'render']);
  assert.equal(scene.reticle.visible, false);
});

test('a refused goal reports the error and leaves goal mode', async () => {
  const { calls, click } = harness({ goalMode: true, replica: 'live', refuse: 'r0' });
  await click();
  assert.deepEqual(calls.slice(2), ['failed stale solution order', 'cancel', 'render']);
});

test('a goal without a known solution order or an eligible robot is cancelled and keeps the reticle', async () => {
  for (const options of [{ solutionOrderKnown: false }, { selected: ['r1'] }, { drawn: [] as string[] }]) {
    const { calls, click, scene } = harness({ goalMode: true, replica: 'live', ...options });
    scene.reticle.visible = true;
    await click();
    assert.deepEqual(calls.slice(2), ['cancel']);
    assert.equal(scene.reticle.visible, true);
  }
});

test('goal mode without a replica cancels; off the ground it falls through to picking', async () => {
  const none = harness({ goalMode: true, robotAt: 'r0' });
  await none.click();
  assert.deepEqual(none.calls.slice(2), ['cancel', 'render']);
  const sky = harness({ goalMode: true, replica: 'live', ground: null, robotAt: 'r0' });
  await sky.click();
  assert.deepEqual(sky.calls.slice(2), ['select r0', 'render']);
});

test('a cancelled pointer never clicks', async () => {
  const { calls, interaction } = harness({ robotAt: 'r0' });
  interaction.pointerDown({ x: 0, y: 0 }, 0, false);
  await interaction.pointerUp(0, true, () => ({ x: 0, y: 0 }), false);
  assert.deepEqual(calls, ['dragging true', 'dragging false']);
});

test('live goal qualification needs a drawn robot that can navigate', () => {
  assert.deepEqual(liveGoalTargets(['a', 'b', 'c'], ['a', 'b'], (id) => id !== 'b'), ['a']);
});
