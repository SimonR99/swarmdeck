import assert from 'node:assert/strict';
import test from 'node:test';
import {
  REPLICA_REVISION_POLL_MS,
  ReplicaCloudLoad,
  type PrepareCloud
} from '../src/lib/components/map3d/replicaCloudLoad.ts';
import {
  replicaSelectionKey,
  type ReplicaDisplayRevision,
  type ReplicaTacticalCloud,
  type ReplicaTacticalSelection
} from '../src/lib/components/map3d/replicaTactical.ts';
import type { TerrainData } from '../src/lib/components/map3d/terrainData.ts';

const selection: ReplicaTacticalSelection = { scope: 'fleet', robotId: 'fleet', sessionId: 's', componentId: 'c' };
const other: ReplicaTacticalSelection = { ...selection, componentId: 'd' };

function cloud(of: ReplicaTacticalSelection, frameKey = 'f1', revisionKey = 'r1'): ReplicaTacticalCloud {
  return {
    sourceKey: replicaSelectionKey(of), frameKey, revisionKey,
    positions: new Float32Array(3), owners: new Uint8Array(1), ownerIds: ['r0'], partial: false,
    view: {} as ReplicaTacticalCloud['view']
  };
}

type Deferred<T> = { resolve: (value: T) => void; reject: (reason: unknown) => void };

function harness() {
  const state = { shown: selection as ReplicaTacticalSelection | null, scene: true, now: 0 };
  const loads: (Deferred<ReplicaTacticalCloud | null> & { known: ReplicaDisplayRevision | null; signal: AbortSignal })[] = [];
  const prepares: (Deferred<TerrainData> & { id: number })[] = [];
  const prepare: PrepareCloud = (id) => new Promise((resolve, reject) => prepares.push({ id, resolve, reject }));
  const load = new ReplicaCloudLoad(
    { selection: () => state.shown, hasScene: () => state.scene },
    prepare,
    {
      loader: {
        load: (_selection, signal, known) =>
          new Promise((resolve, reject) => loads.push({ resolve, reject, known: known ?? null, signal }))
      },
      clock: () => state.now,
      setTimeout: () => 0,
      clearTimeout: () => {}
    }
  );
  return { state, loads, prepares, load };
}

const terrain = {} as TerrainData;
const settle = () => new Promise((resolve) => setImmediate(resolve));

test('a first load builds, resets the view, and later loads name the shown revision', async () => {
  const { loads, prepares, load } = harness();
  const first = load.load();
  assert.equal(await load.load(), null, 'one load at a time');
  loads[0].resolve(cloud(selection));
  await settle();
  prepares[0].resolve(terrain);
  const built = await first;
  assert.equal(built?.kind, 'built');
  if (built?.kind !== 'built') return;
  assert.equal(built.resetView, true);
  load.commit(built.cloud);
  const again = load.load();
  assert.equal(loads[1].known?.revisionKey, 'r1');
  loads[1].resolve(null);
  assert.deepEqual(await again, { kind: 'current' });
});

test('a new revision in the same frame keeps the view; a new frame resets it', async () => {
  const { loads, prepares, load } = harness();
  const results: boolean[] = [];
  for (const next of [cloud(selection), cloud(selection, 'f1', 'r2'), cloud(selection, 'f2', 'r3')]) {
    const pending = load.load();
    loads.at(-1)!.resolve(next);
    await settle();
    prepares.at(-1)!.resolve(terrain);
    const built = await pending;
    if (built?.kind !== 'built') assert.fail('not built');
    results.push(built.resetView);
    load.commit(built.cloud);
  }
  assert.deepEqual(results, [true, false, true]);
});

test('a rebuild or a forgotten revision loads whole', async () => {
  const { loads, prepares, load } = harness();
  const first = load.load();
  loads[0].resolve(cloud(selection));
  await settle();
  prepares[0].resolve(terrain);
  const built = await first;
  if (built?.kind === 'built') load.commit(built.cloud);
  load.requireRebuild();
  void load.load();
  assert.equal(loads[1].known, null);
  loads[1].resolve(null);
  await settle();
  load.forgetRevision(false);
  void load.load();
  assert.equal(loads[2].known, null);
});

test('a cancelled load, or one for a replica no longer shown, is dropped', async () => {
  const { state, loads, prepares, load } = harness();
  const cancelled = load.load();
  load.cancel();
  assert.equal(loads[0].signal.aborted, true);
  loads[0].reject(new DOMException('Aborted', 'AbortError'));
  assert.equal(await cancelled, null);
  const switched = load.load();
  loads[1].resolve(cloud(selection));
  await settle();
  state.shown = other;
  prepares[0].resolve(terrain);
  assert.equal(await switched, null);
});

test('a failure is reported unless it was cancelled', async () => {
  const { loads, load } = harness();
  const failing = load.load();
  loads[0].reject(new Error('Replica view unavailable (503)'));
  assert.deepEqual(await failing, { kind: 'failed', message: 'Replica view unavailable (503)' });
});

test('the poll is due for a rebuild at once and for a revision after the cadence', async () => {
  const { state, loads, prepares, load } = harness();
  const first = load.load();
  loads[0].resolve(cloud(selection));
  await settle();
  prepares[0].resolve(terrain);
  const built = await first;
  state.now = 1000;
  if (built?.kind === 'built') load.commit(built.cloud);
  assert.equal(load.due(), false);
  state.now = 1000 + REPLICA_REVISION_POLL_MS;
  assert.equal(load.due(), true);
  state.now = 1001;
  load.requireRebuild();
  assert.equal(load.due(), true);
});
