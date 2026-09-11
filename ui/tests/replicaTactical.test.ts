import assert from 'node:assert/strict';
import test from 'node:test';
import {
  ReplicaTacticalLoader,
  ReplicaRevisionTracker,
  acceptsReplicaResult,
  replicaSelectionKey,
  replicaTransition,
  type ReplicaTacticalSelection,
  type ReplicaView
} from '../src/lib/components/map3d/replicaTactical.ts';

const digest = 'a'.repeat(64);
const selection: ReplicaTacticalSelection = {
  robotId: 'robot_7',
  sessionId: 'session-a',
  componentId: 'component:a'
};

function xyz(points: number[][]) {
  const bytes = new Uint8Array(16 + points.length * 12);
  bytes.set(new TextEncoder().encode('SDXYZ1\0\0'));
  const view = new DataView(bytes.buffer);
  view.setBigUint64(8, BigInt(points.length), true);
  points.flat().forEach((value, index) => view.setFloat32(16 + index * 4, value, true));
  return bytes;
}

function replicaView(revision: number, tx: number, frame = 'component_a', epoch = 2): ReplicaView {
  const chunk = { sha256: digest, point_count: 2, size_bytes: 40 };
  const selected = {
    component_id: selection.componentId,
    frame_id: frame,
    graph_revision: { component_id: selection.componentId, epoch, revision },
    geometry_revision: revision,
    submaps: [{
      submap_id: 'robot_7/session-a/submap/1',
      geometry_revision: 1,
      pose_revision: revision,
      T_component_submap: [
        [0, -1, 0, tx],
        [1, 0, 0, 20],
        [0, 0, 1, 30],
        [0, 0, 0, 1]
      ],
      chunks: [chunk]
    }]
  };
  return {
    robot_id: selection.robotId,
    session_id: selection.sessionId,
    revision,
    snapshot_id: `snapshot-${revision}`,
    component_id: selection.componentId,
    components: [selected],
    selected,
    chunks: [chunk],
    source_age_s: null
  };
}

test('pose-only replica revisions transform in the component frame and reuse immutable chunks', async () => {
  const oldFetch = globalThis.fetch;
  let current = replicaView(1, 10);
  let chunkDownloads = 0;
  globalThis.fetch = async (input) => {
    if (String(input).includes('/chunks/')) {
      chunkDownloads++;
      return new Response(xyz([[1, 2, 3], [4, 5, 6]]));
    }
    return Response.json(current);
  };
  try {
    const loader = new ReplicaTacticalLoader(1024, 10);
    const first = await loader.load(selection, new AbortController().signal);
    assert.ok(first);
    assert.deepEqual([...first.positions], [8, 21, 33, 5, 24, 36]);
    assert.deepEqual(first.ownerIds, ['robot_7']);
    assert.equal(replicaTransition(null, first), 'selection');

    assert.equal(
      await loader.load(selection, new AbortController().signal, first),
      null
    );
    current = replicaView(2, 11);
    const second = await loader.load(selection, new AbortController().signal, first);
    assert.ok(second);
    assert.deepEqual([...second.positions], [9, 21, 33, 6, 24, 36]);
    assert.equal(replicaTransition(first, second), 'revision');
    assert.equal(chunkDownloads, 1);
  } finally {
    globalThis.fetch = oldFetch;
  }
});

test('robot graph epoch changes require a viewport reset for the same component', async () => {
  const oldFetch = globalThis.fetch;
  let current = replicaView(1, 0, 'component_a', 2);
  globalThis.fetch = async (input) => String(input).includes('/chunks/')
    ? new Response(xyz([[0, 0, 0], [1, 0, 0]]))
    : Response.json(current);
  try {
    const loader = new ReplicaTacticalLoader(1024, 10);
    const first = await loader.load(selection, new AbortController().signal);
    assert.ok(first);
    current = replicaView(2, 0, 'component_a', 3);
    const second = await loader.load(selection, new AbortController().signal, first);
    assert.ok(second);
    assert.equal(replicaTransition(first, second), 'frame');
  } finally {
    globalThis.fetch = oldFetch;
  }
});

test('component epoch or frame changes require a viewport reset', () => {
  const common = { sourceKey: replicaSelectionKey(selection), revisionKey: 'one' };
  assert.equal(replicaTransition(null, { ...common, frameKey: 'epoch-1' }), 'selection');
  assert.equal(replicaTransition(
    { ...common, frameKey: 'epoch-1' },
    { ...common, frameKey: 'epoch-1', revisionKey: 'two' }
  ), 'revision');
  assert.equal(replicaTransition(
    { ...common, frameKey: 'epoch-1' },
    { ...common, frameKey: 'epoch-2', revisionKey: 'two' }
  ), 'frame');
  assert.equal(replicaTransition(
    { ...common, frameKey: 'epoch-1' },
    { sourceKey: `${common.sourceKey}-other`, frameKey: 'epoch-1', revisionKey: 'one' }
  ), 'selection');
});

test('late data from a previous component cannot replace the current selection', () => {
  const oldSource = replicaSelectionKey(selection);
  const next = { ...selection, componentId: 'component:b' };
  assert.equal(acceptsReplicaResult(oldSource, next, 4, 5), false);
  assert.equal(acceptsReplicaResult(oldSource, selection, 4, 5), false);
  assert.equal(acceptsReplicaResult(oldSource, selection, 5, 5), true);
});

test('loader requires the server to return the explicitly selected component', async () => {
  const oldFetch = globalThis.fetch;
  const wrong = replicaView(1, 10);
  wrong.component_id = 'component:b';
  globalThis.fetch = async () => Response.json(wrong);
  try {
    await assert.rejects(
      new ReplicaTacticalLoader().load(selection, new AbortController().signal),
      /does not match the selected component/
    );
  } finally {
    globalThis.fetch = oldFetch;
  }
});

test('fleet catalogue selections use the aggregate view and scope-specific cache key', async () => {
  const oldFetch = globalThis.fetch;
  const fleetSelection: ReplicaTacticalSelection = {
    scope: 'fleet', robotId: 'fleet', sessionId: selection.sessionId, componentId: selection.componentId
  };
  const fleetView = replicaView(3, 0);
  fleetView.scope = 'fleet';
  fleetView.robot_id = 'fleet';
  fleetView.revision = null;
  fleetView.selected!.graph_revision = null;
  let requested = '';
  globalThis.fetch = async (input) => {
    const url = String(input);
    if (url.includes('/chunks/')) return new Response(xyz([[0, 0, 0], [1, 0, 0]]));
    requested = url;
    return Response.json(fleetView);
  };
  try {
    const loaded = await new ReplicaTacticalLoader(1024, 10)
      .load(fleetSelection, new AbortController().signal);
    assert.ok(loaded);
    assert.ok(requested.startsWith(
      `/api/autonomy/replicas/components/view/${selection.sessionId}?`
    ));
    assert.match(requested, /component_id=component%3Amerged|component_id=component%3Aa/);
    assert.match(loaded.sourceKey, /^fleet\u0000/);
    assert.match(loaded.frameKey, /fleet/);
  } finally {
    globalThis.fetch = oldFetch;
  }
});

test('source assembly is capped before the existing render-quality budget', async () => {
  const oldFetch = globalThis.fetch;
  globalThis.fetch = async (input) => String(input).includes('/chunks/')
    ? new Response(xyz([[0, 0, 0], [1, 0, 0]]))
    : Response.json(replicaView(1, 0));
  try {
    const cloud = await new ReplicaTacticalLoader(1024, 1)
      .load(selection, new AbortController().signal);
    assert.ok(cloud);
    assert.equal(cloud.positions.length, 3);
    assert.equal(cloud.owners.length, 1);
    assert.equal(cloud.partial, true);
  } finally {
    globalThis.fetch = oldFetch;
  }
});

test('forced rebuilds reuse a revision while transient failures retain the committed display', async () => {
  const oldFetch = globalThis.fetch;
  let failView = false;
  let chunkDownloads = 0;
  globalThis.fetch = async (input) => {
    if (String(input).includes('/chunks/')) {
      chunkDownloads++;
      return new Response(xyz([[0, 0, 0], [1, 0, 0]]));
    }
    if (failView) throw new Error('transient fetch failure');
    return Response.json(replicaView(1, 0));
  };
  try {
    const loader = new ReplicaTacticalLoader(1024, 10);
    const tracker = new ReplicaRevisionTracker();
    const first = await loader.load(selection, new AbortController().signal);
    assert.ok(first);
    tracker.commit(first);

    // Passing no known revision is the quality-change path: rebuild from the
    // immutable cache even though the server revision has not changed.
    const rebuilt = await loader.load(selection, new AbortController().signal, null);
    assert.ok(rebuilt);
    assert.equal(tracker.transition(rebuilt), 'unchanged');
    assert.equal(chunkDownloads, 1);

    failView = true;
    await assert.rejects(
      loader.load(selection, new AbortController().signal, tracker.current),
      /transient fetch failure/
    );
    assert.deepEqual(tracker.current, {
      sourceKey: first.sourceKey,
      frameKey: first.frameKey,
      revisionKey: first.revisionKey
    });
  } finally {
    globalThis.fetch = oldFetch;
  }
});
