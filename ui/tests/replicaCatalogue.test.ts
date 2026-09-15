import assert from 'node:assert/strict';
import test from 'node:test';
import {
  catalogueLabel,
  catalogueSelection,
  fetchReplicaCatalogue,
  activeMergedRobotIds,
  automaticCatalogueEntry,
  automaticSelectionIsCoherent,
  parseReplicaCatalogue
} from '../src/lib/components/replicas/replicaCatalogue.ts';

const session = '12345678-1234-4234-8234-567812345678';

function body() {
  return {
    version: 1,
    active_session_id: session,
    components: [{
      session_id: session,
      component_id: 'component:merged',
      frame_id: 'component_merged',
      robot_ids: ['robot-a', 'robot-b'],
      source_count: 2,
      submap_count: 4,
      point_count: 120,
      available: true,
      status: 'ready',
      detail: '',
      solution_order: [2, 7],
      sources: [{
        robot_id: 'robot-a', session_id: session, revision: 4, snapshot_id: 'snap-a'
      }]
    }]
  };
}

test('catalogue validates entries and turns them into fleet selections', () => {
  const catalogue = parseReplicaCatalogue(body());
  assert.equal(catalogue.components.length, 1);
  assert.equal(catalogue.active_session_id, session);
  const selected = catalogueSelection(catalogue.components[0]);
  assert.deepEqual(selected, {
    scope: 'fleet', robotId: 'fleet', sessionId: session, componentId: 'component:merged'
  });
  assert.deepEqual(catalogueSelection(catalogue.components[0], 'robot', 'robot-a'), {
    scope: 'robot', robotId: 'robot-a', sessionId: session, componentId: 'component:merged'
  });
  assert.match(catalogueLabel(catalogue.components[0]), /12345678.*robot-a, robot-b/);
});

test('catalogue rejects malformed or unsafe readiness fields', () => {
  const malformed = body();
  malformed.components[0].status = 'ready-ish';
  assert.throws(() => parseReplicaCatalogue(malformed), /status is invalid/);
  const counts = body();
  counts.components[0].point_count = -1;
  assert.throws(() => parseReplicaCatalogue(counts), /point_count is invalid/);
  const readiness = body();
  readiness.components[0].available = 'true';
  assert.throws(() => parseReplicaCatalogue(readiness), /available is invalid/);
});

test('automatic selection stays within the server-declared mission and preferred robot component', () => {
  const catalogue = parseReplicaCatalogue({
    version: 1,
    active_session_id: session,
    components: [
      { ...body().components[0], component_id: 'component:other', robot_ids: ['robot-b'] },
      { ...body().components[0], component_id: 'component:merged', robot_ids: ['robot-a', 'robot-b'] },
      { ...body().components[0], session_id: '99999999-9999-4999-8999-999999999999', component_id: 'component:old' }
    ]
  });
  assert.equal(automaticCatalogueEntry(catalogue, 'robot-a')?.component_id, 'component:merged');
  assert.equal(automaticCatalogueEntry(catalogue, 'robot-a', true, 2)?.component_id, 'component:merged');
  assert.equal(automaticCatalogueEntry(catalogue, 'robot-z'), null);
  assert.equal(automaticCatalogueEntry(catalogue, 'robot-z', true), null);
  const single = parseReplicaCatalogue({
    version: 1,
    active_session_id: session,
    components: [{ ...body().components[0], component_id: 'component:single', robot_ids: ['robot-a'] }]
  });
  assert.equal(automaticCatalogueEntry(single, 'robot-z', true)?.component_id, 'component:single');
  assert.equal(automaticCatalogueEntry(single, 'robot-a', true, 2), null);
});

test('merged count uses the verified active component rather than legacy or historical robots', () => {
  const catalogue = parseReplicaCatalogue({
    version: 1,
    active_session_id: session,
    components: [
      {
        ...body().components[0],
        robot_ids: ['robot-b', 'robot-a', 'robot-a']
      },
      {
        ...body().components[0],
        session_id: '99999999-9999-4999-8999-999999999999',
        component_id: 'component:historical',
        robot_ids: ['old-a', 'old-b', 'old-c']
      },
      {
        ...body().components[0],
        component_id: 'component:unverified',
        robot_ids: ['waiting-a', 'waiting-b', 'waiting-c'],
        available: false,
        status: 'syncing'
      }
    ]
  });

  assert.deepEqual(activeMergedRobotIds(catalogue), ['robot-a', 'robot-b']);
  assert.equal(activeMergedRobotIds({ ...catalogue, active_session_id: null }), null);
});

test('merged count follows the preferred active Global component', () => {
  const catalogue = parseReplicaCatalogue({
    version: 1,
    active_session_id: session,
    components: [
      { ...body().components[0], component_id: 'component:small', robot_ids: ['robot-a', 'robot-b'] },
      { ...body().components[0], component_id: 'component:large', robot_ids: ['robot-c', 'robot-d', 'robot-e'] }
    ]
  });

  assert.deepEqual(activeMergedRobotIds(catalogue, 'robot-a'), ['robot-a', 'robot-b']);
  const explicit = {
    scope: 'fleet' as const, robotId: 'fleet', sessionId: session, componentId: 'component:large'
  };
  assert.deepEqual(activeMergedRobotIds(catalogue, 'robot-a', explicit), ['robot-c', 'robot-d', 'robot-e']);
  assert.deepEqual(activeMergedRobotIds(catalogue, 'robot-a', {
    ...explicit, sessionId: '99999999-9999-4999-8999-999999999999'
  }), ['robot-a', 'robot-b']);
  assert.deepEqual(activeMergedRobotIds(catalogue, 'missing'), []);
  assert.deepEqual(activeMergedRobotIds(parseReplicaCatalogue({
    version: 1,
    active_session_id: session,
    components: [
      { ...body().components[0], component_id: 'component:a', robot_ids: ['robot-a'] },
      { ...body().components[0], component_id: 'component:b', robot_ids: ['robot-b'] }
    ]
  }), 'robot-a'), []);
});

test('explicit active Global component does not fall through when missing or unready', () => {
  const ready = {
    ...body().components[0],
    component_id: 'component:ready',
    robot_ids: ['robot-a', 'robot-b']
  };
  const catalogue = parseReplicaCatalogue({
    version: 1,
    active_session_id: session,
    components: [
      ready,
      {
        ...ready,
        component_id: 'component:syncing',
        robot_ids: ['robot-c', 'robot-d'],
        status: 'syncing'
      },
      {
        ...ready,
        component_id: 'component:singleton',
        robot_ids: ['robot-e']
      }
    ]
  });

  assert.deepEqual(activeMergedRobotIds(catalogue, 'robot-a', {
    scope: 'fleet', robotId: 'fleet', sessionId: session, componentId: 'component:missing'
  }), []);
  assert.deepEqual(activeMergedRobotIds(catalogue, 'robot-a', {
    scope: 'fleet', robotId: 'fleet', sessionId: session, componentId: 'component:syncing'
  }), []);
  assert.deepEqual(activeMergedRobotIds(catalogue, 'robot-a', {
    scope: 'fleet', robotId: 'fleet', sessionId: session, componentId: 'component:singleton'
  }), []);
  assert.deepEqual(activeMergedRobotIds(catalogue, 'robot-a'), ['robot-a', 'robot-b']);
  assert.deepEqual(activeMergedRobotIds(catalogue, 'robot-a', {
    scope: 'fleet',
    robotId: 'fleet',
    sessionId: '99999999-9999-4999-8999-999999999999',
    componentId: 'component:missing'
  }), ['robot-a', 'robot-b']);
});

test('automatic retention cannot relabel a fleet view as local during a transient catalogue state', () => {
  const catalogue = parseReplicaCatalogue(body());
  const entry = catalogue.components[0];
  assert.equal(automaticSelectionIsCoherent(
    { scope: 'fleet', robotId: 'fleet', sessionId: session, componentId: entry.component_id },
    entry,
    session,
    true,
    'robot-a'
  ), false);
  assert.equal(automaticSelectionIsCoherent(
    { scope: 'robot', robotId: 'robot-a', sessionId: session, componentId: entry.component_id },
    { ...entry, status: 'syncing', available: false },
    session,
    true,
    'robot-a'
  ), true);
  assert.equal(automaticSelectionIsCoherent(
    { scope: 'fleet', robotId: 'fleet', sessionId: session, componentId: entry.component_id },
    { ...entry, status: 'conflict', available: false },
    session,
    false
  ), true);
});

test('catalogue client requests all sessions by default and preserves explicit session filter', async () => {
  const oldFetch = globalThis.fetch;
  const calls: string[] = [];
  globalThis.fetch = async (input) => {
    calls.push(String(input));
    return Response.json(body());
  };
  try {
    await fetchReplicaCatalogue(undefined);
    await fetchReplicaCatalogue(session);
    assert.deepEqual(calls, [
      '/api/autonomy/replicas/components',
      `/api/autonomy/replicas/components?session_id=${session}`
    ]);
  } finally {
    globalThis.fetch = oldFetch;
  }
});
