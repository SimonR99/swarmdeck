import assert from 'node:assert/strict';
import test from 'node:test';
import {
  catalogueLabel,
  catalogueSelection,
  fetchReplicaCatalogue,
  parseReplicaCatalogue
} from '../src/lib/components/replicas/replicaCatalogue.ts';

const session = '12345678-1234-4234-8234-567812345678';

function body() {
  return {
    version: 1,
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
  const selected = catalogueSelection(catalogue.components[0]);
  assert.deepEqual(selected, {
    scope: 'fleet', robotId: 'fleet', sessionId: session, componentId: 'component:merged'
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
