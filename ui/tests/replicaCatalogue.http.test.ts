import assert from 'node:assert/strict';
import test from 'node:test';
import { parseReplicaCatalogue } from '../src/lib/components/replicas/replicaCatalogue.ts';

const base = process.env.SWARMDECK_REPLICA_BASE_URL?.trim();
const sessionFilter = process.env.SWARMDECK_REPLICA_SESSION_ID?.trim();
const componentFilter = process.env.SWARMDECK_REPLICA_COMPONENT_ID?.trim();

function requestUrl(path: string, params: Record<string, string> = {}) {
  if (!base) throw new Error('SWARMDECK_REPLICA_BASE_URL is not set');
  const url = new URL(path, base);
  for (const [key, value] of Object.entries(params)) url.searchParams.set(key, value);
  return url;
}

async function fetchJson(url: URL) {
  const response = await fetch(url, { signal: AbortSignal.timeout(5_000) });
  assert.equal(response.ok, true, `${url} returned HTTP ${response.status}`);
  return response.json() as Promise<unknown>;
}

test('HTTP catalogue and aggregate view stay compatible with the UI contract', {
  skip: !base,
  timeout: 12_000
}, async () => {
  const catalogueUrl = requestUrl('/api/autonomy/replicas/components',
    sessionFilter ? { session_id: sessionFilter } : {});
  const catalogue = parseReplicaCatalogue(await fetchJson(catalogueUrl));
  const candidates = catalogue.components.filter((entry) => entry.available &&
    (!sessionFilter || entry.session_id === sessionFilter) &&
    (!componentFilter || entry.component_id === componentFilter));
  assert.ok(candidates.length > 0, 'replay must publish an available component');

  const entry = candidates[0];
  const view = await fetchJson(requestUrl(
    `/api/autonomy/replicas/components/view/${encodeURIComponent(entry.session_id)}`,
    { component_id: entry.component_id }
  )) as Record<string, unknown>;
  assert.equal(view.version, 1);
  assert.equal(view.scope, 'fleet');
  assert.equal(view.robot_id, 'fleet');
  assert.equal(view.session_id, entry.session_id);
  assert.equal(view.component_id, entry.component_id);
  assert.equal(typeof view.snapshot_id, 'string');
  assert.equal(view.revision, null);
  assert.equal(Array.isArray(view.sources), true);
  assert.equal((view.sources as unknown[]).length, entry.source_count);
  const selected = view.selected as Record<string, unknown>;
  assert.equal(typeof selected, 'object');
  assert.equal(selected.frame_id, entry.frame_id);
  assert.equal(selected.graph_revision, null);
  assert.equal(Array.isArray(selected.submaps), true);
});
