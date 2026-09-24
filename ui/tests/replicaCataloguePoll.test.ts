import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  REPLICA_CATALOGUE_POLL_MS,
  ReplicaCataloguePoller,
  type PollTimers,
  type ReplicaCatalogueSnapshot
} from '../src/lib/components/replicas/replicaCataloguePoll.ts';
import type { ReplicaCatalogue } from '../src/lib/components/replicas/replicaCatalogue.ts';
import { peerMergeComponent } from '../src/lib/components/slam/peerStatus.ts';

class FakeTimers implements PollTimers {
  running = new Map<number, { callback: () => void; ms: number }>();
  private next = 1;
  setInterval = (callback: () => void, ms: number) => {
    const id = this.next++;
    this.running.set(id, { callback, ms });
    return id;
  };
  clearInterval = (handle: unknown) => {
    this.running.delete(handle as number);
  };
  cadences() {
    return [...this.running.values()].map((timer) => timer.ms);
  }
  fire() {
    for (const timer of [...this.running.values()]) timer.callback();
  }
}

function catalogue(session: string): ReplicaCatalogue {
  return { version: 1, active_session_id: session, components: [] };
}

function harness() {
  const timers = new FakeTimers();
  const published: ReplicaCatalogueSnapshot[] = [];
  const requests: { resolve: (value: ReplicaCatalogue) => void; reject: (reason: unknown) => void; signal: AbortSignal }[] = [];
  const poller = new ReplicaCataloguePoller(
    (signal) => new Promise((resolve, reject) => requests.push({ resolve, reject, signal })),
    (snapshot) => published.push(snapshot),
    timers
  );
  return { timers, published, requests, poller };
}

const settle = () => new Promise((resolve) => setImmediate(resolve));

test('views share one request and one timer', async () => {
  const { timers, requests, poller } = harness();
  const auto = poller.subscribe();
  const selector = poller.subscribe();
  assert.equal(requests.length, 1, 'a second view reuses the refresh in flight');
  assert.deepEqual(timers.cadences(), [REPLICA_CATALOGUE_POLL_MS]);
  requests[0].resolve(catalogue('s1'));
  await settle();
  assert.equal(poller.current.catalogue?.active_session_id, 's1');
  timers.fire();
  assert.equal(requests.length, 2);
  auto();
  selector();
});

test('the shortest cadence any view asked for wins, and the poll stops with the last view', async () => {
  const { timers, requests, poller } = harness();
  const auto = poller.subscribe();
  requests[0].resolve(catalogue('s1'));
  await settle();
  const panel = poller.subscribe(2000);
  assert.deepEqual(timers.cadences(), [2000]);
  assert.equal(requests.length, 2, 'a view that appears refreshes at once');
  panel();
  assert.deepEqual(timers.cadences(), [REPLICA_CATALOGUE_POLL_MS]);
  auto();
  assert.deepEqual(timers.cadences(), []);
  assert.equal(requests[1].signal.aborted, true, 'the last view leaving abandons the request');
});

test('a failure keeps the last catalogue and reports why; a success clears it', async () => {
  const { requests, poller, published } = harness();
  const stop = poller.subscribe();
  requests[0].resolve(catalogue('s1'));
  await settle();
  void poller.refresh();
  requests[1].reject(new Error('Replica catalogue unavailable (503)'));
  await settle();
  assert.equal(poller.current.catalogue?.active_session_id, 's1');
  assert.equal(poller.current.error, 'Replica catalogue unavailable (503)');
  assert.equal(poller.current.loading, false);
  void poller.refresh();
  assert.equal(published.at(-1)?.loading, true);
  requests[2].resolve(catalogue('s2'));
  await settle();
  assert.deepEqual(poller.current, { catalogue: catalogue('s2'), error: '', refreshFailed: false, loading: false });
  stop();
});

test('a timeout keeps the catalogue and the error text but reports a failed refresh', async () => {
  const { requests, poller } = harness();
  const stop = poller.subscribe();
  requests[0].resolve(catalogue('s1'));
  await settle();
  void poller.refresh();
  requests[1].reject(new DOMException('Aborted', 'AbortError'));
  await settle();
  assert.deepEqual(poller.current, { catalogue: catalogue('s1'), error: '', refreshFailed: true, loading: false });
  stop();
});

test('a request abandoned because the last view left reports nothing', async () => {
  const { requests, poller } = harness();
  const stop = poller.subscribe();
  stop();
  requests[0].reject(new DOMException('Aborted', 'AbortError'));
  await settle();
  assert.equal(poller.current.refreshFailed, false);
  assert.equal(poller.current.error, '');
  assert.equal(poller.current.loading, false);
});

function merged(session: string): ReplicaCatalogue {
  const entry = {
    session_id: session, component_id: 'component:ab', frame_id: 'f', robot_ids: ['a', 'b'],
    source_count: 2, submap_count: 2, point_count: 10, available: true, status: 'ready' as const,
    detail: '', solution_order: null, sources: [], composite: false
  };
  return { version: 1, active_session_id: session, components: [entry] };
}

test('the Peer SLAM merge clears on a timeout or failure and returns on recovery', async () => {
  const { requests, poller } = harness();
  const stop = poller.subscribe(2000);
  requests[0].resolve(merged('s1'));
  await settle();
  assert.deepEqual(peerMergeComponent(poller.current)?.robot_ids, ['a', 'b']);
  void poller.refresh();
  requests[1].reject(new DOMException('Aborted', 'AbortError'));
  await settle();
  assert.equal(peerMergeComponent(poller.current), null, 'a timeout clears the merge');
  assert.equal(poller.current.catalogue?.components.length, 1, 'the selectors keep their cache');
  void poller.refresh();
  requests[2].resolve(merged('s1'));
  await settle();
  assert.deepEqual(peerMergeComponent(poller.current)?.robot_ids, ['a', 'b']);
  void poller.refresh();
  requests[3].reject(new Error('Replica catalogue unavailable (503)'));
  await settle();
  assert.equal(peerMergeComponent(poller.current), null, 'a failure clears the merge');
  stop();
});
