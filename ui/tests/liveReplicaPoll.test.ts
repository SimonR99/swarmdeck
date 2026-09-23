import assert from 'node:assert/strict';
import test from 'node:test';
import { LIVE_REPLICA_TIMEOUT_MS, LiveReplicaPoll } from '../src/lib/components/map3d/liveReplicaPoll.ts';
import { parseLiveReplicaFrame, type LiveReplicaFrame } from '../src/lib/components/map3d/liveReplicaFrame.ts';
import { replicaSelectionKey, type ReplicaTacticalSelection } from '../src/lib/components/map3d/replicaTactical.ts';

const session = '12345678-1234-4234-8234-567812345678';
const selection: ReplicaTacticalSelection = {
  scope: 'fleet', robotId: 'fleet', sessionId: session, componentId: 'component:merged'
};

function frame(x = 1, poseAge = 0): LiveReplicaFrame {
  return parseLiveReplicaFrame({
    version: 1, mission_id: session, session_id: session, component_id: 'component:merged',
    frame_id: 'component_frame', solution_order: [2, 7],
    robots: [{
      robot_id: 'robot-a', mission_id: session, component_id: 'component:merged', navigation_frame: 'nav_a',
      T_component_navigation: [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
      pose: { x, y: 2, z: 0, yaw: 0 }, goal: null,
      planned_path: [], global_planned_path: [], local_planned_path: [],
      freshness: { pose_s: poseAge, goal_s: null, path_s: null },
      nav_status: 'idle', mode: 'idle'
    }]
  });
}

function harness() {
  let now = 1000;
  const events: string[] = [];
  const wakeups: { run: () => void; at: number }[] = [];
  const requests: { resolve: (value: LiveReplicaFrame | null) => void; reject: (reason: unknown) => void; signal: AbortSignal }[] = [];
  const timeouts: number[] = [];
  let wanted = true;
  const poll = new LiveReplicaPoll(
    {
      onDrawChange: () => events.push('draw'),
      onExpire: () => events.push('expire'),
      stillWanted: () => wanted
    },
    {
      clock: () => now,
      fetchFrame: (_selection, signal) => new Promise((resolve, reject) => requests.push({ resolve, reject, signal })),
      timers: {
        set: (run, delay) => wakeups.push({ run, at: now + delay }) - 1,
        clear: (handle) => { wakeups[handle as number] = { run: () => {}, at: Infinity }; }
      },
      setTimeout: (_run, ms) => timeouts.push(ms),
      clearTimeout: () => {}
    }
  );
  return {
    poll, events, requests, wakeups, timeouts,
    advance: (ms: number) => { now += ms; },
    unwant: () => { wanted = false; }
  };
}

const settle = () => new Promise((resolve) => setImmediate(resolve));

test('a new frame is adopted, drawn once, and not requested twice at a time', async () => {
  const { poll, events, requests, timeouts } = harness();
  poll.select(replicaSelectionKey(selection));
  void poll.refresh(selection);
  void poll.refresh(selection);
  assert.equal(requests.length, 1);
  assert.deepEqual(timeouts, [LIVE_REPLICA_TIMEOUT_MS]);
  requests[0].resolve(frame());
  await settle();
  assert.equal(poll.current?.receivedAt, 1000);
  assert.deepEqual(events, ['draw']);
  void poll.refresh(selection);
  requests[1].resolve(frame());
  await settle();
  assert.deepEqual(events, ['draw'], 'the same pose again is not a redraw');
});

test('a missing frame or a failure keeps the last one', async () => {
  const { poll, requests } = harness();
  poll.select(replicaSelectionKey(selection));
  void poll.refresh(selection);
  requests[0].resolve(frame());
  await settle();
  const kept = poll.current;
  void poll.refresh(selection);
  requests[1].resolve(null);
  await settle();
  void poll.refresh(selection);
  requests[2].reject(new Error('503'));
  await settle();
  assert.equal(poll.current, kept);
});

test('a changed selection abandons the request and forgets the frame', async () => {
  const { poll, events, requests } = harness();
  poll.select(replicaSelectionKey(selection));
  void poll.refresh(selection);
  requests[0].resolve(frame());
  await settle();
  void poll.refresh(selection);
  poll.select('');
  assert.equal(requests[1].signal.aborted, true);
  assert.equal(poll.current, null);
  requests[1].resolve(frame(5));
  await settle();
  assert.equal(poll.current, null, 'a response for the old selection is dropped');
  assert.deepEqual(events, ['draw', 'draw']);
});

test('a response no longer wanted is dropped', async () => {
  const { poll, requests, unwant } = harness();
  poll.select(replicaSelectionKey(selection));
  void poll.refresh(selection);
  unwant();
  requests[0].resolve(frame());
  await settle();
  assert.equal(poll.current, null);
});

test('the view is woken when the drawn pose expires', async () => {
  const { poll, events, requests, wakeups } = harness();
  poll.select(replicaSelectionKey(selection));
  void poll.refresh(selection);
  requests[0].resolve(frame(1, 2));
  await settle();
  const pending = wakeups.filter((wakeup) => wakeup.at !== Infinity);
  assert.equal(pending.length, 1);
  assert.equal(pending[0].at, 1000 + 1000 + 1);
  pending[0].run();
  assert.deepEqual(events, ['draw', 'expire']);
  poll.dispose();
});
