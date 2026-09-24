import assert from 'node:assert/strict';
import { test } from 'node:test';

import { canResetSimulation, fetchJsonWithTimeout, resetRequestId, resetRobotMap } from '../src/lib/api/resetHttp.ts';

test('simulation reset is visible only with an explicitly available supervisor', () => {
  assert.equal(canResetSimulation({ supervisor_available: true }), true);
  assert.equal(canResetSimulation({ supervisor_available: false }), false);
  assert.equal(canResetSimulation({}), false);
  assert.equal(canResetSimulation(undefined), false);
  assert.equal(canResetSimulation({ supervisor_available: 'true' }), false);
});

test('reset request IDs use getRandomValues and set UUID v4/variant bits', () => {
  const id = resetRequestId((bytes) => {
    for (let index = 0; index < bytes.length; index++) bytes[index] = index;
  });

  assert.equal(id, '00010203-0405-4607-8809-0a0b0c0d0e0f');
});

test('reset fetch deadline remains active while the response body is read', async () => {
  const fetcher = (async (_input: RequestInfo | URL, init?: RequestInit) => ({
    ok: true,
    json: () => new Promise((_resolve, reject) => {
      init?.signal?.addEventListener('abort', () => reject(new Error('body aborted')));
    })
  }) as Response) as typeof fetch;

  await assert.rejects(
    fetchJsonWithTimeout('/api/sim/reset', { method: 'POST' }, 5, fetcher),
    /body aborted/
  );
});

test('robot reset stays pending until the supervisor verifies a fresh run', async () => {
  const gate = Promise.withResolvers<void>();
  const pending = Promise.withResolvers<void>();
  let finished = false;
  let calls = 0;
  const fetcher = (async (_input, init) => {
    calls++;
    if (calls === 1) {
      assert.equal(init?.method, 'POST');
      return Response.json({ request_id: 'request', robot_id: 'r0', phase: 'accepted', ok: null }, { status: 202 });
    }
    return Response.json({ request_id: 'request', robot_id: 'r0', phase: 'done', ok: true, map_epoch: 1, run_id: 'fresh-run' });
  }) as typeof fetch;
  const result = resetRobotMap('r0', 'request', fetcher, () => {
    pending.resolve();
    return gate.promise;
  }).then(value => { finished = true; return value; });
  await pending.promise;
  assert.equal(finished, false);
  gate.resolve();
  assert.equal((await result).map_epoch, 1);
});

test('robot reset reports a supervisor failure instead of treating HTTP 200 as success', async () => {
  let calls = 0;
  const fetcher = (async () => Response.json({
    request_id: 'request', robot_id: 'r0',
    ...(calls++ === 0
      ? { phase: 'accepted', ok: null }
      : { phase: 'failed', ok: false, error: 'planner did not restart' })
  })) as typeof fetch;
  await assert.rejects(
    resetRobotMap('r0', 'request', fetcher, async () => {}),
    /planner did not restart/
  );
});

test('a lost robot reset POST retries its UUID and never submits a second reset', async () => {
  const requests: string[] = [];
  const fetcher = (async (input, init) => {
    assert.equal(init?.method, 'POST');
    requests.push(String(input));
    if (requests.length === 1) throw new TypeError('connection closed');
    return Response.json({
      request_id: 'request', robot_id: 'r0', phase: 'done', ok: true, map_epoch: 1, run_id: 'fresh-run'
    });
  }) as typeof fetch;
  const result = await resetRobotMap('r0', 'request', fetcher, async () => {});
  assert.equal(result.phase, 'done');
  assert.deepEqual(requests, [
    '/api/map/reset/r0?request_id=request',
    '/api/map/reset/r0?request_id=request'
  ]);
});
