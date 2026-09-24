import assert from 'node:assert/strict';
import { test } from 'node:test';
import { readFileSync } from 'node:fs';
import ts from 'typescript';

import { canResetSimulation, fetchJsonWithTimeout, resetRequestId, resetRobotMap } from '../src/lib/api/resetHttp.ts';

test('simulation reset is visible only with an explicitly available supervisor', () => {
  assert.equal(canResetSimulation({ supervisor_available: true }), true);
  assert.equal(canResetSimulation({ supervisor_available: false }), false);
  assert.equal(canResetSimulation({}), false);
  assert.equal(canResetSimulation(undefined), false);
  assert.equal(canResetSimulation({ supervisor_available: 'true' }), false);
});

const connectionSource = readFileSync(new URL('../src/lib/api/connection.ts', import.meta.url), 'utf8');

function connectionFunction(start: string, end: string, bindings: Record<string, unknown>) {
  const source = connectionSource.slice(connectionSource.indexOf(start), connectionSource.indexOf(end));
  const { outputText } = ts.transpileModule(source, {
    compilerOptions: { target: ts.ScriptTarget.ESNext, module: ts.ModuleKind.None }
  });
  return new Function(...Object.keys(bindings), `${outputText}\nreturn ${start.match(/function (\w+)/)![1]};`)(...Object.values(bindings));
}

test('TopBar keeps reset progress and failure visible without trusting robot capabilities', () => {
  const source = readFileSync(new URL('../src/lib/components/TopBar.svelte', import.meta.url), 'utf8');
  const expression = source.match(/const canReset = \$derived\(([\s\S]*?)\);/)![1];
  const visible = new Function('session', 'fleet', `return (${expression});`);
  const fleet = { robots: [{ capabilities: ['reset'] }] };
  assert.equal(visible({ resetSupervisorAvailable: false, resetting: false, lastReset: null }, fleet), false);
  assert.equal(visible({ resetSupervisorAvailable: true, resetting: false, lastReset: null }, { robots: [] }), true);
  assert.equal(visible({ resetSupervisorAvailable: false, resetting: true, lastReset: null }, fleet), true);
  assert.equal(visible({ resetSupervisorAvailable: false, resetting: false, lastReset: { ok: false } }, fleet), true);
  assert.equal(visible({ resetSupervisorAvailable: false, resetting: false, lastReset: { ok: true } }, fleet), false);
});

test('connection reset status requires HTTP success and clears availability on transport failure', async () => {
  const availability: boolean[] = [];
  let result: [Response, { supervisor_available: boolean }] | Error;
  const fetchStatus = connectionFunction('async function fetchResetStatus', '\nfunction publishResetStatus', {
    fetchJsonWithTimeout: async () => {
      if (result instanceof Error) throw result;
      return result;
    },
    canResetSimulation,
    RESET_FETCH_TIMEOUT_MS: 5000,
    session: { setResetSupervisorAvailable: (value: boolean) => availability.push(value) }
  });
  result = [new Response(null, { status: 200 }), { supervisor_available: true }];
  await fetchStatus();
  result = [new Response(null, { status: 503 }), { supervisor_available: true }];
  await fetchStatus();
  result = new Error('offline');
  await assert.rejects(fetchStatus(), /offline/);
  assert.deepEqual(availability, [true, false, false]);
});

test('connection refreshes reset availability every 30 seconds only while live and clears its timer', () => {
  const intervals: { callback: () => void; delay: number }[] = [];
  const session = { connection: 'connecting', tick: () => {} };
  let refreshes = 0;
  const cleared: number[] = [];
  const source = connectionSource.slice(connectionSource.indexOf('export function startConnection'))
    .replaceAll('export function', 'function');
  const { outputText } = ts.transpileModule(source, {
    compilerOptions: { target: ts.ScriptTarget.ESNext }
  });
  const run = new Function('session', 'setInterval', 'clearInterval', 'resumeReset', `
    let started = false, tickTimer = null, resetAvailabilityTimer = null;
    let retryTimer = null, ws = null, mock = null, retry = 0;
    const detectionCatalog = { load() {} };
    const connect = () => {};
    ${outputText}
    return { startConnection, teardown };
  `)(session, (callback: () => void, delay: number) => {
    intervals.push({ callback, delay });
    return intervals.length;
  }, (id: number) => cleared.push(id), () => { refreshes++; });
  run.startConnection();
  run.startConnection();
  const refresh = intervals.find(({ delay }) => delay === 30_000);
  assert.ok(refresh, 'a slow supervisor refresh timer is installed');
  assert.equal(intervals.length, 2);
  const initial = refreshes;
  refresh.callback();
  assert.equal(refreshes, initial);
  session.connection = 'live';
  refresh.callback();
  assert.equal(refreshes, initial + 1);
  session.connection = 'lost';
  refresh.callback();
  assert.equal(refreshes, initial + 1);
  run.teardown();
  assert.deepEqual(cleared.sort(), [1, 2]);
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
