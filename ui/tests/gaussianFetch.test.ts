import assert from 'node:assert/strict';
import test from 'node:test';
import { GaussianFetch, MAX_GAUSSIAN_BYTES } from '../src/lib/components/map3d/gaussianFetch.ts';

function harness(initialScope = '?session_id=s&component_id=c') {
  const view = { scope: initialScope, scene: true };
  const calls: { url: string; headers: Record<string, string>; signal: AbortSignal }[] = [];
  const replies: ((signal: AbortSignal) => Promise<Response>)[] = [];
  const loader = new GaussianFetch(
    { scope: () => view.scope, hasScene: () => view.scene },
    {
      fetch: ((url: string, init: RequestInit) => {
        const signal = init.signal as AbortSignal;
        calls.push({ url, headers: init.headers as Record<string, string>, signal });
        return replies.shift()!(signal);
      }) as typeof fetch,
      setTimeout: () => 0,
      clearTimeout: () => {}
    }
  );
  const reply = (response: Response) => replies.push(async () => response);
  return { view, calls, replies, loader, reply };
}

test('a reconstruction is downloaded, then asked for by its ETag', async () => {
  const { loader, calls, reply } = harness();
  reply(new Response(new Uint8Array([1, 2, 3]), { headers: { ETag: '"v1"' } }));
  const loaded = await loader.fetch();
  assert.equal(loaded?.kind, 'loaded');
  assert.equal(calls[0].url, '/api/map/gaussians?session_id=s&component_id=c');
  assert.deepEqual(calls[0].headers, {});
  if (loaded?.kind === 'loaded') {
    assert.equal(loaded.buffer.byteLength, 3);
    loader.accept(loaded.etag);
  }
  reply(new Response(null, { status: 304 }));
  assert.deepEqual(await loader.fetch(), { kind: 'unchanged' });
  assert.deepEqual(calls[1].headers, { 'If-None-Match': '"v1"' });
});

test('an ETag not accepted is not sent, and forget drops it', async () => {
  const { loader, calls, reply } = harness();
  reply(new Response(new Uint8Array([1]), { headers: { ETag: '"v1"' } }));
  await loader.fetch();
  reply(new Response(new Uint8Array([1])));
  await loader.fetch();
  assert.deepEqual(calls[1].headers, {});
  loader.accept('"v2"');
  loader.forget();
  reply(new Response(new Uint8Array([1])));
  await loader.fetch();
  assert.deepEqual(calls[2].headers, {});
});

test('a missing reconstruction forgets the ETag', async () => {
  const { loader, calls, reply } = harness();
  loader.accept('"v1"');
  reply(new Response(null, { status: 404 }));
  assert.deepEqual(await loader.fetch(), { kind: 'missing' });
  reply(new Response(null, { status: 304 }));
  await loader.fetch();
  assert.deepEqual(calls[1].headers, {});
});

test('server errors and oversized downloads are reported', async () => {
  const { loader, reply } = harness();
  reply(new Response(null, { status: 503 }));
  assert.deepEqual(await loader.fetch(), { kind: 'failed', message: 'Reconstruction unavailable (503)' });
  reply(new Response(new Uint8Array([1]), { headers: { 'Content-Length': String(MAX_GAUSSIAN_BYTES + 1) } }));
  assert.deepEqual(await loader.fetch(), { kind: 'failed', message: 'Reconstruction exceeds download limit' });
});

test('one request at a time, and a response for another replica is dropped', async () => {
  const { loader, view, replies, reply } = harness();
  let release!: () => void;
  replies.push(() => new Promise((resolve) => { release = () => resolve(new Response(new Uint8Array([1]))); }));
  const first = loader.fetch();
  assert.equal(loader.busy, true);
  assert.equal(await loader.fetch(), null);
  view.scope = '?session_id=s&component_id=other';
  release();
  assert.equal(await first, null);
  assert.equal(loader.busy, false);
  view.scene = false;
  reply(new Response(null, { status: 503 }));
  assert.equal(await loader.fetch(), null, 'a response is dropped without a scene to show it in');
  replies.push(async () => { throw new TypeError('NetworkError'); });
  assert.deepEqual(await loader.fetch(), { kind: 'failed', message: 'NetworkError' },
    'a failed request for the replica on show is reported with or without a scene');
});

test('an aborted request reports nothing', async () => {
  const { loader, replies } = harness();
  replies.push((signal) => new Promise((_resolve, reject) => {
    signal.addEventListener('abort', () => reject(new DOMException('Aborted', 'AbortError')));
  }));
  const pending = loader.fetch();
  loader.abort();
  assert.equal(await pending, null);
});
