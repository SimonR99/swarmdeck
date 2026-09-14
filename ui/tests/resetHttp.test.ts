import assert from 'node:assert/strict';
import { test } from 'node:test';

import { fetchJsonWithTimeout, resetRequestId } from '../src/lib/api/resetHttp.ts';

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
