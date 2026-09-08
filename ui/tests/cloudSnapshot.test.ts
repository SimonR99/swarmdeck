import test from 'node:test';
import assert from 'node:assert/strict';
import { deflate } from 'pako';
import { CloudSnapshotCache } from '../src/lib/components/map3d/cloudSnapshot.ts';

const idA = 'a'.repeat(64), idB = 'b'.repeat(64);
function tile(x: number, rgb: number[]) {
  const raw = new Uint8Array(16);
  new DataView(raw.buffer).setFloat32(0, x, true);
  raw.set(rgb, 13);
  return deflate(raw);
}
function manifest(chunks: {id: string; points: number}[]) {
  return new Response(JSON.stringify({ version: 1, chunks }), { headers: {
    'Content-Type': 'application/json', 'X-Cloud-Points': String(chunks.length),
    'X-Cloud-Format': 'xyz32', 'X-Cloud-RGB': '1', 'ETag': '"revision"'
  }});
}

test('only new regions download and planar XYZ/owners/RGB assemble correctly', async () => {
  const oldFetch = globalThis.fetch;
  let second = false;
  const downloads: string[] = [];
  globalThis.fetch = async (url) => {
    const path = String(url);
    if (path.includes('manifest=1')) return manifest(second ? [{id: idB, points: 1}, {id: idA, points: 1}] : [{id: idA, points: 1}]);
    downloads.push(path);
    return new Response(tile(path.includes(idA) ? 1 : 9, path.includes(idA) ? [255, 0, 0] : [0, 255, 0]));
  };
  try {
    const cache = new CloudSnapshotCache();
    const signal = new AbortController().signal;
    await cache.fetch('/api/map/cloud?robot_id=r', '', signal);
    second = true;
    const result = await cache.fetch('/api/map/cloud?robot_id=r', '', signal);
    assert.equal(downloads.length, 2);
    const raw = result.raw!;
    const xyz = new DataView(raw.buffer, raw.byteOffset);
    assert.equal(xyz.getFloat32(0, true), 9);
    assert.equal(xyz.getFloat32(12, true), 1);
    assert.deepEqual([...raw.subarray(24)], [0, 0, 0, 255, 0, 255, 0, 0]);
    await cache.fetch('/api/map/cloud?robot_id=r', '', signal);
    assert.equal(downloads.length, 2);
  } finally { globalThis.fetch = oldFetch; }
});

test('304 does not fetch chunks and sends the conditional revision', async () => {
  const oldFetch = globalThis.fetch;
  globalThis.fetch = async (_, init) => {
    assert.equal(new Headers(init?.headers).get('If-None-Match'), '"old"');
    return new Response(null, {status: 304});
  };
  try {
    assert.equal((await new CloudSnapshotCache().fetch('/api/map/cloud?source=slam', '"old"', new AbortController().signal)).raw, null);
  } finally { globalThis.fetch = oldFetch; }
});

test('incomplete snapshots are rejected and retries can recover', async () => {
  const oldFetch = globalThis.fetch;
  let failed = true;
  globalThis.fetch = async (url) => String(url).includes('manifest=1')
    ? manifest([{id: idA, points: 1}])
    : failed ? new Response(null, {status: 404}) : new Response(tile(1, [255, 0, 0]));
  try {
    const cache = new CloudSnapshotCache(), signal = new AbortController().signal;
    await assert.rejects(cache.fetch('/api/map/cloud?source=slam', '', signal), /404/);
    failed = false;
    assert.equal((await cache.fetch('/api/map/cloud?source=slam', '', signal)).raw?.length, 16);
  } finally { globalThis.fetch = oldFetch; }
});
