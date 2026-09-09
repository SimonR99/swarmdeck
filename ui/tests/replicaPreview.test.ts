import assert from 'node:assert/strict';
import { test } from 'node:test';
import {
  ReplicaChunkCache,
  ReplicaRequestGate,
  parseXYZF32,
  samplePreviewChunks,
  selectPreviewRefs
} from '../src/lib/components/replicas/replicaPreview.ts';

function xyz(count: number) {
  const bytes = new Uint8Array(16 + count * 12);
  bytes.set(new TextEncoder().encode('SDXYZ1\0\0'));
  new DataView(bytes.buffer).setBigUint64(8, BigInt(count), true);
  for (let index = 0; index < count * 3; index += 1) {
    new DataView(bytes.buffer).setFloat32(16 + index * 4, index, true);
  }
  return bytes;
}

test('XYZ-F32 parser validates framing and preserves little-endian points', () => {
  assert.deepEqual([...parseXYZF32(xyz(2))], [0, 1, 2, 3, 4, 5]);
  assert.throws(() => parseXYZF32(xyz(2).subarray(0, 20)), /Invalid replica chunk length/);
});

test('preview sampling applies one global cap across chunks', () => {
  const sampled = samplePreviewChunks([
    { submapId: 'a', sha256: 'one', points: new Float32Array(40_000 * 3) },
    { submapId: 'b', sha256: 'two', points: new Float32Array(40_000 * 3) }
  ], 50_000);
  const total = sampled.reduce((sum, entry) => sum + entry.points.length / 3, 0);
  assert.ok(total <= 50_000);
  assert.equal(sampled.length, 2);
});

test('chunk cache evicts oldest data at its byte budget', () => {
  const cache = new ReplicaChunkCache(18);
  const first = new Float32Array(3);
  const second = new Float32Array(3);
  assert.equal(cache.set('first', first), true);
  assert.equal(cache.set('second', second), true);
  assert.equal(cache.get('first'), undefined);
  assert.equal(cache.get('second'), second);
  assert.equal(cache.sizeBytes, 12);
});

test('request generations invalidate an older refresh', () => {
  const gate = new ReplicaRequestGate();
  const first = gate.begin();
  const second = gate.begin();
  assert.equal(gate.isCurrent(first.generation), false);
  assert.equal(first.signal.aborted, true);
  assert.equal(gate.isCurrent(second.generation), true);
});

test('many short chunks cannot exceed the global point cap', () => {
  const sampled = samplePreviewChunks(Array.from({ length: 100 }, (_, index) => ({
    submapId: String(index), sha256: String(index), points: new Float32Array(3)
  })), 7);
  assert.ok(sampled.reduce((sum, entry) => sum + entry.points.length / 3, 0) <= 7);
});

test('large previews bound downloads and reuse selected chunks after pose changes', () => {
  const refs = Array.from({ length: 100 }, (_, i) => ({ sha256: String(i), size_bytes: 28 }));
  const selected = selectPreviewRefs(refs, 140);
  assert.deepEqual(selected.map((ref) => ref.sha256), ['0', '20', '40', '60', '80']);
  assert.deepEqual(selectPreviewRefs(refs, 140), selected);
  const cache = new ReplicaChunkCache(140);
  for (const ref of selected) cache.set(ref.sha256, new Float32Array(3));
  cache.set('old-component', new Float32Array(3));
  cache.retain(new Set(selected.map((ref) => ref.sha256)));
  assert.equal(cache.get('old-component'), undefined);
  for (const ref of selected) assert.ok(cache.get(ref.sha256));
});
