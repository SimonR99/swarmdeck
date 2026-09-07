import { test } from 'node:test';
import assert from 'node:assert/strict';
import { prepareTerrain, QUALITY } from '../src/lib/components/map3d/terrainData.ts';

test('adjacent occupied cells share no internal mesh faces', () => {
  const d = prepareTerrain({
    positions: new Float32Array([0.01, 0.01, 0.01, 0.16, 0.01, 0.01]),
    owners: new Uint8Array([0, 1]),
    quality: 'low'
  });
  assert.equal(d.samples.length, 2);
  assert.equal(d.meshPositions.length / 9, 20); // ten quads, not twelve
  // Every triangle winds toward its outward normal.
  for (let i = 0; i < d.meshPositions.length; i += 9) {
    const p = d.meshPositions,
      n = d.meshNormals;
    const a = [p[i + 3] - p[i], p[i + 4] - p[i + 1], p[i + 5] - p[i + 2]],
      b = [p[i + 6] - p[i], p[i + 7] - p[i + 1], p[i + 8] - p[i + 2]];
    assert.ok(
      (a[1] * b[2] - a[2] * b[1]) * n[i] +
        (a[2] * b[0] - a[0] * b[2]) * n[i + 1] +
        (a[0] * b[1] - a[1] * b[0]) * n[i + 2] >
        0
    );
  }
});

test('negative coordinates, invalid samples, and color ownership survive reduction', () => {
  const d = prepareTerrain({
    positions: new Float32Array([-0.01, 0, 0, NaN, 0, 0, 0.01, 0, 0]),
    owners: new Uint8Array([3, 9, 4]),
    rgb: new Uint8Array([255, 0, 0, 0, 0, 0, 0, 255, 0]),
    quality: 'low'
  });
  assert.equal(d.xyz.length, 6);
  assert.deepEqual([...d.owners], [3, 4]);
  assert.deepEqual([...d.rgb!], [255, 0, 0, 0, 255, 0]);
  assert.ok(d.centers[0] < 0);
  assert.ok(d.centers[3] > 0);
});

test('large clouds respect low-power budgets by coarsening the full extent', () => {
  const xyz = new Float32Array(100000 * 3);
  for (let i = 0; i < 100000; i++) {
    xyz[i * 3] = (i % 100) * 0.16;
    xyz[i * 3 + 1] = (Math.floor(i / 100) % 100) * 0.16;
    xyz[i * 3 + 2] = Math.floor(i / 10000) * 0.16;
  }
  const d = prepareTerrain({ positions: xyz, owners: new Uint8Array(100000), quality: 'low' });
  assert.ok(d.xyz.length / 3 <= QUALITY.low.points);
  assert.ok(d.samples.length <= QUALITY.low.voxels);
  assert.ok(d.size > 0.15);
  assert.ok(d.centers.some((v) => v > 15));
});

test('empty clouds produce no geometry', () => {
  const d = prepareTerrain({
    positions: new Float32Array(),
    owners: new Uint8Array(),
    quality: 'low'
  });
  assert.equal(d.meshPositions.length, 0);
  assert.equal(d.centers.length, 0);
});
