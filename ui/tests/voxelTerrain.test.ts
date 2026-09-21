import { test } from 'node:test';
import assert from 'node:assert/strict';
import { VoxelTerrain } from '../src/lib/components/map3d/voxelTerrain.ts';
import { prepareTerrain } from '../src/lib/components/map3d/terrainData.ts';

function cloud(rgb = [255, 0, 0, 0, 255, 0, 255, 0, 0, 0, 255, 0]) {
  return prepareTerrain({
    positions: new Float32Array([
      0.01, 0.01, 0.01, 0.16, 0.01, 0.31,
      0.01, 0.16, 0.01, 0.16, 0.16, 0.31
    ]),
    owners: new Uint8Array([0, 1, 0, 1]),
    rgb: new Uint8Array(rgb),
    quality: 'low'
  });
}

test('switching representations reuses geometry and leaves existing color buffers untouched', () => {
  const terrain = new VoxelTerrain();
  try {
    terrain.build(cloud(), ['#ff0000', '#00ff00']);
    const voxels = terrain.voxelMesh!;
    const version = voxels.instanceColor!.version;
    terrain.setRenderMode('points');
    const points = terrain.pointsMesh!;
    const colors = points.geometry.getAttribute('color');
    terrain.setRenderMode('mesh');
    const mesh = terrain.surfaceMesh!;
    const meshColors = mesh.geometry.getAttribute('color');
    for (const mode of ['gaussians', 'voxels', 'points', 'mesh'] as const)
      terrain.setRenderMode(mode);
    assert.equal(terrain.voxelMesh, voxels);
    assert.equal(terrain.pointsMesh, points);
    assert.equal(terrain.surfaceMesh, mesh);
    assert.equal(voxels.instanceColor!.version, version);
    assert.equal(points.geometry.getAttribute('color'), colors);
    assert.equal(mesh.geometry.getAttribute('color'), meshColors);
    assert.equal(mesh.visible, true);
    assert.equal(points.visible, false);
  } finally {
    terrain.dispose();
  }
});

test('color changes update every allocated representation and new clouds discard stale RGB', () => {
  const terrain = new VoxelTerrain();
  try {
    terrain.build(cloud(), ['#0000ff', '#0000ff']);
    terrain.setRenderMode('points');
    terrain.setRenderMode('mesh');
    terrain.setColorMode('camera');
    const points = terrain.pointsMesh!;
    assert.deepEqual(Array.from(points.geometry.getAttribute('color').array).slice(0, 6), [1, 0, 0, 0, 1, 0]);
    assert.equal(terrain.voxelMesh!.instanceColor!.getX(0), 1);
    assert.equal(terrain.surfaceMesh!.geometry.getAttribute('color').getX(0), 1);
    const colors = points.geometry.getAttribute('color');
    terrain.setColorMode('camera');
    assert.equal(points.geometry.getAttribute('color'), colors);
    let disposed = false;
    points.geometry.addEventListener('dispose', () => {
      disposed = true;
    });
    terrain.setCeilingCutoff(0.2);
    terrain.build(cloud([0, 0, 255, 0, 0, 255, 0, 0, 255, 0, 0, 255]), ['#ff0000']);
    terrain.setRenderMode('points');
    assert.equal(disposed, true);
    assert.notEqual(terrain.pointsMesh, points);
    assert.deepEqual(
      Array.from(terrain.pointsMesh!.geometry.getAttribute('color').array).slice(0, 6),
      [0, 0, 1, 0, 0, 1]
    );
    assert.equal(terrain.ceilingClipPlane.constant, 0.2);
  } finally {
    terrain.dispose();
  }
});

test('Gaussian fallback shows points only until a real reconstruction is available', () => {
  const terrain = new VoxelTerrain();
  try {
    terrain.build(cloud(), ['#ff0000', '#00ff00']);
    terrain.setRenderMode('gaussians');
    assert.equal(terrain.pointsMesh, null);
    terrain.setGaussianProxy(true);
    assert.equal(terrain.pointsMesh!.visible, true);
    assert.equal(terrain.voxelMesh!.visible, false);
    terrain.setGaussianProxy(false);
    assert.equal(terrain.pointsMesh!.visible, false);
  } finally {
    terrain.dispose();
  }
});

test('retiring one robot removes its points and ground while preserving the other robot', () => {
  const terrain = new VoxelTerrain();
  try {
    terrain.build(cloud(), ['#ff0000', '#00ff00']);
    terrain.removeOwner(0, 'low');
    terrain.setRenderMode('points');
    const positions = terrain.pointsMesh!.geometry.getAttribute('position');
    assert.equal(positions.count, 2);
    assert.equal(positions.getX(0), Math.fround(0.16));
    assert.equal(positions.getX(1), Math.fround(0.16));
    assert.equal(terrain.getGroundZ(0.16, 0.01), Math.fround(0.31));
  } finally {
    terrain.dispose();
  }
});
