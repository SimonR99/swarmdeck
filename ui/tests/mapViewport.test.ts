import { test } from 'node:test';
import assert from 'node:assert/strict';
import { rebaseViewport } from '../src/lib/components/map2d/mapViewport.ts';
import { mapSnapshotInfo } from '../src/lib/stores/mapSnapshot.ts';

function screen(view, map, x, y) {
  const gx = (x - map.origin.x) / map.resolution;
  const gy = map.height - (y - map.origin.y) / map.resolution;
  return [
    view.tx + view.scale * (gx * Math.cos(view.rotation) - gy * Math.sin(view.rotation)),
    view.ty + view.scale * (gx * Math.sin(view.rotation) + gy * Math.cos(view.rotation))
  ];
}

test('raster response carries the transform used to place robot overlays', () => {
  const headers = new Headers({
    'X-Map-Resolution': '0.1', 'X-Map-Width': '10', 'X-Map-Height': '20',
    'X-Map-Origin-X': '-1', 'X-Map-Origin-Y': '-2',
    'X-Map-Transforms': '{"r0":{"x":4,"y":-3,"yaw":0.5}}'
  });
  const parsed = mapSnapshotInfo(headers, {
    resolution: 1, width: 1, height: 1, origin: { x: 0, y: 0 }, seq: 1
  });
  assert.deepEqual(parsed.transforms, { r0: { x: 4, y: -3, yaw: 0.5 } });
  const absent = mapSnapshotInfo(new Headers({
    'X-Map-Resolution': '0.1', 'X-Map-Width': '10', 'X-Map-Height': '20',
    'X-Map-Origin-X': '-1', 'X-Map-Origin-Y': '-2'
  }), parsed);
  assert.equal(absent.transforms, undefined);
});

for (const rotation of [0, Math.PI / 2, -0.73]) {
  test(`map growth and resolution updates keep world points fixed at rotation ${rotation}`, () => {
    const original = { resolution: 0.1, height: 100, origin: { x: -4, y: -2 } };
    let map = original;
    let view = { scale: 1.7, tx: 24, ty: -83, rotation };
    const landmarks = [[0, 0], [3, -1], [-2, 6]];
    const expected = landmarks.map(([x, y]) => screen(view, map, x, y));
    for (const next of [
      { resolution: 0.1, height: 110, origin: { x: -4, y: -3 } }, // Growth below the map.
      { resolution: 0.1, height: 150, origin: { x: -6, y: -3 } },
      { resolution: 0.05, height: 300, origin: { x: -6, y: -3 } },
      original
    ]) {
      view = { ...view, ...rebaseViewport(view, map, next) };
      map = next;
      landmarks.forEach(([x, y], index) => {
        screen(view, map, x, y).forEach((coordinate, axis) =>
          assert.ok(Math.abs(coordinate - expected[index][axis]) < 1e-9));
      });
      assert.ok(Math.abs(view.scale / map.resolution - 17) < 1e-9);
    }
  });
}
