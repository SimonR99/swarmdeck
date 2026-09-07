import { test } from 'node:test';
import assert from 'node:assert/strict';
import { cloudToWorld, decalPose } from '../src/lib/components/map3d/mapFrames.ts';
const transform = { x: 10, y: 20, yaw: Math.PI / 2 };

test('single-robot SLAM cloud is not transformed a second time', () => {
  const points = new Float32Array([8, 21, 0.5]);
  cloudToWorld(points, 'world', transform);
  assert.deepEqual([...points], [8, 21, 0.5]);
});

test('local robot cloud and costmap land in the same rotated world frame', () => {
  const points = new Float32Array([1, 2, 0.5]);
  cloudToWorld(points, 'local', transform);
  const pose = decalPose({width: 2, height: 4, resolution: 1, origin:{x:0,y:0}}, transform);
  assert.deepEqual([...points], [8, 21, 0.5]);
  assert.equal(pose.x, points[0]); assert.equal(pose.y, points[1]);
  assert.equal(pose.yaw, Math.PI / 2);
  assert.equal(pose.width, 2); assert.equal(pose.height, 4);
});
