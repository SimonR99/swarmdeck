import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  cloudToWorld,
  decalPose,
  routePositions
} from '../src/lib/components/map3d/mapFrames.ts';
import {
  hasQualifiedRasterFrame,
  projectRobotToRaster,
  RasterRobotProjectionCache
} from '../src/lib/components/map2d/mapFrames.ts';
import type { Point, Pose, RobotState } from '../src/lib/types/protocol.ts';
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

test('3D routes preserve map-frame height and only infer absent legacy Z', () => {
  const calls: [number, number][] = [];
  const positions = routePositions([
    { x: 1, y: 2, z: -0.4 },
    { x: 3, y: 4 },
    { x: 5, y: 6, z: 1.75 }
  ], (x, y) => {
    calls.push([x, y]);
    return 0.25;
  });
  assert.deepEqual(positions, [1, 2, -0.4, 3, 4, 0.25, 5, 6, 1.75]);
  assert.deepEqual(calls, [[3, 4]]);
});

test('3D route display sampling remains bounded and retains both endpoints', () => {
  const path = Array.from({ length: 5000 }, (_, x) => ({ x, y: x * 2, z: x / 10 }));
  const positions = routePositions(path);
  assert.equal(positions.length, 1024 * 3);
  assert.deepEqual(positions.slice(0, 3), [0, 0, 0]);
  assert.deepEqual(positions.slice(-3), [4999, 9998, 499.9]);
});

function packet(source: Pose, rawPose: Pose): RobotState {
  const apply = <T extends Point & { yaw?: number }>(point: T): T => {
    const c = Math.cos(source.yaw), s = Math.sin(source.yaw);
    return { ...point, x: source.x + point.x * c - point.y * s,
      y: source.y + point.x * s + point.y * c } as T;
  };
  const rawPath = [{ x: 0, y: 0 }, { x: 4, y: 1 }, { x: 8, y: 2 }];
  return {
    robot_id: 'r0', navigation_transform: source, pose: apply(rawPose),
    goal: apply({ x: 8, y: 2 }), planned_path: rawPath.map(apply),
    global_planned_path: rawPath.map(apply), local_planned_path: rawPath.slice(0, 2).map(apply)
  } as RobotState;
}

test('moving telemetry and a corrected source frame stay fixed on an older raster', () => {
  const raster = { r0: { x: -3, y: 7, yaw: -0.6 } };
  const first = projectRobotToRaster(packet({ x: 2, y: 1, yaw: 0.2 }, { x: 0, y: 0, yaw: 0 }), raster)!;
  const nextPacket = packet({ x: 12, y: -4, yaw: 1.1 }, { x: 1.5, y: 0.2, yaw: 0.3 });
  const second = projectRobotToRaster(nextPacket, raster)!;
  assert.ok(Math.hypot(second.goal!.x - first.goal!.x, second.goal!.y - first.goal!.y) < 1e-12);
  second.global_planned_path!.forEach((point, index) => assert.ok(Math.hypot(
    point.x - first.global_planned_path![index].x,
    point.y - first.global_planned_path![index].y
  ) < 1e-12));
  assert.notDeepEqual(second.pose, first.pose);

  const cache = new RasterRobotProjectionCache();
  assert.equal(cache.project(nextPacket, raster), cache.project(nextPacket, raster));
  const correctedRaster = { r0: { x: -2, y: 7, yaw: -0.6 } };
  assert.notEqual(cache.project(nextPacket, correctedRaster), cache.project(nextPacket, raster));
});

test('missing raster provenance blocks projection and navigation qualification', () => {
  const robot = packet({ x: 2, y: 1, yaw: 0.2 }, { x: 0, y: 0, yaw: 0 });
  assert.equal(projectRobotToRaster(robot, undefined), null);
  assert.equal(hasQualifiedRasterFrame(robot, undefined), false);
  const legacy = { ...robot, navigation_transform: undefined };
  assert.equal(projectRobotToRaster(legacy, undefined), legacy);
  assert.equal(hasQualifiedRasterFrame(legacy, undefined), true);
});
