import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  decalPose,
  routePositions
} from '../src/lib/components/map3d/mapFrames.ts';
import {
  hasQualifiedRasterFrame,
  projectRobotToRaster,
  projectTrailToRaster,
  rasterProjection,
  RasterRobotProjectionCache,
  RasterTrailProjectionCache
} from '../src/lib/components/map2d/mapFrames.ts';
import { globalMapMembers } from '../src/lib/components/map/mapMembership.ts';
import {
  applyPlanarTransform,
  overlayFrameOnGlobalGrid,
  planarTransform
} from '../src/lib/components/map/overlayFrame.ts';
import type { Point, Pose, RobotState } from '../src/lib/types/protocol.ts';
import { TrailRecorder } from '../src/lib/stores/trailRecorder.ts';
const transform = { x: 10, y: 20, yaw: Math.PI / 2 };


test('network decals preserve the raster frame transform', () => {
  const pose = decalPose({width: 2, height: 4, resolution: 1, origin:{x:0,y:0}}, transform);
  assert.equal(pose.x, 8); assert.equal(pose.y, 21);
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

test('a recorded trail is placed on the raster with the robot that drove it', () => {
  const raster = { r0: { x: -3, y: 7, yaw: -0.6 } };
  const source = { x: 2, y: 1, yaw: 0.2 };
  const robot = packet(source, { x: 0, y: 0, yaw: 0 });
  // The trail holds the poses the robot reported, so projecting the trail and
  // projecting the robot must put the last point under the robot.
  const trail = [{ x: robot.pose.x, y: robot.pose.y }];
  const placed = projectTrailToRaster(trail, rasterProjection(robot, raster));
  const projected = projectRobotToRaster(robot, raster)!;
  assert.ok(Math.hypot(placed[0].x - projected.pose.x, placed[0].y - projected.pose.y) < 1e-12);

  // A robot the raster cannot place has nothing to draw; a robot that needs no
  // correction keeps its recorded points.
  assert.deepEqual(projectTrailToRaster(trail, rasterProjection(robot, undefined)), []);
  const legacy = { ...robot, navigation_transform: undefined };
  assert.equal(projectTrailToRaster(trail, rasterProjection(legacy, undefined)), trail);
});

test('mixed registration packets cannot create a fictitious trail on an unchanged raster', () => {
  const recorder = new TrailRecorder();
  const raster = { r0: { x: 0, y: 0, yaw: 0 } };
  let robot = packet(raster.r0, { x: 0, y: 0, yaw: 0 });
  recorder.record('r0', robot.pose.x, robot.pose.y, robot.navigation_transform);
  const cache = new RasterTrailProjectionCache();
  assert.deepEqual(cache.project(robot, recorder.points('r0'), raster), [{ x: 0, y: 0 }]);
  robot = packet({ x: 1, y: 0, yaw: 0 }, { x: 0, y: 0, yaw: 0 });
  recorder.record('r0', robot.pose.x, robot.pose.y, robot.navigation_transform);
  assert.deepEqual(cache.project(robot, recorder.points('r0'), raster), [{ x: 0, y: 0 }]);
});

test('trail placement is recomputed for a new point or a new raster, not for a redraw', () => {
  const raster = { r0: { x: -3, y: 7, yaw: -0.6 } };
  const robot = packet({ x: 2, y: 1, yaw: 0.2 }, { x: 0, y: 0, yaw: 0 });
  const trail = [{ x: 1, y: 1 }];
  const cache = new RasterTrailProjectionCache();
  const first = cache.project(robot, trail, raster);
  assert.equal(cache.project(robot, trail, raster), first);
  trail.push({ x: 1.4, y: 1 });
  assert.notEqual(cache.project(robot, trail, raster), first);
  const moved = cache.project(robot, trail, raster);
  assert.notEqual(cache.project(robot, trail, { r0: { x: -2, y: 7, yaw: -0.6 } }), moved);
});

test('missing raster provenance blocks projection and navigation qualification', () => {
  const robot = packet({ x: 2, y: 1, yaw: 0.2 }, { x: 0, y: 0, yaw: 0 });
  assert.equal(projectRobotToRaster(robot, undefined), null);
  assert.equal(hasQualifiedRasterFrame(robot, undefined), false);
  const legacy = { ...robot, navigation_transform: undefined };
  assert.equal(projectRobotToRaster(legacy, undefined), legacy);
  assert.equal(hasQualifiedRasterFrame(legacy, undefined), true);
});

const rasterFrames = { robot_0: { x: 0, y: 0, yaw: 0 }, robot_1: { x: 4, y: -2, yaw: 1.2 } };

test('an optimized raster on show is populated by its catalogue scope, not the SLAM merge', () => {
  // Measured on benchbot: the composite raster is displayed while the central
  // SLAM service is idle and reports no merged-map members at all.
  assert.deepEqual(
    globalMapMembers({
      showingOptimizedGrid: true,
      optimizedRobots: ['robot_0', 'robot_1'],
      transforms: rasterFrames,
      globalMembers: []
    }),
    ['robot_0', 'robot_1']
  );
  // The catalogue entry wins over both the transform header and the SLAM list.
  assert.deepEqual(
    globalMapMembers({
      showingOptimizedGrid: true,
      optimizedRobots: ['robot_0', 'robot_1'],
      transforms: { ...rasterFrames, robot_2: { x: 1, y: 1, yaw: 0 } },
      globalMembers: ['robot_2']
    }),
    ['robot_0', 'robot_1']
  );
});

test('an optimized raster whose scope the index no longer lists falls back to its transform header', () => {
  assert.deepEqual(
    globalMapMembers({
      showingOptimizedGrid: true,
      optimizedRobots: undefined,
      transforms: rasterFrames,
      globalMembers: []
    }),
    ['robot_0', 'robot_1']
  );
  // A listed scope with no robots is as good as unlisted.
  assert.deepEqual(
    globalMapMembers({
      showingOptimizedGrid: true,
      optimizedRobots: [],
      transforms: rasterFrames,
      globalMembers: ['robot_2']
    }),
    ['robot_0', 'robot_1']
  );
});

test('the SLAM merged grid keeps its own membership', () => {
  assert.deepEqual(
    globalMapMembers({
      showingOptimizedGrid: false,
      optimizedRobots: ['robot_0', 'robot_1'],
      transforms: rasterFrames,
      globalMembers: ['robot_1', 'robot_2']
    }),
    ['robot_1', 'robot_2']
  );
  assert.deepEqual(
    globalMapMembers({ showingOptimizedGrid: false, transforms: undefined, globalMembers: [] }),
    null
  );
});

test('nothing known about the global map places every robot on it', () => {
  // Operator decision: hiding robots is the worse failure, so both views show
  // everyone rather than nobody (the 2D canvas used to show nobody here).
  assert.equal(globalMapMembers({ showingOptimizedGrid: false, transforms: undefined }), null);
  assert.equal(globalMapMembers({ showingOptimizedGrid: true, transforms: undefined }), null);
  assert.equal(
    globalMapMembers({ showingOptimizedGrid: true, optimizedRobots: null, transforms: {}, globalMembers: null }),
    null
  );
});

test('network heatmaps require transform provenance from the displayed raster header', () => {
  const rasterFrames = { robot_2: { x: -0.02, y: -2.0, yaw: 0.0015 } };
  assert.deepEqual(overlayFrameOnGlobalGrid('robot_2', rasterFrames), rasterFrames.robot_2);
  assert.equal(overlayFrameOnGlobalGrid('robot_2', undefined), undefined);
  assert.equal(overlayFrameOnGlobalGrid('robot_9', rasterFrames), undefined);
});


test('the 2D raster and the 3D decal place overlays with the same rigid transform', () => {
  const pose = { x: 3.2, y: -1.7, yaw: 0.9 };
  const point = { x: 4.1, y: 2.3 };
  const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
  const expected = { x: pose.x + point.x * c - point.y * s, y: pose.y + point.x * s + point.y * c };
  assert.deepEqual(applyPlanarTransform(planarTransform(pose), point), expected);
  const decal = decalPose({ width: 0, height: 0, resolution: 1, origin: point }, pose);
  assert.deepEqual({ x: decal.x, y: decal.y }, expected);
  const identity = decalPose({ width: 2, height: 4, resolution: 0.5, origin: { x: -1, y: 1 } });
  assert.deepEqual(identity, { width: 1, height: 2, x: -0.5, y: 2, yaw: 0 });
});
