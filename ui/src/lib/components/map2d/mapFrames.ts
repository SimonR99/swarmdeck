import type { Point, Pose, RobotState } from '../../types/protocol.ts';

export type FrameTransforms = Record<string, Pose> | undefined;

/** What the displayed replica raster knows about who belongs on the canvas. */
export interface GlobalMapMembership {
  showingOptimizedGrid: boolean;
  optimizedRobots?: readonly string[] | null;
  transforms: FrameTransforms;
  globalMembers?: readonly string[] | null;
}

/**
 * Robot ids that belong on the global canvas. While an optimized raster is on
 * show, the merged SLAM map's membership says nothing about it (the central
 * SLAM service can be idle with an empty merged grid while the composite or a
 * merged component is displayed), so the members are the robots the displayed
 * scope lists in the catalogue, else every robot the raster's transform header
 * placed. Otherwise the SLAM merged map's own members, as before.
 */
export function globalMapMembers(membership: GlobalMapMembership): string[] {
  const { showingOptimizedGrid, optimizedRobots, transforms, globalMembers } = membership;
  if (showingOptimizedGrid) {
    if (optimizedRobots && optimizedRobots.length > 0) return [...optimizedRobots];
    return Object.keys(transforms ?? {});
  }
  return globalMembers ? [...globalMembers] : [];
}

/**
 * The rigid transform that places a robot-local overlay on the displayed
 * raster. Transform provenance is accepted only from that raster response.
 */
export function overlayFrameOnGlobalGrid(
  robotId: string,
  rasterFrames: FrameTransforms
): Pose | undefined {
  return rasterFrames?.[robotId];
}

export function hasQualifiedRasterFrame(robot: RobotState, frames: FrameTransforms): boolean {
  return !robot.navigation_transform || Boolean(frames?.[robot.robot_id]);
}

/** The rigid world-to-raster correction for one robot, or null when it has none. */
export interface RasterProjection {
  x: number;
  y: number;
  yaw: number;
  c: number;
  s: number;
}

/**
 * The correction between the transform a robot's telemetry was published with
 * and the one baked into the displayed raster. `undefined` when the robot
 * needs none, `null` when the raster cannot place it.
 */
export function rasterProjection(
  robot: Pick<RobotState, 'robot_id' | 'navigation_transform'>,
  frames: FrameTransforms
): RasterProjection | null | undefined {
  const source = robot.navigation_transform;
  if (!source) return undefined;
  const target = frames?.[robot.robot_id];
  if (!target) return null;
  const yaw = target.yaw - source.yaw;
  const c = Math.cos(yaw), s = Math.sin(yaw);
  return {
    x: target.x - source.x * c + source.y * s,
    y: target.y - source.x * s - source.y * c,
    yaw,
    c,
    s
  };
}

/** Place recorded world-frame trail points on the displayed raster. */
export function projectTrailToRaster(
  points: readonly { x: number; y: number }[],
  projection: RasterProjection | null | undefined
): { x: number; y: number }[] {
  if (projection === undefined) return points as { x: number; y: number }[];
  if (projection === null) return [];
  const { x, y, c, s } = projection;
  return points.map((point) => ({
    x: x + point.x * c - point.y * s,
    y: y + point.x * s + point.y * c
  }));
}

/** Re-express one coherent telemetry packet in the transform baked into the raster. */
export function projectRobotToRaster(robot: RobotState, frames: FrameTransforms): RobotState | null {
  const projection = rasterProjection(robot, frames);
  if (projection === undefined) return robot;
  if (projection === null) return null;
  // Compose once per packet; every path vertex shares this rigid transform.
  const { x, y, c, s, yaw } = projection;
  const point = <T extends Point & { yaw?: number }>(value: T): T => ({
    ...value,
    x: x + value.x * c - value.y * s,
    y: y + value.x * s + value.y * c,
    ...(value.yaw === undefined ? {} : { yaw: value.yaw + yaw })
  });
  return {
    ...robot,
    pose: point(robot.pose),
    goal: robot.goal ? point(robot.goal) : null,
    planned_path: robot.planned_path.map(point),
    global_planned_path: robot.global_planned_path?.map(point),
    local_planned_path: robot.local_planned_path?.map(point)
  };
}

export class RasterRobotProjectionCache {
  private entries = new WeakMap<RobotState, { key: string; value: RobotState | null }>();

  project(robot: RobotState, frames: FrameTransforms): RobotState | null {
    const key = JSON.stringify([robot.navigation_transform, frames?.[robot.robot_id]]);
    const cached = this.entries.get(robot);
    if (cached?.key === key) return cached.value;
    const value = projectRobotToRaster(robot, frames);
    this.entries.set(robot, { key, value });
    return value;
  }
}

/**
 * The same correction for a robot's trail. Recomputed only when the trail
 * gained a point or the raster changed the transform it was published with: a
 * trail is six hundred points and the canvas redraws for many other reasons.
 */
export class RasterTrailProjectionCache {
  private entries = new Map<
    string,
    { key: string; length: number; last: unknown; value: { x: number; y: number }[] }
  >();

  project(
    robot: Pick<RobotState, 'robot_id' | 'navigation_transform'>,
    points: readonly { x: number; y: number }[],
    frames: FrameTransforms
  ): { x: number; y: number }[] {
    const key = JSON.stringify([robot.navigation_transform, frames?.[robot.robot_id]]);
    const last = points[points.length - 1] ?? null;
    const cached = this.entries.get(robot.robot_id);
    if (cached && cached.key === key && cached.length === points.length && cached.last === last) {
      return cached.value;
    }
    const value = projectTrailToRaster(points, rasterProjection(robot, frames));
    this.entries.set(robot.robot_id, { key, length: points.length, last, value });
    return value;
  }

  clear() {
    this.entries.clear();
  }
}
