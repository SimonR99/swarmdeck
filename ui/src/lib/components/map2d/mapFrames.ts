import type { Point, Pose, RobotState } from '../../types/protocol.ts';

export type FrameTransforms = Record<string, Pose> | undefined;

/** What the global view knows about who belongs on the grid it displays. */
export interface GlobalMapMembership {
  /** True while the global canvas shows a server-rasterized optimized grid. */
  showingOptimizedGrid: boolean;
  /** `robots` of that grid's catalogue entry (GET /api/map/optimized), when listed. */
  optimizedRobots?: readonly string[] | null;
  /** `X-Map-Transforms` of the displayed raster, keyed by robot id. */
  transforms: FrameTransforms;
  /** The SLAM merged map's membership from GET /api/map/status. */
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
 * The rigid transform that places a robot-local overlay (its Nav2 costmap, in
 * its own map frame) on the global canvas. A server-rasterized optimized grid
 * carries every member's frame in `X-Map-Transforms`, the same transform the
 * robots themselves are projected with; only the legacy SLAM merged map,
 * which sends no such header, falls back to the status transforms, which are
 * the surveyed start poses in the deployment frame and rotate an overlay by a
 * whole start yaw when applied to a component raster.
 */
export function overlayFrameOnGlobalGrid(
  robotId: string,
  rasterFrames: FrameTransforms,
  statusTransforms: Record<string, Pose> | undefined
): Pose | undefined {
  return rasterFrames?.[robotId] ?? statusTransforms?.[robotId];
}

export function hasQualifiedRasterFrame(robot: RobotState, frames: FrameTransforms): boolean {
  return !robot.navigation_transform || Boolean(frames?.[robot.robot_id]);
}

/** Re-express one coherent telemetry packet in the transform baked into the raster. */
export function projectRobotToRaster(robot: RobotState, frames: FrameTransforms): RobotState | null {
  const source = robot.navigation_transform;
  if (!source) return robot;
  const target = frames?.[robot.robot_id];
  if (!target) return null;
  // Compose once per packet; every path vertex shares this rigid transform.
  const yaw = target.yaw - source.yaw;
  const c = Math.cos(yaw), s = Math.sin(yaw);
  const x = target.x - source.x * c + source.y * s;
  const y = target.y - source.x * s - source.y * c;
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
