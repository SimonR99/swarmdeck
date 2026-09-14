import type { Point, Pose, RobotState } from '../../types/protocol.ts';

export type FrameTransforms = Record<string, Pose> | undefined;

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
