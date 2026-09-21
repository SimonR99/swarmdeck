export interface MapTransform { x: number; y: number; yaw: number }

export interface MapPoint3D { x: number; y: number; z?: number }

/**
 * Preserve a planner-provided map-frame height. Planar legacy paths fall back
 * to the rendered terrain, but a real Z value is never rescaled or replaced.
 */
export function mapFrameZ(
  point: MapPoint3D,
  getGroundZ?: (x: number, y: number) => number
): number {
  return point.z !== undefined && Number.isFinite(point.z)
    ? point.z
    : (getGroundZ?.(point.x, point.y) ?? 0);
}

/** Sample a display route without changing its metric XYZ or either endpoint. */
export function routePositions(
  path: MapPoint3D[],
  getGroundZ?: (x: number, y: number) => number,
  maxPoints = 1024
): number[] {
  if (path.length === 0) return [];
  const count = Math.min(path.length, Math.max(2, Math.floor(maxPoints)));
  const positions: number[] = [];
  for (let i = 0; i < count; i++) {
    const point = path[Math.round(i * (path.length - 1) / Math.max(1, count - 1))];
    positions.push(point.x, point.y, mapFrameZ(point, getGroundZ));
  }
  return positions;
}


/** Costmaps and network grids are always in their source robot's map frame. */
export function decalPose(
  info: { width: number; height: number; resolution: number; origin: { x: number; y: number } },
  transform?: MapTransform
) {
  const width = info.width * info.resolution, height = info.height * info.resolution;
  const x = info.origin.x + width / 2, y = info.origin.y + height / 2;
  const yaw = transform?.yaw ?? 0, c = Math.cos(yaw), s = Math.sin(yaw);
  return { width, height, x: (transform?.x ?? 0) + x*c-y*s, y: (transform?.y ?? 0) + x*s+y*c, yaw };
}
