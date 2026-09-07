export interface MapTransform { x: number; y: number; yaw: number }

/** Robot clouds can be local; the SLAM fallback already publishes world XYZ. */
export function cloudToWorld(
  positions: Float32Array, frame: string | null, transform?: MapTransform
) {
  if (frame === 'world' || !transform) return;
  const c = Math.cos(transform.yaw), s = Math.sin(transform.yaw);
  for (let i = 0; i < positions.length; i += 3) {
    const x = positions[i], y = positions[i + 1];
    positions[i] = transform.x + x * c - y * s;
    positions[i + 1] = transform.y + x * s + y * c;
  }
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
