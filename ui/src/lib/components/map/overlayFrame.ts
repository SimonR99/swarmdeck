import type { Pose } from '../../types/protocol.ts';

/** The per-robot transforms the displayed raster's response carried. */
export type FrameTransforms = Record<string, Pose> | undefined;

/** A rigid planar transform with its rotation precomputed. */
export interface PlanarTransform {
  x: number;
  y: number;
  yaw: number;
  c: number;
  s: number;
}

export function planarTransform(pose: { x: number; y: number; yaw: number }): PlanarTransform {
  return { x: pose.x, y: pose.y, yaw: pose.yaw, c: Math.cos(pose.yaw), s: Math.sin(pose.yaw) };
}

export function applyPlanarTransform(
  transform: PlanarTransform,
  point: { x: number; y: number }
): { x: number; y: number } {
  const { x, y, c, s } = transform;
  return { x: x + point.x * c - point.y * s, y: y + point.x * s + point.y * c };
}

/**
 * The rigid transform that places a robot-local overlay (its network heatmap)
 * on the displayed raster, in 2D and 3D alike. Transform provenance is
 * accepted only from that raster response.
 */
export function overlayFrameOnGlobalGrid(
  robotId: string,
  rasterFrames: FrameTransforms
): Pose | undefined {
  return rasterFrames?.[robotId];
}
