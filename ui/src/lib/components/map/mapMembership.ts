import type { FrameTransforms } from './overlayFrame.ts';

/**
 * Which robots a map draws: the one rule the 2D canvas and the 3D scene share.
 *
 * A robot is drawn when it is enabled and belongs to what the map shows. A
 * local view belongs to one robot. A global view shows the members of the
 * displayed map (`globalMapMembers`), the same for both views, and every robot
 * a live replica frame carries (`members: null`). With no merge information at
 * all, a global view shows every enabled robot: hiding robots from the
 * operator is the worse failure.
 */
export interface MapMembership {
  /** The robot a local view belongs to; null for a global view. */
  localRobot: string | null;
  /** The displayed source's members; null when it places every robot. */
  members: readonly string[] | null;
  isEnabled: (robotId: string) => boolean;
}

export function isOnMap(robotId: string, membership: MapMembership): boolean {
  if (!membership.isEnabled(robotId)) return false;
  if (membership.localRobot !== null) return robotId === membership.localRobot;
  return membership.members === null || membership.members.includes(robotId);
}

/** The candidates on the map, in their given order. */
export function membersOnMap<T extends { robot_id: string }>(
  candidates: readonly T[],
  membership: MapMembership
): T[] {
  return candidates.filter((robot) => isOnMap(robot.robot_id, membership));
}

/** The robot a map view belongs to: its robot in local mode, else none. */
export function localRobotOf(viewMode: 'global' | 'local', viewRobot: string | null): string | null {
  return viewMode === 'local' ? viewRobot : null;
}

/** What the displayed global map knows about who belongs on it. */
export interface GlobalMapSource {
  showingOptimizedGrid: boolean;
  optimizedRobots?: readonly string[] | null;
  transforms: FrameTransforms;
  globalMembers?: readonly string[] | null;
}

/**
 * Robot ids that belong on the displayed global map, or null when nothing
 * names them and every robot is shown. While an optimized raster is on show,
 * the merged SLAM map's membership says nothing about it (the central SLAM
 * service can be idle with an empty merged grid while the composite or a
 * merged component is displayed), so the members are the robots the displayed
 * scope lists in the catalogue, else every robot the raster's transform header
 * placed. Otherwise the SLAM merged map's own members.
 */
export function globalMapMembers(source: GlobalMapSource): readonly string[] | null {
  const { showingOptimizedGrid, optimizedRobots, transforms, globalMembers } = source;
  if (showingOptimizedGrid) {
    if (optimizedRobots && optimizedRobots.length > 0) return [...optimizedRobots];
    const placed = Object.keys(transforms ?? {});
    return placed.length > 0 ? placed : null;
  }
  return globalMembers && globalMembers.length > 0 ? [...globalMembers] : null;
}
