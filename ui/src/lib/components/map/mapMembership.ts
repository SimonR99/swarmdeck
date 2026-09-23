/**
 * Which robots a map draws: the one rule the 2D canvas and the 3D scene share.
 *
 * A robot is drawn when it is enabled and belongs to what the map shows. A
 * local view belongs to one robot. A global view shows the members of its
 * source, which each view names: the displayed raster's scope for the canvas
 * (`globalMapMembers` in map2d/mapFrames.ts), the SLAM merge for the 3D scene
 * without a replica (`slamMergeMembers`), and every robot a live replica frame
 * carries (`members: null`).
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

/** The SLAM merged map's members, or every robot while it reports none. */
export function slamMergeMembers(globalMembers: readonly string[] | null | undefined): readonly string[] | null {
  return globalMembers && globalMembers.length > 0 ? globalMembers : null;
}
