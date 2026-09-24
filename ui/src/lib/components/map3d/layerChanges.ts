import type { MapRobot } from '../map/mapRobot.ts';

/**
 * Change detection for the 3D layer groups.
 *
 * The layers used to stringify their inputs every 200 ms to find out whether
 * anything had moved, which cost more than the rebuild it was avoiding. Each
 * layer now declares the values it is built from — primitives, and references
 * the fleet store keeps stable while their contents are unchanged — and they
 * are compared position by position.
 */
export class LayerDependencies {
  private values = new Map<string, unknown[]>();

  changed(key: string, values: unknown[]): boolean {
    const previous = this.values.get(key);
    this.values.set(key, values);
    if (!previous || previous.length !== values.length) return true;
    for (let i = 0; i < values.length; i++) {
      if (!Object.is(previous[i], values[i])) return true;
    }
    return false;
  }

  clear() {
    this.values.clear();
  }
}

/** Goal beacons follow both the goal and the robot the guide line starts at. */
export function goalDependencies(robots: readonly MapRobot[], showPlans: boolean): unknown[] {
  const values: unknown[] = [showPlans, robots.length];
  for (const robot of robots) {
    values.push(
      robot.robot_id,
      robot.pose.x,
      robot.pose.y,
      robot.goal?.x ?? null,
      robot.goal?.y ?? null,
      robot.goal?.z ?? null,
      robot.nav_status,
      robot.mode
    );
  }
  return values;
}

/** Routes are rebuilt only when a planner publishes different vertices. */
export function pathDependencies(
  robots: readonly MapRobot[],
  showPlans: boolean,
  selected: readonly string[]
): unknown[] {
  const values: unknown[] = [showPlans, selected, robots.length];
  for (const robot of robots) {
    values.push(
      robot.robot_id,
      robot.nav_status,
      robot.mode,
      robot.goal,
      robot.planned_path,
      robot.global_planned_path,
      robot.local_planned_path
    );
  }
  return values;
}

/**
 * Trails grow by appending a point, so the last point's identity reports a new
 * sample even once a trail is long enough to be dropping its oldest one.
 */
export function trailDependencies(
  robots: readonly MapRobot[],
  trails: ReadonlyMap<string, { x: number; y: number }[]>,
  showTrails: boolean
): unknown[] {
  const values: unknown[] = [showTrails, robots.length];
  for (const robot of robots) {
    const trail = trails.get(robot.robot_id);
    values.push(robot.robot_id, trail?.length ?? 0, trail?.[(trail?.length ?? 0) - 1] ?? null);
  }
  return values;
}

/** Inter-robot closure lines are drawn between the two robots' current poses. */
export function loopDependencies(
  robots: readonly MapRobot[],
  showPlans: boolean,
  slamGraphs: unknown
): unknown[] {
  const values: unknown[] = [showPlans, slamGraphs, robots.length];
  for (const robot of robots) values.push(robot.robot_id, robot.pose.x, robot.pose.y);
  return values;
}
