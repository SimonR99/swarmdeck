import type { Footprint, RobotState } from '../../types/protocol.ts';

export type RoutePoint = { x: number; y: number; z?: number };

/** A robot as the 2D canvas and the 3D scene draw it. */
export interface MapRobot {
  robot_id: string;
  robot_type?: string;
  pose: { x: number; y: number; z?: number; yaw: number };
  planned_path?: RoutePoint[];
  global_planned_path?: RoutePoint[];
  local_planned_path?: RoutePoint[];
  footprint_radius?: number;
  footprint?: Footprint | null;
  goal?: RoutePoint | null;
  nav_status?: RobotState['nav_status'];
  mode?: RobotState['mode'];
}

/** A robot shows its route and goal while navigating: an active goal, nav mode, or any goal. */
export function isNavigating(robot: MapRobot): boolean {
  return robot.nav_status === 'active' || robot.mode === 'nav' || Boolean(robot.goal);
}

/** The routes and goal a map draws for one robot; null where nothing is drawn. */
export interface DisplayedRoute {
  /** The planner's route: the global plan when published, else the planned path. */
  global: RoutePoint[] | null;
  /** The controller's local trajectory. */
  local: RoutePoint[] | null;
  goal: RoutePoint | null;
}

const NO_ROUTE: DisplayedRoute = Object.freeze({ global: null, local: null, goal: null });

function drawable(path: RoutePoint[] | undefined): RoutePoint[] | null {
  return path && path.length >= 2 ? path : null;
}

/** The same for 2D and 3D: nothing while idle, and a route needs two vertices. */
export function displayedRoute(robot: MapRobot): DisplayedRoute {
  if (!isNavigating(robot)) return NO_ROUTE;
  const global = robot.global_planned_path && robot.global_planned_path.length > 0
    ? robot.global_planned_path
    : robot.planned_path;
  return {
    global: drawable(global),
    local: drawable(robot.local_planned_path),
    goal: robot.goal ?? null
  };
}
