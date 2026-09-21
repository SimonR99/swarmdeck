import { postLiveReplicaGoal } from '../map3d/liveReplicaFrame.ts';
import { isComponentScope, isDeploymentScope } from '../../stores/optimizedScopes.ts';

/**
 * Every displayed optimized raster uses the live replica goal endpoint. The
 * solution order returned by that endpoint fences the click against a newer
 * raster publication before dispatch.
 */
export function usesLiveRasterGoal(viewMode: string, scope: string | null): scope is string {
  return (viewMode === 'global' || viewMode === 'local') &&
    scope !== null && (isComponentScope(scope) || isDeploymentScope(scope));
}

interface LiveRobot {
  robot_id: string;
  pose: { x: number; y: number; z?: number; yaw?: number };
  T_component_navigation: number[][] | number[];
}

interface LiveView {
  session_id: string;
  solution_order: [number, number] | null;
  robots: LiveRobot[];
}

function matrix(value: number[][] | number[]): number[][] {
  if (value.length === 16 && typeof value[0] === 'number') {
    const flat = value as number[];
    return [0, 1, 2, 3].map((row) => flat.slice(row * 4, row * 4 + 4));
  }
  return value as number[][];
}

/** The robot's live pose expressed in the component frame, heading toward `goal`. */
export function componentGoal(
  robot: LiveRobot,
  goal: { x: number; y: number }
): { x: number; y: number; z: number; yaw: number } {
  const T = matrix(robot.T_component_navigation);
  const p = [robot.pose.x, robot.pose.y, robot.pose.z ?? 0, 1];
  const here = [0, 1, 2].map((row) => T[row].reduce((sum, value, column) => sum + value * p[column], 0));
  return { x: goal.x, y: goal.y, z: here[2], yaw: Math.atan2(goal.y - here[1], goal.x - here[0]) };
}

export async function postGlobalRasterGoal(
  robotId: string,
  scope: string,
  goal: { x: number; y: number },
  fetchImpl: typeof fetch = fetch
): Promise<void> {
  const catalogue = await fetchImpl('/api/autonomy/replicas/components', { cache: 'no-store' });
  if (!catalogue.ok) throw new Error(`replica catalogue ${catalogue.status}`);
  const sessionId = String(((await catalogue.json()) as { active_session_id?: string }).active_session_id ?? '');
  if (!sessionId) throw new Error('No active mission');
  const live = await fetchImpl(
    `/api/autonomy/replicas/components/live/${encodeURIComponent(sessionId)}?component_id=${encodeURIComponent(scope)}`,
    { cache: 'no-store' }
  );
  if (!live.ok) throw new Error(`live component ${live.status}`);
  const view = (await live.json()) as LiveView;
  const robot = view.robots.find((entry) => entry.robot_id === robotId);
  if (!robot) throw new Error(`${robotId} has no fresh telemetry in this component`);
  await postLiveReplicaGoal(
    { robotId, sessionId, componentId: scope },
    robotId,
    view.solution_order,
    componentGoal(robot, goal)
  );
}
