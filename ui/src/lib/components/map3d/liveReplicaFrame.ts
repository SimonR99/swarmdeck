import type { MapRobot } from '../map2d/mapLayers.ts';
import type { ReplicaTacticalSelection } from './replicaTactical.ts';

export const LIVE_REPLICA_FRESHNESS_BUDGET_S = 3;

export interface LiveReplicaPoint {
  x: number;
  y: number;
  z: number;
}

export interface LiveReplicaPose extends LiveReplicaPoint {
  yaw: number;
}

export interface LiveReplicaFreshness {
  pose_s: number;
  goal_s: number | null;
  path_s: number | null;
}

export interface LiveReplicaRobot {
  robot_id: string;
  mission_id: string;
  component_id: string;
  navigation_frame: string;
  T_component_navigation: number[][];
  pose: LiveReplicaPose;
  goal: (LiveReplicaPoint & { yaw: number }) | null;
  planned_path: LiveReplicaPoint[];
  global_planned_path: LiveReplicaPoint[];
  local_planned_path: LiveReplicaPoint[];
  freshness: LiveReplicaFreshness;
  robot_type?: string;
  nav_status?: string;
  mode?: string;
}

export interface LiveReplicaFrame {
  version: 1;
  mission_id: string;
  session_id: string;
  component_id: string;
  frame_id: string;
  solution_order: [number, number];
  robots: LiveReplicaRobot[];
}

export interface LiveReplicaSelection {
  frame: LiveReplicaFrame;
  receivedAt: number;
}

export function liveReplicaMatchesSelection(
  live: LiveReplicaSelection | null,
  selection: ReplicaTacticalSelection,
  displayedSolutionOrder: unknown
): boolean {
  let displayed: [number, number];
  try {
    displayed = solutionOrder(displayedSolutionOrder, true);
  } catch {
    return false;
  }
  return Boolean(
    live &&
    live.frame.mission_id === selection.sessionId &&
    live.frame.session_id === selection.sessionId &&
    live.frame.component_id === selection.componentId &&
    live.frame.solution_order[0] === displayed[0] &&
    live.frame.solution_order[1] === displayed[1]
  );
}

export class LiveReplicaHttpError extends Error {
  readonly status: number;

  constructor(status: number) {
    super(`Live replica telemetry unavailable (${status})`);
    this.name = 'LiveReplicaHttpError';
    this.status = status;
  }
}

function invalid(message: string): never {
  throw new Error(`Invalid live replica telemetry: ${message}`);
}

function text(value: unknown, field: string): string {
  if (typeof value !== 'string' || value.length === 0) invalid(`${field} is invalid`);
  return value;
}

function number(value: unknown, field: string): number {
  if (typeof value !== 'number' || !Number.isFinite(value)) invalid(`${field} is invalid`);
  return value;
}

function nonnegative(value: unknown, field: string): number {
  const result = number(value, field);
  if (result < 0) invalid(`${field} is invalid`);
  return result;
}

function solutionOrder(value: unknown, allowNullSentinel = false): [number, number] {
  if (allowNullSentinel && value === null) return [0, -1];
  if (!Array.isArray(value) || value.length !== 2 ||
      !Number.isSafeInteger(value[0]) || !Number.isSafeInteger(value[1])) {
    invalid('solution_order is invalid');
  }
  const result: [number, number] = [value[0] as number, value[1] as number];
  if (result[0] < 0 || result[1] < -1 || (result[1] === -1 && result[0] !== 0)) {
    invalid('solution_order is invalid');
  }
  return result;
}

function point(value: unknown, field: string): LiveReplicaPoint {
  if (!value || typeof value !== 'object') invalid(`${field} is invalid`);
  const item = value as Record<string, unknown>;
  return Object.freeze({
    x: number(item.x, `${field}.x`),
    y: number(item.y, `${field}.y`),
    z: item.z === undefined ? 0 : number(item.z, `${field}.z`)
  });
}

function pose(value: unknown, field: string): LiveReplicaPose {
  const item = point(value, field);
  const source = value as Record<string, unknown>;
  return Object.freeze({ ...item, yaw: number(source.yaw, `${field}.yaw`) });
}

function path(value: unknown, field: string): LiveReplicaPoint[] {
  if (!Array.isArray(value)) invalid(`${field} is invalid`);
  return Object.freeze(value.map((item, index) => point(item, `${field}[${index}]`))) as unknown as LiveReplicaPoint[];
}

function matrix(value: unknown): number[][] {
  if (!Array.isArray(value) || value.length !== 4 || value.some((row) => !Array.isArray(row) || row.length !== 4)) {
    invalid('T_component_navigation is invalid');
  }
  const rows = value as unknown[][];
  const result = rows.map((row) => row.map((entry: unknown) => number(entry, 'T_component_navigation')));
  const bottom = result[3];
  if (Math.abs(bottom[0]) > 1e-5 || Math.abs(bottom[1]) > 1e-5 ||
      Math.abs(bottom[2]) > 1e-5 || Math.abs(bottom[3] - 1) > 1e-5) {
    invalid('T_component_navigation is not homogeneous');
  }
  const r = result.slice(0, 3).map((row) => row.slice(0, 3));
  const rowNorms = r.map((row) => Math.hypot(...row));
  const dot = (a: number[], b: number[]) => a.reduce((sum, value, index) => sum + value * b[index], 0);
  if (rowNorms.some((norm) => Math.abs(norm - 1) > 1e-4) ||
      Math.abs(dot(r[0], r[1])) > 1e-4 || Math.abs(dot(r[0], r[2])) > 1e-4 ||
      Math.abs(dot(r[1], r[2])) > 1e-4) {
    invalid('T_component_navigation rotation is invalid');
  }
  const determinant =
    r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) -
    r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
    r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
  if (Math.abs(determinant - 1) > 1e-4) invalid('T_component_navigation rotation is invalid');
  return Object.freeze(result.map((row) => Object.freeze(row))) as unknown as number[][];
}

function freshness(value: unknown): LiveReplicaFreshness {
  if (!value || typeof value !== 'object') invalid('freshness is invalid');
  const item = value as Record<string, unknown>;
  const optionalAge = (entry: unknown, field: string) =>
    entry === null ? null : nonnegative(entry, field);
  return {
    pose_s: nonnegative(item.pose_s, 'freshness.pose_s'),
    goal_s: optionalAge(item.goal_s, 'freshness.goal_s'),
    path_s: optionalAge(item.path_s, 'freshness.path_s')
  };
}

function robot(value: unknown, frame: { mission_id: string; component_id: string }): LiveReplicaRobot {
  if (!value || typeof value !== 'object') invalid('robot is invalid');
  const item = value as Record<string, unknown>;
  const missionId = text(item.mission_id, 'robot.mission_id');
  const componentId = text(item.component_id, 'robot.component_id');
  if (missionId !== frame.mission_id || componentId !== frame.component_id) {
    invalid('robot frame identity does not match response');
  }
  const goal = item.goal === null ? null : pose(item.goal, 'robot.goal');
  return {
    robot_id: text(item.robot_id, 'robot.robot_id'),
    mission_id: missionId,
    component_id: componentId,
    navigation_frame: text(item.navigation_frame, 'robot.navigation_frame'),
    T_component_navigation: matrix(item.T_component_navigation),
    pose: pose(item.pose, 'robot.pose'),
    goal,
    planned_path: path(item.planned_path, 'robot.planned_path'),
    global_planned_path: path(item.global_planned_path ?? [], 'robot.global_planned_path'),
    local_planned_path: path(item.local_planned_path ?? [], 'robot.local_planned_path'),
    freshness: freshness(item.freshness),
    robot_type: typeof item.robot_type === 'string' ? item.robot_type : undefined,
    nav_status: typeof item.nav_status === 'string' ? item.nav_status : undefined,
    mode: typeof item.mode === 'string' ? item.mode : undefined
  };
}

export function parseLiveReplicaFrame(value: unknown): LiveReplicaFrame {
  if (!value || typeof value !== 'object') invalid('response is not an object');
  const item = value as Record<string, unknown>;
  if (item.version !== 1 || !Array.isArray(item.robots)) invalid('response envelope is invalid');
  const missionId = text(item.mission_id ?? item.session_id, 'mission_id');
  const sessionId = text(item.session_id ?? item.mission_id, 'session_id');
  return {
    version: 1,
    mission_id: missionId,
    session_id: sessionId,
    component_id: text(item.component_id, 'component_id'),
    frame_id: text(item.frame_id, 'frame_id'),
    solution_order: solutionOrder(item.solution_order),
    robots: item.robots.map((entry) => robot(entry, {
      mission_id: missionId,
      component_id: text(item.component_id, 'component_id')
    }))
  };
}

export function liveReplicaUrl(selection: ReplicaTacticalSelection): string {
  return `/api/autonomy/replicas/components/live/${encodeURIComponent(selection.sessionId)}?component_id=${encodeURIComponent(selection.componentId)}`;
}

export async function fetchLiveReplicaFrame(
  selection: ReplicaTacticalSelection,
  signal?: AbortSignal
): Promise<LiveReplicaFrame | null> {
  const response = await fetch(liveReplicaUrl(selection), { cache: 'no-store', signal });
  if (response.status === 404 || response.status === 409) return null;
  if (!response.ok) throw new LiveReplicaHttpError(response.status);
  const frame = parseLiveReplicaFrame(await response.json());
  if (frame.session_id !== selection.sessionId || frame.component_id !== selection.componentId) {
    invalid('response selection does not match request');
  }
  return frame;
}

function transformPoint(pointValue: LiveReplicaPoint, transform: number[][]): LiveReplicaPoint {
  return {
    x: transform[0][0] * pointValue.x + transform[0][1] * pointValue.y + transform[0][2] * pointValue.z + transform[0][3],
    y: transform[1][0] * pointValue.x + transform[1][1] * pointValue.y + transform[1][2] * pointValue.z + transform[1][3],
    z: transform[2][0] * pointValue.x + transform[2][1] * pointValue.y + transform[2][2] * pointValue.z + transform[2][3]
  };
}

function transformYaw(yaw: number, transform: number[][]): number {
  const x = transform[0][0] * Math.cos(yaw) + transform[0][1] * Math.sin(yaw);
  const y = transform[1][0] * Math.cos(yaw) + transform[1][1] * Math.sin(yaw);
  return Math.atan2(y, x);
}

interface ProjectedLiveRobotGeometry {
  transform: number[][];
  pose: LiveReplicaPoint;
  poseYaw: number;
  goal: LiveReplicaPoint | null;
  plannedPath: LiveReplicaPoint[];
  globalPlannedPath: LiveReplicaPoint[];
  localPlannedPath: LiveReplicaPoint[];
  poseSource: LiveReplicaPose;
  goalSource: LiveReplicaRobot['goal'];
  plannedPathSource: LiveReplicaPoint[];
  globalPlannedPathSource: LiveReplicaPoint[];
  localPlannedPathSource: LiveReplicaPoint[];
}

const projectedRobotGeometry = new WeakMap<LiveReplicaRobot, ProjectedLiveRobotGeometry>();
const emptyPath = Object.freeze([]) as unknown as LiveReplicaPoint[];

function projectedPath(values: LiveReplicaPoint[], transform: number[][]): LiveReplicaPoint[] {
  return Object.freeze(
    values.map((value) => Object.freeze(transformPoint(value, transform)))
  ) as unknown as LiveReplicaPoint[];
}

function projectedGeometry(robot: LiveReplicaRobot): ProjectedLiveRobotGeometry {
  const cached = projectedRobotGeometry.get(robot);
  if (cached && cached.transform === robot.T_component_navigation &&
      cached.poseSource === robot.pose && cached.goalSource === robot.goal &&
      cached.plannedPathSource === robot.planned_path &&
      cached.globalPlannedPathSource === robot.global_planned_path &&
      cached.localPlannedPathSource === robot.local_planned_path) {
    return cached;
  }
  const pose = transformPoint(robot.pose, robot.T_component_navigation);
  const goal = robot.goal ? transformPoint(robot.goal, robot.T_component_navigation) : null;
  const result: ProjectedLiveRobotGeometry = {
    transform: robot.T_component_navigation,
    pose,
    poseYaw: transformYaw(robot.pose.yaw, robot.T_component_navigation),
    goal: goal ? Object.freeze(goal) : null,
    plannedPath: projectedPath(robot.planned_path, robot.T_component_navigation),
    globalPlannedPath: projectedPath(robot.global_planned_path, robot.T_component_navigation),
    localPlannedPath: projectedPath(robot.local_planned_path, robot.T_component_navigation),
    poseSource: robot.pose,
    goalSource: robot.goal,
    plannedPathSource: robot.planned_path,
    globalPlannedPathSource: robot.global_planned_path,
    localPlannedPathSource: robot.local_planned_path
  };
  projectedRobotGeometry.set(robot, result);
  return result;
}

export function liveRobotToMapRobot(robot: LiveReplicaRobot, base?: MapRobot, elapsedS = 0): MapRobot {
  const geometry = projectedGeometry(robot);
  const freshGoal = robot.goal && robot.freshness.goal_s !== null &&
    robot.freshness.goal_s + elapsedS <= LIVE_REPLICA_FRESHNESS_BUDGET_S;
  const freshPath = robot.freshness.path_s !== null &&
    robot.freshness.path_s + elapsedS <= LIVE_REPLICA_FRESHNESS_BUDGET_S;
  return {
    ...base,
    robot_id: robot.robot_id,
    robot_type: robot.robot_type ?? base?.robot_type,
    pose: { x: geometry.pose.x, y: geometry.pose.y, z: geometry.pose.z, yaw: geometry.poseYaw },
    goal: freshGoal ? geometry.goal : null,
    planned_path: freshPath ? geometry.plannedPath : emptyPath,
    global_planned_path: freshPath ? geometry.globalPlannedPath : emptyPath,
    local_planned_path: freshPath ? geometry.localPlannedPath : emptyPath,
    nav_status: (robot.nav_status as MapRobot['nav_status']) ?? base?.nav_status,
    mode: (robot.mode as MapRobot['mode']) ?? base?.mode
  };
}

export async function postLiveReplicaGoal(
  selection: ReplicaTacticalSelection,
  robotId: string,
  displayedSolutionOrder: [number, number] | null,
  goal: { x: number; y: number; z: number; yaw: number },
  signal?: AbortSignal,
  exploreIfUnknown = false
): Promise<void> {
  const response = await fetch(`/api/autonomy/replicas/components/live/${encodeURIComponent(selection.sessionId)}/goal`, {
    method: 'POST',
    cache: 'no-store',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      robot_id: robotId,
      component_id: selection.componentId,
      solution_order: solutionOrder(displayedSolutionOrder, true),
      goal,
      explore_if_unknown: exploreIfUnknown
    }),
    signal
  });
  if (!response.ok) throw new LiveReplicaHttpError(response.status);
  const result = await response.json().catch(() => null) as { ok?: unknown } | null;
  if (!result || result.ok !== true) throw new Error('Live replica goal was not accepted');
}
