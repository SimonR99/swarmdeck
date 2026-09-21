/** One entry from GET /api/map/optimized: a rasterized replica product. */
export interface OptimizedScope {
  /** `component:*` is a verified multi-robot component; `deployment:*` is the surveyed fallback. */
  scope: string;
  robots: string[];
  resolution: number;
  width: number;
  height: number;
  origin: { x: number; y: number };
  seq?: number;
}

const COMPONENT_PREFIX = 'component:';
const DEPLOYMENT_PREFIX = 'deployment:';
const ROBOT_PREFIX = 'robot:';

export function isComponentScope(scope: string): boolean {
  return scope.startsWith(COMPONENT_PREFIX);
}

export function isDeploymentScope(scope: string): boolean {
  return scope.startsWith(DEPLOYMENT_PREFIX);
}

/** The server's per-robot raster of that robot's own component, in its own frame. */
export function robotOptimizedScope(robotId: string): string {
  return `${ROBOT_PREFIX}${robotId}`;
}

/** 0 for a verified component, 1 for the deployment fallback, else unranked. */
function globalRank(entry: OptimizedScope): number | null {
  if (isComponentScope(entry.scope)) return entry.robots.length >= 2 ? 0 : null;
  if (isDeploymentScope(entry.scope)) return entry.robots.length > 0 ? 1 : null;
  return null;
}

/**
 * The scopes the global view may show, best first: verified `component:*`
 * scopes holding two or more robots (more robots, then the larger grid, then
 * the name), and only after every one of them a `deployment:*` composite
 * raster holding two or more robots. The composite is a display placement of
 * single-robot maps by their surveyed start poses, never a verified merge, so
 * a real merged component always wins. Single-robot scopes never qualify: the
 * global view shows a fleet, not one robot.
 */
export function rankGlobalOptimizedScopes(scopes: readonly OptimizedScope[]): OptimizedScope[] {
  return scopes
    .map((entry) => ({ entry, rank: globalRank(entry) }))
    .filter((item): item is { entry: OptimizedScope; rank: number } => item.rank !== null)
    .sort(
      (a, b) =>
        a.rank - b.rank ||
        b.entry.robots.length - a.entry.robots.length ||
        b.entry.width * b.entry.height - a.entry.width * a.entry.height ||
        a.entry.scope.localeCompare(b.entry.scope)
    )
    .map((item) => item.entry);
}

export function selectGlobalOptimizedScope(
  scopes: readonly OptimizedScope[]
): OptimizedScope | undefined {
  return rankGlobalOptimizedScopes(scopes)[0];
}

/**
 * Robots that the fleet map on show does not place: every robot of a
 * single-robot `component:*` scope, except the members of `shownScope` when
 * that is a deployment composite (the raster places them by their surveyed
 * poses, so they are on the map even though nothing merged them). Order is
 * the scope listing's order, each robot once.
 */
export function unmergedRobotIds(
  scopes: readonly OptimizedScope[],
  shownScope: string | null
): string[] {
  const placed = new Set<string>();
  if (shownScope !== null && isDeploymentScope(shownScope)) {
    for (const entry of scopes) {
      if (entry.scope === shownScope) entry.robots.forEach((robot) => placed.add(robot));
    }
  }
  const ids: string[] = [];
  for (const entry of scopes) {
    if (!isComponentScope(entry.scope) || entry.robots.length >= 2) continue;
    for (const robot of entry.robots) {
      if (!placed.has(robot) && !ids.includes(robot)) ids.push(robot);
    }
  }
  return ids;
}

/** How the operator sees a scope: the composite and a robot's own map by their role. */
export function optimizedScopeLabel(scope: string): string {
  if (isDeploymentScope(scope)) return 'deployment composite';
  if (scope.startsWith(ROBOT_PREFIX)) return `${scope.slice(ROBOT_PREFIX.length)} own map`;
  return scope;
}
