/** One entry from GET /api/map/optimized: a grid the collaborative solver posed. */
export interface OptimizedScope {
  /**
   * `robot:<id>` or `component:<n>` from the SLAM back-end, or
   * `deployment:<session>`, the server's raster of the replicated keyframes
   * placed in the surveyed deployment frame. Opaque to the server routes.
   */
  scope: string;
  robots: string[];
  resolution: number;
  width: number;
  height: number;
  origin: { x: number; y: number };
}

const COMPONENT_PREFIX = 'component:';
const DEPLOYMENT_PREFIX = 'deployment:';

export function isComponentScope(scope: string): boolean {
  return scope.startsWith(COMPONENT_PREFIX);
}

export function isDeploymentScope(scope: string): boolean {
  return scope.startsWith(DEPLOYMENT_PREFIX);
}

/** 0 for a verified component, 1 for the deployment composite, else unranked. */
function globalRank(entry: OptimizedScope): number | null {
  if (entry.robots.length < 2) return null;
  if (isComponentScope(entry.scope)) return 0;
  if (isDeploymentScope(entry.scope)) return 1;
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

/** How the operator sees a scope: the composite by its role, not its session id. */
export function optimizedScopeLabel(scope: string): string {
  return isDeploymentScope(scope) ? 'deployment composite' : scope;
}
