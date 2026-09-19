import type {
  ReplicaSelectionScope,
  ReplicaTacticalSelection
} from '../map3d/replicaTactical.ts';

export type ReplicaCatalogueStatus = 'ready' | 'syncing' | 'conflict';

export interface ReplicaCatalogueSource {
  robot_id: string;
  session_id: string;
  revision: number;
  snapshot_id: string;
}

export interface ReplicaCatalogueEntry {
  session_id: string;
  component_id: string;
  frame_id: string;
  robot_ids: string[];
  source_count: number;
  submap_count: number;
  point_count: number;
  available: boolean;
  status: ReplicaCatalogueStatus;
  detail: string;
  solution_order: [number, number] | null;
  sources: ReplicaCatalogueSource[];
  /**
   * The server-composed `deployment:<session>` entry: every replicated
   * single-robot component placed in the surveyed deployment frame. A display
   * and goal-entry composition, never a verified merge, so it ranks below any
   * real component and does not count as merged membership.
   */
  composite: boolean;
}

const DEPLOYMENT_COMPOSITE_PREFIX = 'deployment:';

export function isDeploymentComposite(componentId: string | null | undefined): boolean {
  return typeof componentId === 'string' && componentId.startsWith(DEPLOYMENT_COMPOSITE_PREFIX);
}

export interface ReplicaCatalogue {
  version: 1;
  /** Server-declared current mission; absent means the server cannot select safely. */
  active_session_id: string | null;
  components: ReplicaCatalogueEntry[];
}

function invalid(message: string): never {
  throw new Error(`Invalid replica catalogue: ${message}`);
}

function nonnegative(value: unknown, field: string): number {
  if (!Number.isSafeInteger(value) || (value as number) < 0) invalid(`${field} is invalid`);
  return value as number;
}

function text(value: unknown, field: string): string {
  if (typeof value !== 'string' || value.length === 0) invalid(`${field} is invalid`);
  return value;
}

function source(value: unknown): ReplicaCatalogueSource {
  if (!value || typeof value !== 'object') invalid('source is invalid');
  const item = value as Record<string, unknown>;
  return {
    robot_id: text(item.robot_id, 'source robot_id'),
    session_id: text(item.session_id, 'source session_id'),
    revision: nonnegative(item.revision, 'source revision'),
    snapshot_id: text(item.snapshot_id, 'source snapshot_id')
  };
}

function entry(value: unknown): ReplicaCatalogueEntry {
  if (!value || typeof value !== 'object') invalid('component is invalid');
  const item = value as Record<string, unknown>;
  const status = item.status;
  if (status !== 'ready' && status !== 'syncing' && status !== 'conflict') {
    invalid('component status is invalid');
  }
  if (!Array.isArray(item.robot_ids) || item.robot_ids.some((id) => typeof id !== 'string')) {
    invalid('component robot_ids are invalid');
  }
  if (!Array.isArray(item.sources)) invalid('component sources are invalid');
  if (typeof item.available !== 'boolean') invalid('component available is invalid');
  const solutionOrder = item.solution_order;
  if (
    solutionOrder !== null &&
    (!Array.isArray(solutionOrder) || solutionOrder.length !== 2 ||
      !Number.isSafeInteger(solutionOrder[0]) || !Number.isSafeInteger(solutionOrder[1]))
  ) invalid('component solution_order is invalid');
  if (item.composite !== undefined && typeof item.composite !== 'boolean') {
    invalid('component composite is invalid');
  }
  return {
    session_id: text(item.session_id, 'session_id'),
    component_id: text(item.component_id, 'component_id'),
    frame_id: text(item.frame_id, 'frame_id'),
    robot_ids: [...item.robot_ids] as string[],
    source_count: nonnegative(item.source_count, 'source_count'),
    submap_count: nonnegative(item.submap_count, 'submap_count'),
    point_count: nonnegative(item.point_count, 'point_count'),
    available: item.available,
    status,
    detail: typeof item.detail === 'string' ? item.detail : '',
    solution_order: solutionOrder as [number, number] | null,
    sources: item.sources.map(source),
    composite: item.composite === true
  };
}

export function parseReplicaCatalogue(value: unknown): ReplicaCatalogue {
  if (!value || typeof value !== 'object') invalid('response is not an object');
  const item = value as Record<string, unknown>;
  if (item.version !== 1 || !Array.isArray(item.components)) invalid('response envelope is invalid');
  const activeSession = item.active_session_id;
  if (activeSession !== undefined && activeSession !== null && typeof activeSession !== 'string') {
    invalid('active_session_id is invalid');
  }
  return {
    version: 1,
    active_session_id: activeSession === undefined ? null : activeSession,
    components: item.components.map(entry)
  };
}

/**
 * Pick a current component only when the server identified the mission. A
 * preferred robot narrows the choice to a component that actually contains
 * that robot; ties are deterministic and never combine disconnected frames.
 * A verified component always outranks the deployment composite, which the
 * Global view falls back to only while no verified multi-robot component
 * exists; a local (single robot) view never uses the composite.
 */
export function automaticCatalogueEntry(
  catalogue: ReplicaCatalogue,
  preferredRobotId?: string | null,
  fallbackToSingle = false,
  minimumRobotCount = 1,
  allowComposite = true
): ReplicaCatalogueEntry | null {
  if (!catalogue.active_session_id) return null;
  let candidates = catalogue.components
    .filter((item) => item.available && item.status === 'ready' && item.session_id === catalogue.active_session_id &&
      item.robot_ids.length >= minimumRobotCount && (allowComposite || !item.composite))
  if (preferredRobotId) {
    const matching = candidates.filter((item) => item.robot_ids.includes(preferredRobotId));
    if (matching.length) candidates = matching;
    else if (!fallbackToSingle || candidates.length !== 1) return null;
  }
  if (!candidates.length) return null;
  return [...candidates].sort((a, b) =>
    Number(a.composite) - Number(b.composite) ||
    b.robot_ids.length - a.robot_ids.length ||
    a.component_id.localeCompare(b.component_id)
  )[0];
}

/**
 * Members of the active component selected by the Global view's rules. The
 * deployment composite is not a merge, so it never counts as membership.
 */
export function activeMergedRobotIds(
  catalogue: ReplicaCatalogue,
  preferredRobotId?: string | null,
  explicitSelection?: ReplicaTacticalSelection | null
): string[] | null {
  if (!catalogue.active_session_id) return null;
  let candidates = catalogue.components
    .filter((entry) =>
      entry.session_id === catalogue.active_session_id &&
      entry.available &&
      entry.status === 'ready' &&
      !entry.composite
    )
    .map((entry) => ({
      componentId: entry.component_id,
      robotIds: [...new Set(entry.robot_ids)].sort()
    }))
    .filter((entry) => entry.robotIds.length >= 2);
  if (explicitSelection?.scope === 'fleet' &&
      explicitSelection.sessionId === catalogue.active_session_id) {
    const selected = candidates.find(
      (entry) => entry.componentId === explicitSelection.componentId
    );
    return selected?.robotIds ?? [];
  }
  if (preferredRobotId) {
    const matching = candidates.filter((entry) => entry.robotIds.includes(preferredRobotId));
    if (matching.length) candidates = matching;
    else if (candidates.length !== 1) return [];
  }
  candidates.sort((a, b) =>
    b.robotIds.length - a.robotIds.length || a.componentId.localeCompare(b.componentId)
  );
  return candidates[0]?.robotIds ?? [];
}

/**
 * Keep an already rendered automatic view during a catalogue transition only
 * when it still has the same mission and the same scope/robot ownership.
 * Readiness is intentionally excluded: a transient syncing/conflict status
 * must not relabel fleet geometry as a local view (or vice versa).
 */
export function automaticSelectionIsCoherent(
  selection: ReplicaTacticalSelection | null | undefined,
  entry: ReplicaCatalogueEntry | null | undefined,
  activeSessionId: string | null,
  local: boolean,
  preferredRobotId?: string | null
): boolean {
  if (!selection || !entry || !activeSessionId || entry.session_id !== activeSessionId) return false;
  if (local) {
    return selection.scope === 'robot' &&
      Boolean(preferredRobotId) &&
      selection.robotId === preferredRobotId &&
      entry.robot_ids.includes(preferredRobotId as string);
  }
  return selection.scope === 'fleet' && selection.robotId === 'fleet' && entry.robot_ids.length >= 2;
}

export async function fetchReplicaCatalogue(
  sessionId?: string,
  signal?: AbortSignal
): Promise<ReplicaCatalogue> {
  const query = sessionId ? `?session_id=${encodeURIComponent(sessionId)}` : '';
  const requestController = new AbortController();
  const timeout = globalThis.setTimeout(() => requestController.abort(), 5_000);
  const abort = () => requestController.abort();
  if (signal?.aborted) requestController.abort();
  else signal?.addEventListener('abort', abort, { once: true });
  try {
    const response = await fetch(`/api/autonomy/replicas/components${query}`, {
      cache: 'no-store',
      signal: requestController.signal
    });
    if (!response.ok) throw new Error(`Replica catalogue unavailable (${response.status})`);
    return parseReplicaCatalogue(await response.json());
  } finally {
    globalThis.clearTimeout(timeout);
    signal?.removeEventListener('abort', abort);
  }
}

/** Catalogue entries always use the aggregate read-only view. */
export function catalogueSelection(
  entry: ReplicaCatalogueEntry,
  scope: ReplicaSelectionScope = 'fleet',
  robotId = 'fleet'
): ReplicaTacticalSelection {
  return {
    scope,
    robotId,
    sessionId: entry.session_id,
    componentId: entry.component_id
  };
}

export function catalogueLabel(entry: ReplicaCatalogueEntry): string {
  const robots = entry.robot_ids.join(', ') || 'no robot sources';
  if (entry.composite) {
    return `${entry.session_id.slice(0, 8)} · deployment composite · ${robots}`;
  }
  const component = entry.component_id.startsWith('component:')
    ? entry.component_id.slice('component:'.length)
    : entry.component_id;
  const compactComponent = component.length > 20
    ? `${component.slice(0, 12)}…${component.slice(-4)}`
    : component;
  return `${entry.session_id.slice(0, 8)} · ${compactComponent} · ${robots}`;
}
