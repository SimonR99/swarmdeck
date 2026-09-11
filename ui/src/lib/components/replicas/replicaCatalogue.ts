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
}

export interface ReplicaCatalogue {
  version: 1;
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
    sources: item.sources.map(source)
  };
}

export function parseReplicaCatalogue(value: unknown): ReplicaCatalogue {
  if (!value || typeof value !== 'object') invalid('response is not an object');
  const item = value as Record<string, unknown>;
  if (item.version !== 1 || !Array.isArray(item.components)) invalid('response envelope is invalid');
  return { version: 1, components: item.components.map(entry) };
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
export function catalogueSelection(entry: ReplicaCatalogueEntry): ReplicaTacticalSelection {
  const scope: ReplicaSelectionScope = 'fleet';
  return {
    scope,
    robotId: 'fleet',
    sessionId: entry.session_id,
    componentId: entry.component_id
  };
}

export function catalogueLabel(entry: ReplicaCatalogueEntry): string {
  const robots = entry.robot_ids.join(', ') || 'no robot sources';
  const component = entry.component_id.startsWith('component:')
    ? entry.component_id.slice('component:'.length)
    : entry.component_id;
  const compactComponent = component.length > 20
    ? `${component.slice(0, 12)}…${component.slice(-4)}`
    : component;
  return `${entry.session_id.slice(0, 8)} · ${compactComponent} · ${robots}`;
}
