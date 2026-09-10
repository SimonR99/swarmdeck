import {
  MAX_CACHE_BYTES,
  ReplicaChunkCache,
  parseXYZF32,
  samplePreviewChunks,
  selectPreviewRefs,
  type ChunkRef,
  type PreviewChunk
} from '../replicas/replicaPreview.ts';

export const MAX_TACTICAL_SOURCE_POINTS = 300_000;
const MAX_DOWNLOADS = 3;

export interface ReplicaTacticalSelection {
  robotId: string;
  sessionId: string;
  componentId: string;
}

export interface ReplicaSubmap {
  submap_id: string;
  geometry_revision?: number;
  pose_revision?: unknown;
  T_component_submap: number[][];
  chunks: ChunkRef[];
}

export interface ReplicaComponent {
  component_id: string;
  frame_id: string;
  graph_revision?: { component_id?: string; epoch?: number; revision?: number } | null;
  geometry_revision?: unknown;
  submaps: ReplicaSubmap[];
}

export interface ReplicaView {
  robot_id: string;
  session_id: string;
  revision: number;
  snapshot_id?: string;
  component_id: string | null;
  components: ReplicaComponent[];
  selected: ReplicaComponent | null;
  chunks: ChunkRef[];
  source_age_s: number | null;
  reconstruction?: { state?: string; artifact?: string } | null;
}

export interface ReplicaTacticalCloud {
  sourceKey: string;
  frameKey: string;
  revisionKey: string;
  positions: Float32Array;
  owners: Uint8Array;
  ownerIds: string[];
  partial: boolean;
  view: ReplicaView;
}

export type ReplicaTransition = 'selection' | 'frame' | 'revision' | 'unchanged';
export type ReplicaDisplayRevision = Pick<ReplicaTacticalCloud, 'sourceKey' | 'frameKey' | 'revisionKey'>;

/** Track only a fully rendered revision; failed preparation leaves the prior display current. */
export class ReplicaRevisionTracker {
  private displayed: ReplicaDisplayRevision | null = null;

  get current(): ReplicaDisplayRevision | null {
    return this.displayed;
  }

  transition(next: ReplicaDisplayRevision): ReplicaTransition {
    return replicaTransition(this.displayed, next);
  }

  commit(next: ReplicaDisplayRevision): void {
    this.displayed = {
      sourceKey: next.sourceKey,
      frameKey: next.frameKey,
      revisionKey: next.revisionKey
    };
  }

  clear(): void {
    this.displayed = null;
  }
}

export function replicaSelectionKey(selection: ReplicaTacticalSelection): string {
  return `${selection.robotId}\u0000${selection.sessionId}\u0000${selection.componentId}`;
}

export function replicaTransition(
  previous: ReplicaDisplayRevision | null,
  next: ReplicaDisplayRevision
): ReplicaTransition {
  if (!previous || previous.sourceKey !== next.sourceKey) return 'selection';
  if (previous.frameKey !== next.frameKey) return 'frame';
  return previous.revisionKey === next.revisionKey ? 'unchanged' : 'revision';
}

export function acceptsReplicaResult(
  requestedSource: string,
  currentSelection: ReplicaTacticalSelection | null,
  requestGeneration: number,
  currentGeneration: number
): boolean {
  return requestGeneration === currentGeneration
    && currentSelection !== null
    && requestedSource === replicaSelectionKey(currentSelection);
}

function validateTransform(T: number[][]) {
  if (T.length !== 4 || T.some((row) => row.length !== 4 || row.some((value) => !Number.isFinite(value)))) {
    throw new Error('Invalid replica submap transform');
  }
  if (Math.abs(T[3][0]) > 1e-6 || Math.abs(T[3][1]) > 1e-6 ||
      Math.abs(T[3][2]) > 1e-6 || Math.abs(T[3][3] - 1) > 1e-6) {
    throw new Error('Invalid replica submap transform');
  }
}

function transformPoint(target: Float32Array, offset: number, point: Float32Array, index: number, T: number[][]) {
  const x = point[index], y = point[index + 1], z = point[index + 2];
  target[offset] = T[0][0] * x + T[0][1] * y + T[0][2] * z + T[0][3];
  target[offset + 1] = T[1][0] * x + T[1][1] * y + T[1][2] * z + T[1][3];
  target[offset + 2] = T[2][0] * x + T[2][1] * y + T[2][2] * z + T[2][3];
}

function ownerOf(submapId: string, fallback: string): string {
  // Server-normalised IDs carry their publisher as the first segment. This is
  // used only for colour attribution; T_component_submap defines the frame.
  const owner = submapId.split('/')[0];
  return owner || fallback;
}

function frameKey(view: ReplicaView, selected: ReplicaComponent): string {
  const revision = selected.graph_revision;
  return JSON.stringify([
    replicaSelectionKey({
      robotId: view.robot_id,
      sessionId: view.session_id,
      componentId: selected.component_id
    }),
    selected.frame_id,
    revision?.component_id ?? selected.component_id,
    revision?.epoch ?? null
  ]);
}

function revisionKey(view: ReplicaView, selected: ReplicaComponent): string {
  return JSON.stringify([
    view.revision,
    view.snapshot_id ?? null,
    selected.geometry_revision ?? null,
    selected.graph_revision?.revision ?? null,
    selected.submaps.map((submap) => [
      submap.submap_id,
      submap.geometry_revision ?? null,
      submap.pose_revision ?? null,
      submap.chunks.map((chunk) => chunk.sha256)
    ])
  ]);
}

export class ReplicaTacticalLoader {
  private readonly cache: ReplicaChunkCache;
  private readonly maxSourcePoints: number;
  private readonly maxBytes: number;

  constructor(maxBytes = MAX_CACHE_BYTES, maxSourcePoints = MAX_TACTICAL_SOURCE_POINTS) {
    this.cache = new ReplicaChunkCache(maxBytes);
    this.maxBytes = maxBytes;
    this.maxSourcePoints = maxSourcePoints;
  }

  private async fetchChunk(ref: ChunkRef, signal: AbortSignal): Promise<Float32Array> {
    if (!/^[a-f0-9]{64}$/.test(ref.sha256)) throw new Error('Invalid replica chunk digest');
    const cached = this.cache.get(ref.sha256);
    if (cached) return cached;
    const response = await fetch(`/api/autonomy/chunks/${encodeURIComponent(ref.sha256)}`, {
      cache: 'force-cache', signal
    });
    if (!response.ok) throw new Error(`Replica chunk unavailable (${response.status})`);
    const advertised = Number(response.headers.get('content-length') ?? 0);
    if (advertised > this.maxBytes) throw new Error('Replica chunk exceeds the browser safety limit');
    const points = parseXYZF32(new Uint8Array(await response.arrayBuffer()));
    if (ref.size_bytes !== undefined && points.byteLength + 16 !== ref.size_bytes) {
      throw new Error('Replica chunk does not match its declared size');
    }
    if (!this.cache.set(ref.sha256, points)) throw new Error('Replica chunk exceeds the tactical cache');
    return points;
  }

  async load(
    selection: ReplicaTacticalSelection,
    signal: AbortSignal,
    known: ReplicaDisplayRevision | null = null
  ): Promise<ReplicaTacticalCloud | null> {
    const sourceKey = replicaSelectionKey(selection);
    const response = await fetch(
      `/api/autonomy/replicas/view/${encodeURIComponent(selection.robotId)}/${encodeURIComponent(selection.sessionId)}`
        + `?component_id=${encodeURIComponent(selection.componentId)}`,
      { cache: 'no-store', signal }
    );
    if (!response.ok) throw new Error(`Replica view unavailable (${response.status})`);
    const view = await response.json() as ReplicaView;
    const selected = view.selected;
    if (!selected || view.component_id !== selection.componentId ||
        selected.component_id !== selection.componentId ||
        view.robot_id !== selection.robotId || view.session_id !== selection.sessionId) {
      throw new Error('Replica view does not match the selected component');
    }
    if (!Array.isArray(selected.submaps) || !Array.isArray(view.chunks)) {
      throw new Error('Invalid replica component view');
    }
    for (const submap of selected.submaps) {
      if (!Array.isArray(submap.chunks)) throw new Error('Invalid replica submap');
      validateTransform(submap.T_component_submap);
    }
    const metadata = {
      sourceKey,
      frameKey: frameKey(view, selected),
      revisionKey: revisionKey(view, selected)
    };
    if (known && replicaTransition(known, metadata) === 'unchanged') return null;

    const refs = selectPreviewRefs(view.chunks, this.maxBytes);
    const retained = new Set(refs.map((ref) => ref.sha256));
    this.cache.retain(retained);
    let cursor = 0;
    const workers = Array.from({ length: Math.min(MAX_DOWNLOADS, refs.length) }, async () => {
      while (cursor < refs.length) await this.fetchChunk(refs[cursor++], signal);
    });
    await Promise.all(workers);

    const chunks: (PreviewChunk & { transform: number[][]; ownerId: string })[] = [];
    for (const submap of selected.submaps) {
      for (const ref of submap.chunks) {
        if (!retained.has(ref.sha256)) continue;
        const points = this.cache.get(ref.sha256);
        if (points) chunks.push({
          submapId: submap.submap_id,
          sha256: ref.sha256,
          points,
          transform: submap.T_component_submap,
          ownerId: ownerOf(submap.submap_id, selection.robotId)
        });
      }
    }
    const sourcePointCount = chunks.reduce((sum, chunk) => sum + chunk.points.length / 3, 0);
    const sampled = samplePreviewChunks(chunks, this.maxSourcePoints) as typeof chunks;
    const count = sampled.reduce((sum, chunk) => sum + chunk.points.length / 3, 0);
    const positions = new Float32Array(count * 3);
    const owners = new Uint8Array(count);
    const ownerIds: string[] = [];
    const ownerIndices = new Map<string, number>();
    let outputPoint = 0;
    for (const chunk of sampled) {
      let owner = ownerIndices.get(chunk.ownerId);
      if (owner === undefined) {
        owner = ownerIds.length;
        if (owner > 255) throw new Error('Replica has too many geometry owners');
        ownerIndices.set(chunk.ownerId, owner);
        ownerIds.push(chunk.ownerId);
      }
      for (let point = 0; point < chunk.points.length; point += 3) {
        transformPoint(positions, outputPoint * 3, chunk.points, point, chunk.transform);
        owners[outputPoint++] = owner;
      }
    }
    return {
      ...metadata,
      positions,
      owners,
      ownerIds,
      partial: retained.size < new Set(view.chunks.map((ref) => ref.sha256)).size
        || sourcePointCount > this.maxSourcePoints,
      view
    };
  }
}
