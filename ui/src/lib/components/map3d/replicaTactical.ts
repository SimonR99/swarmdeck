import {
  ReplicaChunkCache,
  parseXYZF32,
  parseXYZRGBAF32U8,
  samplePreviewChunks,
  selectPreviewRefs,
  type ChunkRef,
  type PreviewChunk
} from '../replicas/replicaPreview.ts';

/**
 * The live map is the fleet's whole replicated geometry, not a preview: a
 * four-robot Bistro composite is about 4 M points in 80 MB of chunks after a
 * three-minute exploration (benchbot 2026-09-19). Under the preview budgets
 * (64 MB, 300,000 points) `selectPreviewRefs` dropped whole keyframes and the
 * sampler kept one point in thirteen, so the 3D view showed a thin subset of
 * the same map the 2D raster drew in full. These budgets hold the whole
 * composite of a session like that; a dedicated GPU renders them at frame
 * rate, integrated graphics at a lower one.
 */
export const MAX_TACTICAL_CACHE_BYTES = 256 * 1024 * 1024;
export const MAX_TACTICAL_SOURCE_POINTS = 6_000_000;
const MAX_DOWNLOADS = 3;

export type ReplicaSelectionScope = 'robot' | 'fleet';

export interface ReplicaTacticalSelection {
  robotId: string;
  sessionId: string;
  componentId: string;
  /** Legacy inspector selections default to one robot; the catalogue uses fleet. */
  scope?: ReplicaSelectionScope;
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
  /** Aggregate fleet views have no authoritative per-robot revision. */
  revision: number | null;
  snapshot_id?: string;
  component_id: string | null;
  components: ReplicaComponent[];
  selected: ReplicaComponent | null;
  chunks: ChunkRef[];
  source_age_s: number | null;
  solution_order?: [number, number] | null;
  solution_order_known: boolean;
  scope?: ReplicaSelectionScope;
  sources?: ReplicaViewSource[];
  reconstruction?: { state?: string; artifact?: string } | null;
}

export interface ReplicaViewSource {
  robot_id: string;
  session_id: string;
  revision: number;
  snapshot_id: string;
}

export interface ReplicaTacticalCloud {
  sourceKey: string;
  frameKey: string;
  revisionKey: string;
  positions: Float32Array;
  owners: Uint8Array;
  rgb?: Uint8Array;
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
  return `${selection.scope ?? 'robot'}\u0000${selection.robotId}\u0000${selection.sessionId}\u0000${selection.componentId}`;
}

export function replicaSelectionScope(selection: ReplicaTacticalSelection): ReplicaSelectionScope {
  return selection.scope ?? 'robot';
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

function validateSolutionOrder(value: unknown) {
  if (value === undefined || value === null) return;
  if (!Array.isArray(value) || value.length !== 2 ||
      !Number.isSafeInteger(value[0]) || !Number.isSafeInteger(value[1]) ||
      value[0] < 0 || value[1] < -1 || (value[1] === -1 && value[0] !== 0)) {
    throw new Error('Invalid replica solution order');
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

function frameKey(
  selection: ReplicaTacticalSelection,
  selected: ReplicaComponent
): string {
  if (replicaSelectionScope(selection) === 'fleet') {
    return JSON.stringify([
      'fleet',
      selection.sessionId,
      selected.component_id,
      selected.frame_id
    ]);
  }
  const graphRevision = selected.graph_revision;
  return JSON.stringify([
    replicaSelectionKey(selection),
    selected.frame_id,
    graphRevision?.component_id ?? selected.component_id,
    graphRevision?.epoch ?? null
  ]);
}

function revisionKey(
  view: ReplicaView,
  selected: ReplicaComponent,
  visibleSubmaps = selected.submaps
): string {
  return JSON.stringify([
    view.revision,
    view.snapshot_id ?? null,
    view.solution_order_known,
    view.solution_order ?? null,
    selected.geometry_revision ?? null,
    selected.graph_revision?.revision ?? null,
    visibleSubmaps.map((submap) => [
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
  private readonly colors = new Map<string, Uint8Array>();

  constructor(maxBytes = MAX_TACTICAL_CACHE_BYTES, maxSourcePoints = MAX_TACTICAL_SOURCE_POINTS) {
    this.cache = new ReplicaChunkCache(maxBytes);
    this.maxBytes = maxBytes;
    this.maxSourcePoints = maxSourcePoints;
  }

  private async fetchChunk(ref: ChunkRef, signal: AbortSignal): Promise<Float32Array> {
    if (!/^[a-f0-9]{64}$/.test(ref.sha256)) throw new Error('Invalid replica chunk digest');
    const colored = ref.encoding === 'application/vnd.swarmdeck.xyzrgba-f32-u8.v1';
    if (!colored && ref.encoding !== 'application/vnd.swarmdeck.xyz-f32.v1') {
      throw new Error('Unsupported replica chunk encoding');
    }
    const cached = this.cache.get(ref.sha256);
    if (cached) return cached;
    const response = await fetch(`/api/autonomy/chunks/${encodeURIComponent(ref.sha256)}`, {
      cache: 'force-cache', signal
    });
    if (!response.ok) throw new Error(`Replica chunk unavailable (${response.status})`);
    const advertised = Number(response.headers.get('content-length') ?? 0);
    if (advertised > this.maxBytes) throw new Error('Replica chunk exceeds the browser safety limit');
    const bytes = new Uint8Array(await response.arrayBuffer());
    const decoded = colored ? parseXYZRGBAF32U8(bytes) : { points: parseXYZF32(bytes), rgba: undefined };
    const points = decoded.points;
    if (ref.size_bytes !== undefined && bytes.byteLength !== ref.size_bytes) {
      throw new Error('Replica chunk does not match its declared size');
    }
    if (ref.point_count !== undefined && points.length / 3 !== ref.point_count) {
      throw new Error('Replica chunk does not match its declared point count');
    }
    if (!this.cache.set(ref.sha256, points)) throw new Error('Replica chunk exceeds the tactical cache');
    if (decoded.rgba) this.colors.set(ref.sha256, decoded.rgba);
    return points;
  }

  async load(
    selection: ReplicaTacticalSelection,
    signal: AbortSignal,
    known: ReplicaDisplayRevision | null = null
  ): Promise<ReplicaTacticalCloud | null> {
    const sourceKey = replicaSelectionKey(selection);
    const scope = replicaSelectionScope(selection);
    const endpoint = scope === 'fleet'
      ? `/api/autonomy/replicas/components/view/${encodeURIComponent(selection.sessionId)}`
        + `?component_id=${encodeURIComponent(selection.componentId)}`
      : `/api/autonomy/replicas/view/${encodeURIComponent(selection.robotId)}/${encodeURIComponent(selection.sessionId)}`
        + `?component_id=${encodeURIComponent(selection.componentId)}`;
    const response = await fetch(endpoint, { cache: 'no-store', signal });
    if (!response.ok) throw new Error(`Replica view unavailable (${response.status})`);
    const view = await response.json() as ReplicaView;
    const selected = view.selected;
    if (!selected || view.component_id !== selection.componentId ||
        selected.component_id !== selection.componentId ||
        view.robot_id !== (scope === 'fleet' ? 'fleet' : selection.robotId) ||
        view.session_id !== selection.sessionId ||
        (view.scope ?? 'robot') !== scope) {
      throw new Error('Replica view does not match the selected component');
    }
    if (!Array.isArray(selected.submaps) || !Array.isArray(view.chunks)) {
      throw new Error('Invalid replica component view');
    }
    if (typeof view.solution_order_known !== 'boolean') {
      throw new Error('Invalid replica solution order provenance');
    }
    validateSolutionOrder(view.solution_order);
    if (view.solution_order_known && view.solution_order === undefined) {
      throw new Error('Invalid replica solution order provenance');
    }
    for (const submap of selected.submaps) {
      if (!Array.isArray(submap.chunks)) throw new Error('Invalid replica submap');
      validateTransform(submap.T_component_submap);
    }
    const visibleSubmaps = scope === 'robot'
      ? selected.submaps.filter((submap) => ownerOf(submap.submap_id, '') === selection.robotId)
      : selected.submaps;
    const metadata = {
      sourceKey,
      frameKey: frameKey(selection, selected),
      revisionKey: revisionKey(view, selected, visibleSubmaps)
    };
    if (known && replicaTransition(known, metadata) === 'unchanged') return null;

    const visibleChunks = visibleSubmaps.flatMap((submap) => submap.chunks);
    const refs = selectPreviewRefs(visibleChunks, this.maxBytes);
    const retained = new Set(refs.map((ref) => ref.sha256));
    this.cache.retain(retained);
    for (const key of this.colors.keys()) if (!retained.has(key)) this.colors.delete(key);
    let cursor = 0;
    const workers = Array.from({ length: Math.min(MAX_DOWNLOADS, refs.length) }, async () => {
      while (cursor < refs.length) await this.fetchChunk(refs[cursor++], signal);
    });
    await Promise.all(workers);

    const chunks: (PreviewChunk & { transform: number[][]; ownerId: string })[] = [];
    for (const submap of visibleSubmaps) {
      for (const ref of submap.chunks) {
        if (!retained.has(ref.sha256)) continue;
        const points = this.cache.get(ref.sha256);
        if (points) chunks.push({
          submapId: submap.submap_id,
          sha256: ref.sha256,
          points,
          rgba: this.colors.get(ref.sha256),
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
    const hasColor = sampled.some((chunk) => Boolean(chunk.rgba));
    const rgb = hasColor ? new Uint8Array(count * 3) : undefined;
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
        if (rgb) {
          const rgba = chunk.rgba;
          const source = point / 3;
          const target = outputPoint - 1;
          if (rgba && rgba[source * 4 + 3]) rgb.set(rgba.subarray(source * 4, source * 4 + 3), target * 3);
          else rgb.set([148, 148, 148], target * 3);
        }
      }
    }
    return {
      ...metadata,
      positions,
      owners,
      rgb,
      ownerIds,
      partial: retained.size < new Set(visibleChunks.map((ref) => ref.sha256)).size
        || sourcePointCount > this.maxSourcePoints,
      view
    };
  }
}
