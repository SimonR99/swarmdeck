import type { TerrainData } from './terrainData.ts';
import {
  acceptsReplicaResult,
  replicaSelectionKey,
  ReplicaRevisionTracker,
  ReplicaTacticalLoader,
  type ReplicaTacticalCloud,
  type ReplicaTacticalSelection
} from './replicaTactical.ts';

/**
 * A composite's revision advances with every member keyframe, which during
 * exploration is about once a second across the fleet. Rebuilding the terrain
 * that often flickers and wastes the worker; geometry-only revisions are
 * picked up on this cadence, while a selection or frame change still rebuilds
 * at once.
 */
export const REPLICA_REVISION_POLL_MS = 5000;
export const REPLICA_CLOUD_TIMEOUT_MS = 20_000;

/** Turns fetched points into terrain, off the main thread. */
export type PrepareCloud = (
  id: number,
  signal: AbortSignal,
  positions: Float32Array,
  owners: Uint8Array,
  rgb?: Uint8Array
) => Promise<TerrainData>;

export type ReplicaCloudResult =
  /** The shown revision is current; any earlier error is over. */
  | { kind: 'current' }
  /** A new terrain to show; `resetView` for a new selection or frame. */
  | { kind: 'built'; data: TerrainData; cloud: ReplicaTacticalCloud; resetView: boolean }
  | { kind: 'failed'; message: string };

export interface ReplicaCloudView {
  /** The replica shown now; a result for another is dropped. */
  selection(): ReplicaTacticalSelection | null;
  hasScene(): boolean;
}

export interface ReplicaCloudDeps {
  loader: Pick<ReplicaTacticalLoader, 'load'>;
  clock: () => number;
  setTimeout: (run: () => void, ms: number) => unknown;
  clearTimeout: (handle: unknown) => void;
}

/**
 * The replica point cloud behind the 3D terrain, apart from the Svelte view.
 *
 * One load at a time fetches the replica's chunks (ReplicaTacticalLoader),
 * skipped when the shown revision is still current, and has the worker turn
 * them into terrain. Every cancellation advances a generation, so a load that
 * finishes after the selection changed, the view paused or the map reset is
 * dropped. The view shows a built terrain and then calls `commit`.
 */
export class ReplicaCloudLoad {
  private generation = 0;
  private pending: AbortController | null = null;
  private needsRebuild = false;
  private lastBuildAt = 0;
  private readonly revision = new ReplicaRevisionTracker();
  private readonly view: ReplicaCloudView;
  private readonly prepare: PrepareCloud;
  private readonly deps: ReplicaCloudDeps;

  constructor(view: ReplicaCloudView, prepare: PrepareCloud, deps: Partial<ReplicaCloudDeps> = {}) {
    this.view = view;
    this.prepare = prepare;
    this.deps = {
      loader: new ReplicaTacticalLoader(),
      clock: () => performance.now(),
      setTimeout: (run, ms) => globalThis.setTimeout(run, ms),
      clearTimeout: (handle) => globalThis.clearTimeout(handle as ReturnType<typeof setTimeout>),
      ...deps
    };
  }

  get busy(): boolean {
    return this.pending !== null;
  }

  /** Whether the 1 s poll should load: a rebuild is owed or the revision cadence passed. */
  due(): boolean {
    return this.needsRebuild || this.deps.clock() - this.lastBuildAt >= REPLICA_REVISION_POLL_MS;
  }

  /** Drop the load in flight and any result still to come. */
  cancel() {
    this.generation++;
    this.abort();
  }

  /** Abandon the request in flight; a result already on its way is still shown. */
  abort() {
    this.pending?.abort();
    this.pending = null;
  }

  /** Rebuild the terrain whole on the next load, even at the same revision. */
  requireRebuild() {
    this.needsRebuild = true;
  }

  /** Forget the shown revision: a new selection, or the map it came from was reset. */
  forgetRevision(rebuild: boolean) {
    this.revision.clear();
    this.needsRebuild = rebuild;
  }

  /** Load the selected replica; null when there is nothing to show. */
  async load(): Promise<ReplicaCloudResult | null> {
    const shown = this.view.selection();
    if (this.pending || !shown) return null;
    const controller = new AbortController();
    this.pending = controller;
    const timeout = this.deps.setTimeout(() => controller.abort(), REPLICA_CLOUD_TIMEOUT_MS);
    const id = ++this.generation;
    const selection = { ...shown };
    const accepted = (source: string) =>
      acceptsReplicaResult(source, this.view.selection(), id, this.generation);
    try {
      const cloud = await this.deps.loader.load(
        selection,
        controller.signal,
        this.needsRebuild ? null : this.revision.current
      );
      if (!cloud) return accepted(replicaSelectionKey(selection)) ? { kind: 'current' } : null;
      if (!this.view.hasScene() || !accepted(cloud.sourceKey)) return null;
      const transition = this.revision.transition(cloud);
      const data = await this.prepare(id, controller.signal, cloud.positions, cloud.owners, cloud.rgb);
      if (!this.view.hasScene() || !accepted(cloud.sourceKey)) return null;
      return { kind: 'built', data, cloud, resetView: transition === 'selection' || transition === 'frame' };
    } catch (cause) {
      if (controller.signal.aborted || id !== this.generation) return null;
      return { kind: 'failed', message: cause instanceof Error ? cause.message : String(cause) };
    } finally {
      this.deps.clearTimeout(timeout);
      if (this.pending === controller) this.pending = null;
    }
  }

  /** The built terrain is on show. */
  commit(cloud: ReplicaTacticalCloud) {
    this.revision.commit(cloud);
    this.needsRebuild = false;
    this.lastBuildAt = this.deps.clock();
  }
}
