import {
  fetchLiveReplicaFrame,
  liveReplicaDrawChanged,
  liveReplicaFreshnessDeadline,
  type LiveReplicaFrame,
  type LiveReplicaSelection
} from './liveReplicaFrame.ts';
import { replicaSelectionKey, type ReplicaTacticalSelection } from './replicaTactical.ts';
import { DeadlineWakeup, type WakeTimers } from './renderScheduler.ts';

/** How long one live telemetry request may take before it is abandoned. */
export const LIVE_REPLICA_TIMEOUT_MS = 1500;

export interface LiveReplicaPollHooks {
  /** What the map draws from the live frame changed. */
  onDrawChange(): void;
  /** Something drawn from the live frame went stale on its own clock. */
  onExpire(): void;
  /** Whether a response is still wanted: live mode on and a replica selected. */
  stillWanted(): boolean;
}

export interface LiveReplicaPollDeps {
  fetchFrame: (selection: ReplicaTacticalSelection, signal: AbortSignal) => Promise<LiveReplicaFrame | null>;
  clock: () => number;
  timers?: WakeTimers;
  setTimeout: (run: () => void, ms: number) => unknown;
  clearTimeout: (handle: unknown) => void;
}

const browserDeps: LiveReplicaPollDeps = {
  fetchFrame: fetchLiveReplicaFrame,
  clock: () => performance.now(),
  setTimeout: (run, ms) => globalThis.setTimeout(run, ms),
  clearTimeout: (handle) => globalThis.clearTimeout(handle as ReturnType<typeof setTimeout>)
};

/**
 * The live robot telemetry drawn over a replica in the 3D map, apart from
 * the Svelte view.
 *
 * It holds the latest verified frame for the selected replica, polls for the
 * next one without overlapping requests, drops everything when the selection
 * changes, and wakes the view when a drawn pose, goal or path expires. The
 * frame is not reactive: every poll carries new freshness ages, so the view
 * is told only when what it draws changed (`liveReplicaDrawChanged`).
 */
export class LiveReplicaPoll {
  private live: LiveReplicaSelection | null = null;
  private key = '';
  private pending: AbortController | null = null;
  private readonly hooks: LiveReplicaPollHooks;
  private readonly deps: LiveReplicaPollDeps;
  private readonly wakeup: DeadlineWakeup;

  constructor(hooks: LiveReplicaPollHooks, deps: Partial<LiveReplicaPollDeps> = {}) {
    this.hooks = hooks;
    this.deps = { ...browserDeps, ...deps };
    this.wakeup = new DeadlineWakeup(
      (now) => liveReplicaFreshnessDeadline(this.live, now),
      () => hooks.onExpire(),
      this.deps.clock,
      this.deps.timers
    );
  }

  /** The latest verified frame and when its request started. */
  get current(): LiveReplicaSelection | null {
    return this.live;
  }

  /** Replace the frame, re-arming the expiry wake-up; null forgets it. */
  set(next: LiveReplicaSelection | null) {
    const redraw = liveReplicaDrawChanged(this.live, next, this.deps.clock());
    this.live = next;
    this.wakeup.arm();
    if (redraw) this.hooks.onDrawChange();
  }

  /**
   * Follow the selection whose telemetry is wanted, '' for none. A different
   * selection abandons the request in flight and forgets the frame.
   */
  select(key: string) {
    if (key === this.key) return;
    this.key = key;
    this.pending?.abort();
    this.pending = null;
    this.set(null);
  }

  /** Ask for the selection's next frame unless a request is in flight. */
  async refresh(selection: ReplicaTacticalSelection): Promise<void> {
    if (this.pending) return;
    const key = replicaSelectionKey(selection);
    const controller = new AbortController();
    const startedAt = this.deps.clock();
    const timeout = this.deps.setTimeout(() => controller.abort(), LIVE_REPLICA_TIMEOUT_MS);
    this.pending = controller;
    try {
      const frame = await this.deps.fetchFrame(selection, controller.signal);
      if (key !== this.key || !this.hooks.stillWanted()) return;
      if (frame) this.set({ frame, receivedAt: startedAt });
      // A 404/409 retains the last verified overlay until its three-second
      // freshness budget expires; this avoids blinking during a frame swap.
    } catch {
      // Keep the last coherent live telemetry through a transient response
      // failure; the freshness budget hides it once it is no longer current.
    } finally {
      this.deps.clearTimeout(timeout);
      if (this.pending === controller) this.pending = null;
    }
  }

  dispose() {
    this.pending?.abort();
    this.wakeup.cancel();
  }
}
