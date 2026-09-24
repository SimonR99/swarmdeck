import type { ReplicaCatalogue } from './replicaCatalogue.ts';

/** The cadence the map's automatic source and the source selector poll at. */
export const REPLICA_CATALOGUE_POLL_MS = 10_000;

export interface ReplicaCatalogueSnapshot {
  /** The last catalogue that parsed; kept through a later failure. */
  catalogue: ReplicaCatalogue | null;
  /**
   * Why the latest refresh failed, for the source selector, else ''. A
   * timeout leaves it as it was, as the selector always did.
   */
  error: string;
  /**
   * The latest completed refresh failed, timeouts included. `catalogue` still
   * holds the last one that parsed; a view that must not show stale
   * membership checks this first.
   */
  refreshFailed: boolean;
  loading: boolean;
}

export interface PollTimers {
  setInterval: (callback: () => void, ms: number) => unknown;
  clearInterval: (handle: unknown) => void;
}

const globalTimers: PollTimers = {
  setInterval: (callback, ms) => globalThis.setInterval(callback, ms),
  clearInterval: (handle) => globalThis.clearInterval(handle as ReturnType<typeof setInterval>)
};

/**
 * One poll of the replica catalogue shared by every view that reads it.
 *
 * The map's automatic source, the source selector and the peer SLAM panel
 * each polled `/api/autonomy/replicas/components` on their own timer. They now
 * subscribe here with the cadence they need: the catalogue is fetched once
 * when a view subscribes (the views used to refresh when they appeared), then
 * at the shortest cadence any current subscriber asked for, and not at all
 * once none is left. A refresh already in flight is never duplicated.
 */
export class ReplicaCataloguePoller {
  private readonly load: (signal: AbortSignal) => Promise<ReplicaCatalogue>;
  private readonly publish: (snapshot: ReplicaCatalogueSnapshot) => void;
  private readonly timers: PollTimers;
  private readonly subscribers = new Map<number, number>();
  private nextSubscriber = 0;
  private timer: unknown = null;
  private timerMs = 0;
  private inFlight: AbortController | null = null;
  private snapshot: ReplicaCatalogueSnapshot = {
    catalogue: null,
    error: '',
    refreshFailed: false,
    loading: false
  };

  constructor(
    load: (signal: AbortSignal) => Promise<ReplicaCatalogue>,
    publish: (snapshot: ReplicaCatalogueSnapshot) => void,
    timers: PollTimers = globalTimers
  ) {
    this.load = load;
    this.publish = publish;
    this.timers = timers;
  }

  get current(): ReplicaCatalogueSnapshot {
    return this.snapshot;
  }

  /** Poll at `intervalMs` or faster until the returned function is called. */
  subscribe(intervalMs = REPLICA_CATALOGUE_POLL_MS): () => void {
    const id = this.nextSubscriber++;
    this.subscribers.set(id, intervalMs);
    this.reschedule();
    void this.refresh();
    return () => {
      if (!this.subscribers.delete(id)) return;
      this.reschedule();
      if (this.subscribers.size === 0) this.inFlight?.abort();
    };
  }

  async refresh(): Promise<void> {
    if (this.inFlight) return;
    const controller = new AbortController();
    this.inFlight = controller;
    this.update({ loading: true });
    try {
      const catalogue = await this.load(controller.signal);
      if (controller.signal.aborted) return;
      this.update({ catalogue, error: '', refreshFailed: false });
    } catch (reason) {
      // Abandoned because no view is left: nothing to report.
      if (controller.signal.aborted) return;
      // The last verified catalogue stays in place through a transient failure.
      if (reason instanceof DOMException && reason.name === 'AbortError') {
        this.update({ refreshFailed: true });
        return;
      }
      this.update({
        error: reason instanceof Error ? reason.message : 'Component catalogue unavailable',
        refreshFailed: true
      });
    } finally {
      if (this.inFlight === controller) this.inFlight = null;
      this.update({ loading: false });
    }
  }

  private update(change: Partial<ReplicaCatalogueSnapshot>) {
    this.snapshot = { ...this.snapshot, ...change };
    this.publish(this.snapshot);
  }

  private reschedule() {
    const intervalMs = this.subscribers.size ? Math.min(...this.subscribers.values()) : 0;
    if (intervalMs === this.timerMs) return;
    if (this.timer !== null) this.timers.clearInterval(this.timer);
    this.timer = null;
    this.timerMs = intervalMs;
    if (intervalMs > 0) this.timer = this.timers.setInterval(() => void this.refresh(), intervalMs);
  }
}
