/**
 * Frames per second granted when the only thing animating is decoration: the
 * selection reticle's pulse, a rally beacon's rings, a detection crystal's
 * spin. Nothing in the map has moved, so the scene is redrawn just often
 * enough for those cues to read as alive. Set this to 0 to freeze decoration
 * instead, which stops rendering altogether while the fleet is parked.
 */
export const DECORATION_FPS = 12;

/** How long after a telemetry change the scene is still treated as moving. */
export const MOTION_LINGER_MS = 400;

/**
 * Animation frames arrive a hair early or late. Without this slack a 30 fps
 * budget on a 60 Hz display keeps missing its window by microseconds and
 * settles near 20 fps instead.
 */
const FRAME_SLACK_MS = 1;

export interface RenderRequest {
  /** Timestamp of this animation frame, milliseconds of wall clock. */
  now: number;
  /** The tab is not being composited; nothing is worth drawing. */
  hidden: boolean;
  /** Poses changed, the camera is being driven, or follow mode is tracking. */
  moving: boolean;
  /** Animated markers are on screen. */
  decorating: boolean;
  /** Frame cap of the current graphics quality tier. */
  fps: number;
}

export interface RenderDecision {
  /** Draw this frame. */
  render: boolean;
  /** Ask for another animation frame after this one. */
  again: boolean;
}

/**
 * Decides which animation frames the 3D map actually draws.
 *
 * The map used to render at the quality tier's frame cap for as long as it was
 * on screen, whether or not anything had changed — the dashboard's largest
 * single cost. It now draws when something changed, at the full cap while
 * anything is moving, at DECORATION_FPS when only an animated marker is on
 * screen, and not at all when the fleet is parked or the tab is hidden.
 *
 * Animation phase stays a function of wall time, so a slower cadence changes
 * only how smooth a pulse looks, never how fast it runs.
 */
export class RenderScheduler {
  private dirty = true;
  private lastFrame = Number.NEGATIVE_INFINITY;
  private resumePending = false;

  /** Something the scene is built from changed: draw on the next frame. */
  markDirty() {
    this.dirty = true;
  }

  get pending(): boolean {
    return this.dirty;
  }

  frame(request: RenderRequest): RenderDecision {
    if (request.hidden) {
      // Nothing is composited, and whatever changed while hidden has to be
      // drawn once as soon as the tab comes back.
      this.resumePending = true;
      return { render: false, again: false };
    }

    const decorating = request.decorating && DECORATION_FPS > 0;
    const wanted = this.dirty || request.moving || decorating;

    if (this.resumePending) {
      this.resumePending = false;
      this.dirty = false;
      this.lastFrame = request.now;
      return { render: true, again: request.moving || decorating };
    }

    if (!wanted) return { render: false, again: false };

    // A change or real movement is worth the tier's full frame budget;
    // decoration alone is not.
    const fps = this.dirty || request.moving ? request.fps : DECORATION_FPS;
    if (request.now - this.lastFrame < 1000 / fps - FRAME_SLACK_MS) {
      return { render: false, again: true };
    }

    this.dirty = false;
    this.lastFrame = request.now;
    return { render: true, again: request.moving || decorating };
  }

  reset() {
    this.dirty = true;
    this.lastFrame = Number.NEGATIVE_INFINITY;
    this.resumePending = false;
  }
}

/** The timer functions DeadlineWakeup uses; a test passes its own clock. */
export interface WakeTimers {
  set(run: () => void, delayMs: number): unknown;
  clear(handle: unknown): void;
}

const browserTimers: WakeTimers = {
  set: (run, delayMs) => setTimeout(run, delayMs),
  clear: (handle) => clearTimeout(handle as ReturnType<typeof setTimeout>)
};

/**
 * Wakes the renderer when something drawn expires on its own clock.
 *
 * The map draws only when an input changes, but an item that is drawn only
 * while fresh — a replica robot, goal or path — goes stale without any input
 * changing, and without the poll succeeding again. This keeps exactly one
 * timer, set for the next deadline `nextDeadline(now)` reports; when it fires
 * it calls `wake` and arms itself for the deadline after that.
 */
export class DeadlineWakeup {
  private handle: unknown = null;
  private readonly nextDeadline: (now: number) => number | null;
  private readonly wake: () => void;
  private readonly clock: () => number;
  private readonly timers: WakeTimers;

  constructor(
    nextDeadline: (now: number) => number | null,
    wake: () => void,
    clock: () => number = () => performance.now(),
    timers: WakeTimers = browserTimers
  ) {
    this.nextDeadline = nextDeadline;
    this.wake = wake;
    this.clock = clock;
    this.timers = timers;
  }

  /** Read the next deadline again, replacing any wake-up already pending. */
  arm() {
    this.cancel();
    const now = this.clock();
    const deadline = this.nextDeadline(now);
    if (deadline === null) return;
    this.handle = this.timers.set(() => {
      this.handle = null;
      this.wake();
      this.arm();
    }, Math.max(0, deadline - now));
  }

  cancel() {
    if (this.handle !== null) this.timers.clear(this.handle);
    this.handle = null;
  }
}
