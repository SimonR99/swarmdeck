import { LayerUpdateGate } from './layerUpdateGate.ts';
import { MOTION_LINGER_MS, RenderScheduler, type RenderRequest } from './renderScheduler.ts';

/** The animation-frame clock; a test passes its own. */
export interface FrameClock {
  request(callback: (timestamp: number) => void): number;
  cancel(handle: number): void;
  now(): number;
}

const browserFrames: FrameClock = {
  request: (callback) => requestAnimationFrame(callback),
  cancel: (handle) => cancelAnimationFrame(handle),
  now: () => performance.now()
};

/** What the loop asks the 3D view each animation frame. */
export interface RenderLoopHost {
  /** Whether a frame may be requested at all: mounted, shown, tab visible. */
  canStart(): boolean;
  /** This frame's scheduling inputs, or null when there is no scene to draw. */
  frameInputs(): Pick<RenderRequest, 'hidden' | 'decorating' | 'fps'> | null;
  /** Draw one frame; `updateLayers` says the layer rebuild budget allows it. */
  draw(timestamp: number, updateLayers: boolean): void;
}

/**
 * The 3D map's render-on-demand loop, apart from the Svelte view.
 *
 * A change calls `request`, which marks the scene dirty and starts the
 * animation-frame loop if it had stopped. Each frame asks RenderScheduler
 * whether to draw and whether to keep asking, so the loop runs only while
 * something changed, moves or animates; `request(true)` counts as movement
 * for MOTION_LINGER_MS. The layer rebuild budget (LayerUpdateGate) is taken
 * once per drawn frame.
 */
export class RenderLoop {
  private readonly host: RenderLoopHost;
  private readonly clock: FrameClock;
  private readonly scheduler = new RenderScheduler();
  private readonly layerUpdates = new LayerUpdateGate();
  private handle = 0;
  /** Wall clock until which telemetry counts as movement. */
  private movingUntil = 0;

  constructor(host: RenderLoopHost, clock: FrameClock = browserFrames) {
    this.host = host;
    this.clock = clock;
  }

  /** Draw on the next animation frame, starting the loop if it had stopped. */
  request(moving = false) {
    if (moving) this.movingUntil = this.clock.now() + MOTION_LINGER_MS;
    this.scheduler.markDirty();
    if (!this.handle && this.host.canStart()) this.handle = this.clock.request(this.tick);
  }

  /** Something drawn went stale: rebuild the layers on the next frame drawn. */
  invalidateFreshness() {
    this.layerUpdates.invalidateFreshness();
  }

  /** Stop asking for frames; whatever changed meanwhile is drawn on return. */
  pause() {
    this.stop();
    this.scheduler.markDirty();
  }

  stop() {
    if (this.handle) this.clock.cancel(this.handle);
    this.handle = 0;
  }

  private tick = (timestamp: number) => {
    this.handle = 0;
    const inputs = this.host.frameInputs();
    if (!inputs) return;
    const decision = this.scheduler.frame({
      ...inputs,
      now: timestamp,
      moving: timestamp < this.movingUntil
    });
    if (decision.render || decision.again) this.handle = this.clock.request(this.tick);
    if (!decision.render) return;
    this.host.draw(timestamp, this.layerUpdates.take(timestamp));
  };
}
