import { TrailRecorder, type TrailPoint } from './trailRecorder';
import type { Pose } from '../types/protocol';

/**
 * Movement history of the fleet.
 *
 * Held here rather than in the 2D canvas so that both map views draw the same
 * history, and so that it is sampled when telemetry arrives instead of once
 * per displayed frame. The points are in the merged world frame the server
 * publishes poses in; the 2D canvas re-expresses them on the raster it is
 * showing, exactly as it does for the robots themselves.
 */
const recorder = new TrailRecorder();
const state = $state({ revision: 0 });

export const trails = {
  /** Advances whenever a trail gained a point or was retired. */
  get revision() {
    return state.revision;
  },

  record(robotId: string, x: number, y: number, source?: Pose) {
    if (recorder.record(robotId, x, y, source)) state.revision++;
  },

  /**
   * The recorded points. Deliberately not reactive: the trails are read while
   * drawing, which is driven by `revision`, and a reader that subscribed here
   * as well would redraw the whole page for a trail point.
   */
  points(robotId: string): TrailPoint[] {
    return recorder.points(robotId);
  },

  /** The live map of trails. Readers must treat it as read-only. */
  all(): Map<string, TrailPoint[]> {
    return recorder.all();
  },

  clear(robotId?: string) {
    recorder.clear(robotId);
    state.revision++;
  }
};
