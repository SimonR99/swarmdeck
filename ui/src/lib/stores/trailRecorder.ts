/** Where a robot has been, in the merged world frame its telemetry arrives in. */
export interface TrailPoint {
  x: number;
  y: number;
}

/** Points kept per robot. At 8 cm spacing this is about 50 m of history. */
export const TRAIL_MAX_POINTS = 600;
/** Shortest move that is worth another point. */
export const TRAIL_MIN_STEP_M = 0.08;
/**
 * A jump this large is not driving: it is a relocalisation, a map reset or a
 * fresh mission, and the history before it describes somewhere else.
 */
export const TRAIL_RESET_JUMP_M = 3.0;

/**
 * Movement history per robot.
 *
 * Recorded as telemetry arrives rather than while drawing: the 2D canvas was
 * the only recorder, so it sampled the fleet at display rate while it was on
 * screen and recorded nothing at all while the 3D view was, which is why 3D
 * trails were empty or stale.
 */
export class TrailRecorder {
  private trails = new Map<string, TrailPoint[]>();

  /** Returns true when the recorded history changed. */
  record(robotId: string, x: number, y: number): boolean {
    if (!Number.isFinite(x) || !Number.isFinite(y)) return false;
    let trail = this.trails.get(robotId);
    if (!trail) {
      trail = [];
      this.trails.set(robotId, trail);
    }
    const last = trail[trail.length - 1];
    if (!last) {
      trail.push({ x, y });
      return true;
    }
    const distance = Math.hypot(last.x - x, last.y - y);
    if (distance > TRAIL_RESET_JUMP_M) {
      trail.length = 0;
      trail.push({ x, y });
      return true;
    }
    if (distance <= TRAIL_MIN_STEP_M) return false;
    trail.push({ x, y });
    if (trail.length > TRAIL_MAX_POINTS) trail.shift();
    return true;
  }

  points(robotId: string): TrailPoint[] {
    return this.trails.get(robotId) ?? [];
  }

  all(): Map<string, TrailPoint[]> {
    return this.trails;
  }

  clear(robotId?: string) {
    if (robotId === undefined) this.trails.clear();
    else this.trails.delete(robotId);
  }
}
