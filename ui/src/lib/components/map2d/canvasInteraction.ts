import type { CanvasViewport } from './canvasViewport.ts';
import { hasQualifiedRasterFrame, type FrameTransforms } from './mapFrames.ts';
import type { RobotState } from '../../types/protocol.ts';

type XY = { x: number; y: number };

/** A click within this many CSS px of a robot's marker selects it. */
export const ROBOT_PICK_RADIUS_PX = 18;
/** A press that travels further than this is a drag, not a click. */
export const DRAG_THRESHOLD_PX = 6;

/**
 * The selected robots a click on the raster can send a goal to: those that
 * may navigate and whose telemetry frame the displayed raster can place.
 */
export function qualifiedNavigateTargets(
  selected: readonly string[],
  canNavigate: (robotId: string) => boolean,
  robotOf: (robotId: string) => RobotState | undefined,
  frames: FrameTransforms
): string[] {
  return selected.filter((id) => {
    if (!canNavigate(id)) return false;
    const robot = robotOf(id);
    return !robot || hasQualifiedRasterFrame(robot, frames);
  });
}

/** The robot whose marker is nearest a screen point, within the pick radius. */
export function pickRobot(
  robots: readonly { robot_id: string; pose: XY }[],
  click: XY,
  screenOfWorld: (x: number, y: number) => { sx: number; sy: number } | null
): string | null {
  let nearest: { id: string; distance: number } | null = null;
  for (const robot of robots) {
    const screen = screenOfWorld(robot.pose.x, robot.pose.y);
    if (!screen) continue;
    const distance = Math.hypot(screen.sx - click.x, screen.sy - click.y);
    if (distance <= ROBOT_PICK_RADIUS_PX && (!nearest || distance < nearest.distance)) {
      nearest = { id: robot.robot_id, distance };
    }
  }
  return nearest?.id ?? null;
}

export interface ReviewedObjectHit {
  id: string;
  robotId?: string | null;
  robotIds: string[];
}

/** What the 2D canvas's pointer handling reads and changes, injected by MapView. */
export interface CanvasInteractionHost {
  goalMode(): boolean;
  robotsOnMap(): readonly { robot_id: string; pose: XY }[];
  worldToGrid(x: number, y: number): { gx: number; gy: number } | null;
  gridToWorld(gx: number, gy: number): XY | null;
  /** The reviewed object drawn under a canvas point. */
  reviewedObjectAt(x: number, y: number): ReviewedObjectHit | null;
  reviewSelected(): string | null;
  selectReview(id: string | null): void;
  focusRobot(robotId: string): void;
  selectRobot(robotId: string, additive: boolean): void;
  goalTargets(): string[];
  /** The optimized raster scope a goal is posted against, if one is shown. */
  goalScope(): string | null;
  sendGoal(robotId: string, scope: string, world: XY): Promise<void>;
  goalRefused(robotId: string, reason: unknown): void;
  cancelGoalMode(): void;
  finishGoal(world: XY): void;
  /** The operator took the camera: follow mode ends. */
  stopFollowing(): void;
  setCursor(world: XY | null): void;
}

/**
 * The 2D canvas's pointer gestures: hover, drag to pan, two-finger pinch, and
 * a click that picks a reviewed object or a robot, or in goal mode sends the
 * goal. Positions are client px; `origin` reads the canvas's client offset,
 * or null while there is no canvas, and is called only when needed.
 */
export class CanvasInteraction {
  private readonly pointers = new Map<number, XY>();
  private pressedAt: XY | null = null;
  private dragged = false;
  private readonly viewport: CanvasViewport;
  private readonly host: CanvasInteractionHost;

  constructor(viewport: CanvasViewport, host: CanvasInteractionHost) {
    this.viewport = viewport;
    this.host = host;
  }

  pointerDown(pointerId: number, at: XY) {
    this.pointers.set(pointerId, at);
    if (this.pointers.size === 1) {
      this.pressedAt = at;
      this.dragged = false;
    }
  }

  pointerMove(pointerId: number, at: XY, origin: () => XY | null) {
    const prev = this.pointers.get(pointerId);
    if (!prev) {
      const offset = origin();
      if (offset) {
        const grid = this.viewport.gridOf(at.x - offset.x, at.y - offset.y);
        this.host.setCursor(this.host.gridToWorld(grid.gx, grid.gy));
      }
      return;
    }
    const offset = this.pointers.size === 2 ? origin() : null;
    if (offset) {
      const other = [...this.pointers.entries()].find(([id]) => id !== pointerId);
      if (other) {
        this.viewport.pinch(prev, at, other[1], offset);
        this.pointers.set(pointerId, at);
        this.dragged = true;
        this.host.stopFollowing();
        return;
      }
    }
    this.pointers.set(pointerId, at);
    if (this.pressedAt && Math.hypot(at.x - this.pressedAt.x, at.y - this.pressedAt.y) > DRAG_THRESHOLD_PX) {
      this.dragged = true;
      this.host.stopFollowing();
    }
    this.viewport.pan(at.x - prev.x, at.y - prev.y);
  }

  pointerUp(pointerId: number, at: XY, origin: () => XY | null, additive: boolean) {
    const wasDrag = this.dragged;
    this.pointers.delete(pointerId);
    if (this.pointers.size === 0) this.pressedAt = null;
    if (wasDrag) return;
    const offset = origin();
    if (!offset) return;
    this.click({ x: at.x - offset.x, y: at.y - offset.y }, additive);
  }

  private click(click: XY, additive: boolean) {
    const host = this.host;
    // In inspection mode, map markers are directly selectable. Shift-click
    // mirrors the fleet rail's additive selection behaviour.
    if (!host.goalMode()) {
      const detection = host.reviewedObjectAt(click.x, click.y);
      if (detection) {
        if (host.reviewSelected() === detection.id) {
          host.selectReview(null);
        } else {
          host.selectReview(detection.id);
          const robotId = detection.robotId ?? detection.robotIds[0];
          if (robotId) host.focusRobot(robotId);
        }
        return;
      }
      const picked = pickRobot(host.robotsOnMap(), click, (x, y) => {
        const grid = host.worldToGrid(x, y);
        return grid ? this.viewport.screenOf(grid.gx, grid.gy) : null;
      });
      if (picked) host.selectRobot(picked, additive);
      return;
    }

    const grid = this.viewport.gridOf(click.x, click.y);
    const world = host.gridToWorld(grid.gx, grid.gy);
    if (!world) return;
    const targets = host.goalTargets();
    const scope = host.goalScope();
    if (!scope) {
      host.cancelGoalMode();
      return;
    }
    for (const id of targets) {
      void host.sendGoal(id, scope, world).catch((reason) => host.goalRefused(id, reason));
    }
    if (targets.length) host.finishGoal(world);
  }
}
