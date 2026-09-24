type XY = { x: number; y: number };
type XYZ = { x: number; y: number; z: number };

/** A pointer position in normalised device coordinates, -1..1 each way. */
export type NDC = XY;

/** What pointer handling needs from the 3D scene; Map3D adapts Map3DScene to it. */
export interface InteractionScene {
  yaw: number;
  pitch: number;
  panBy(dx: number, dy: number): void;
  groundAt(ndc: NDC): XYZ | null;
  robotAt(ndc: NDC): string | null;
  detectionAt(ndc: NDC): string | null;
  /** The goal cursor ring that follows the pointer in goal mode. */
  reticle: { visible: boolean; position: { set(x: number, y: number, z: number): unknown } };
}

/** The replica a goal is posted against, as the click saw it. */
export interface LiveGoalContext<S> {
  selection: S;
  /** The solution order the drawn replica was published with. */
  solutionOrder: [number, number] | null | undefined;
  solutionOrderKnown: boolean;
}

/** What the 3D map's pointer handling reads and changes, injected by Map3D. */
export interface SceneInteractionHost<S> {
  scene(): InteractionScene | null;
  requestRender(): void;
  /** The operator took the camera: follow mode ends. */
  cameraMoved(): void;
  setDragging(dragging: boolean): void;
  /** The ground point under the pointer, or null; `planar` goes to the 2D cursor readout. */
  setCursor(point: XYZ | null, planar: XY | null): void;
  goalMode(): boolean;
  /** A replica is shown (live or not); the 2D cursor readout is then off. */
  replicaShown(): boolean;
  /** A live replica is shown; goals can be placed on it. */
  liveReplica(): LiveGoalContext<S> | null;
  /** The robots drawn now. */
  drawnRobotIds(): string[];
  selected(): readonly string[];
  canNavigate(robotId: string): boolean;
  sendGoal(context: LiveGoalContext<S>, robotId: string, goal: XYZ & { yaw: number }): Promise<void>;
  goalFailed(message: string): void;
  cancelGoalMode(): void;
  finishGoal(goal: XY): void;
  selectRobot(robotId: string, additive: boolean): void;
  selectDetection(id: string | null): void;
  detectionSelected(): string | null;
}

/** Selected robots a click on a live replica can send a goal to: drawn and able to navigate. */
export function liveGoalTargets(
  selected: readonly string[],
  drawn: Iterable<string>,
  canNavigate: (robotId: string) => boolean
): string[] {
  const live = new Set(drawn);
  return selected.filter((id) => live.has(id) && canNavigate(id));
}

const DRAG_THRESHOLD_PX = 5;
const PAN_GAIN = 0.8;
const ORBIT_GAIN = 0.007;

/**
 * The 3D map's pointer gestures: left-drag to orbit, right, middle or
 * shift-drag to pan, hover to move the cursor and goal reticle, and a click
 * that sends a goal on a live replica, selects a robot or picks a detection.
 */
export class SceneInteraction<S> {
  private panning = false;
  private pressedAt: XY | null = null;
  private lastAt: XY | null = null;
  private dragged = false;
  private readonly host: SceneInteractionHost<S>;

  constructor(host: SceneInteractionHost<S>) {
    this.host = host;
  }

  pointerDown(at: XY, button: number, shiftKey: boolean) {
    this.pressedAt = at;
    this.lastAt = at;
    this.dragged = false;
    this.host.setDragging(true);
    this.panning = button === 2 || button === 1 || shiftKey;
  }

  pointerMove(at: XY, ndc: NDC) {
    const scene = this.host.scene();
    if (!scene) return;
    if (this.lastAt) {
      const dx = at.x - this.lastAt.x;
      const dy = at.y - this.lastAt.y;
      if (this.pressedAt && Math.hypot(at.x - this.pressedAt.x, at.y - this.pressedAt.y) > DRAG_THRESHOLD_PX) {
        this.dragged = true;
        this.host.cameraMoved();
      }
      if (this.panning) {
        scene.panBy(-dx * PAN_GAIN, -dy * PAN_GAIN);
      } else {
        scene.yaw -= dx * ORBIT_GAIN;
        scene.pitch = Math.max(0.12, Math.min(1.48, scene.pitch + dy * ORBIT_GAIN));
      }
      this.lastAt = at;
      this.host.requestRender();
      return;
    }
    const reticle = scene.reticle;
    const wasVisible = reticle.visible;
    const ground = scene.groundAt(ndc);
    if (ground) {
      this.host.setCursor(ground, this.host.replicaShown() ? null : { x: ground.x, y: ground.y });
      reticle.visible = this.host.goalMode() && this.host.liveReplica() !== null;
      if (reticle.visible) reticle.position.set(ground.x, ground.y, ground.z + 0.015);
    } else {
      this.host.setCursor(null, null);
      reticle.visible = false;
    }
    // Only the reticle follows the pointer; hovering an otherwise still
    // scene is not a reason to redraw it.
    if (reticle.visible || wasVisible) this.host.requestRender();
  }

  /** `cancelled` for a pointercancel, which never clicks; `ndc` is read only for a click. */
  pointerUp(button: number, cancelled: boolean, ndc: () => NDC, additive: boolean): Promise<void> | void {
    if (!this.host.scene()) return;
    this.host.setDragging(false);
    const click = !this.dragged && button === 0 && !cancelled;
    this.lastAt = null;
    this.pressedAt = null;
    this.dragged = false;
    if (click) return this.click(ndc(), additive);
  }

  private async click(ndc: NDC, additive: boolean) {
    const host = this.host;
    const scene = host.scene();
    if (!scene) return;
    const live = host.liveReplica();
    if (host.replicaShown() && !live) return;

    if (host.goalMode()) {
      const hit = scene.groundAt(ndc);
      if (hit) {
        // A goal refused before sending leaves the reticle to the next hover.
        if (live && !(await this.sendGoal(live, hit))) return;
        if (!live) host.cancelGoalMode();
        scene.reticle.visible = false;
        host.requestRender();
        return;
      }
    }

    const robotId = scene.robotAt(ndc);
    if (robotId) {
      host.selectRobot(robotId, additive);
      host.requestRender();
      return;
    }
    const detection = scene.detectionAt(ndc);
    if (detection) {
      host.selectDetection(detection);
      host.requestRender();
      return;
    }
    // Clicking empty space deselects the detection.
    if (host.detectionSelected()) host.selectDetection(null);
  }

  /** False when the goal was refused before anything was sent. */
  private async sendGoal(live: LiveGoalContext<S>, hit: XYZ): Promise<boolean> {
    const host = this.host;
    if (!live.solutionOrderKnown || live.solutionOrder === undefined) {
      host.cancelGoalMode();
      return false;
    }
    const targets = liveGoalTargets(host.selected(), host.drawnRobotIds(), (id) => host.canNavigate(id));
    if (!targets.length) {
      host.cancelGoalMode();
      return false;
    }
    try {
      for (const id of targets) {
        await host.sendGoal(live, id, { x: hit.x, y: hit.y, z: hit.z, yaw: 0 });
      }
      host.finishGoal({ x: hit.x, y: hit.y });
    } catch (reason) {
      host.goalFailed(reason instanceof Error ? reason.message : String(reason));
      host.cancelGoalMode();
    }
    return true;
  }
}
