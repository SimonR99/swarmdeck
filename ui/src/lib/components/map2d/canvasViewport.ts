import type { ScreenPoint } from './mapLayers.ts';

/** The 2D canvas's pan, zoom and rotation, in screen px per raster cell. */
export interface CanvasView {
  scale: number;
  tx: number;
  ty: number;
  rotation: number;
  initialised: boolean;
}

export const MIN_CANVAS_SCALE = 0.12;
export const MAX_CANVAS_SCALE = 6;
const FIT_PADDING_PX = 32;

type ScreenXY = { x: number; y: number };

function clampScale(scale: number): number {
  return Math.max(MIN_CANVAS_SCALE, Math.min(MAX_CANVAS_SCALE, scale));
}

function wrapAngle(angle: number): number {
  while (angle > Math.PI) angle -= Math.PI * 2;
  while (angle < -Math.PI) angle += Math.PI * 2;
  return angle;
}

/**
 * The 2D map's viewport logic, apart from the Svelte view that draws it.
 *
 * It operates on the `view` object it is given. MapView passes its reactive
 * `$state` object, so every change made here schedules a canvas redraw just
 * as the same assignment in the component did.
 */
export class CanvasViewport {
  readonly view: CanvasView;

  constructor(view: CanvasView) {
    this.view = view;
  }

  /** Screen px of a raster cell coordinate. */
  screenOf = (gx: number, gy: number): ScreenPoint => {
    const view = this.view;
    const sxUnrotated = gx * view.scale;
    const syUnrotated = gy * view.scale;
    if (!view.rotation) return { sx: sxUnrotated + view.tx, sy: syUnrotated + view.ty };
    const c = Math.cos(view.rotation);
    const s = Math.sin(view.rotation);
    return {
      sx: sxUnrotated * c - syUnrotated * s + view.tx,
      sy: sxUnrotated * s + syUnrotated * c + view.ty
    };
  };

  /** Raster cell coordinate under a screen px. */
  gridOf = (sx: number, sy: number): { gx: number; gy: number } => {
    const view = this.view;
    const dx = sx - view.tx;
    const dy = sy - view.ty;
    if (!view.rotation) return { gx: dx / view.scale, gy: dy / view.scale };
    const c = Math.cos(-view.rotation);
    const s = Math.sin(-view.rotation);
    return { gx: (dx * c - dy * s) / view.scale, gy: (dx * s + dy * c) / view.scale };
  };

  /** Keep the raster cell under screen point `px, py` there after a change. */
  private pinned(px: number, py: number, change: () => void) {
    const before = this.gridOf(px, py);
    change();
    const after = this.screenOf(before.gx, before.gy);
    this.view.tx += px - after.sx;
    this.view.ty += py - after.sy;
  }

  zoomAt(factor: number, px: number, py: number) {
    this.pinned(px, py, () => (this.view.scale = clampScale(this.view.scale * factor)));
  }

  rotateAt(angleDelta: number, px: number, py: number) {
    this.pinned(px, py, () => (this.view.rotation = wrapAngle(this.view.rotation + angleDelta)));
  }

  pan(dx: number, dy: number) {
    this.view.tx += dx;
    this.view.ty += dy;
  }

  /** Show the whole raster in a host of this size, unrotated scale limits kept. */
  fit(hostWidth: number, hostHeight: number, raster: { width: number; height: number }) {
    const width = Math.max(1, hostWidth - FIT_PADDING_PX * 2);
    const height = Math.max(1, hostHeight - FIT_PADDING_PX * 2);
    this.view.scale = clampScale(Math.min(width / raster.width, height / raster.height));
    this.view.tx = (hostWidth - raster.width * this.view.scale) / 2;
    this.view.ty = (hostHeight - raster.height * this.view.scale) / 2;
  }

  /** Put raster cell `gx, gy` at the centre of a canvas this many CSS px across. */
  centreAt(gx: number, gy: number, cssWidth: number, cssHeight: number) {
    this.view.tx = cssWidth / 2 - gx * this.view.scale;
    this.view.ty = cssHeight / 2 - gy * this.view.scale;
  }

  /**
   * Two-finger zoom and rotate about the fingers' midpoint. `a` is the finger
   * that moved, `b` the one that did not; positions are client px and
   * `origin` is the canvas's client offset.
   */
  pinch(prevA: ScreenXY, curA: ScreenXY, b: ScreenXY, origin: ScreenXY) {
    const prevMid = { x: (prevA.x + b.x) / 2 - origin.x, y: (prevA.y + b.y) / 2 - origin.y };
    const curMid = { x: (curA.x + b.x) / 2 - origin.x, y: (curA.y + b.y) / 2 - origin.y };
    const prevDistance = Math.hypot(prevA.x - b.x, prevA.y - b.y);
    const curDistance = Math.hypot(curA.x - b.x, curA.y - b.y);
    const angleDelta = wrapAngle(
      Math.atan2(b.y - curA.y, b.x - curA.x) - Math.atan2(b.y - prevA.y, b.x - prevA.x)
    );
    const scaleFactor = prevDistance > 5 ? curDistance / prevDistance : 1.0;

    // The raster cell under the old midpoint ends under the new one.
    const anchor = this.gridOf(prevMid.x, prevMid.y);
    this.view.scale = clampScale(this.view.scale * scaleFactor);
    this.view.rotation = wrapAngle(this.view.rotation + angleDelta);
    const after = this.screenOf(anchor.gx, anchor.gy);
    this.view.tx += curMid.x - after.sx;
    this.view.ty += curMid.y - after.sy;
  }
}
