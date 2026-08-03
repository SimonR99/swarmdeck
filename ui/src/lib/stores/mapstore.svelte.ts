import { inflate } from 'pako';
import type { MapInfo, MapPatch, MapStatus } from '$lib/types/protocol';

/**
 * Occupancy grid, held as an offscreen canvas. The displayed grid is either
 * the verified merged map or one selected robot's native SLAM map.
 * Patches blit in; the full grid is only fetched on connect or reload (NFR-6).
 */

// Slightly darker than the application canvas so explored free space and the
// occupancy-grid boundary remain legible without turning the map into a dark UI.
const UNKNOWN = [229, 232, 236] as const;
const FREE = [255, 255, 255] as const;
const OCCUPIED = [52, 58, 68] as const;

const state = $state({
  info: null as MapInfo | null,
  seq: -1,
  revision: 0, // bumped on every change so views can react
  ready: false,
  status: null as MapStatus | null,
  statusUpdatedAt: 0,
  viewMode: 'global' as 'global' | 'local',
  viewRobot: null as string | null
});

let canvas: HTMLCanvasElement | null = null;
let ctx: CanvasRenderingContext2D | null = null;
let statusLoading = false;
let globalInfo: MapInfo | null = null;
let loadGeneration = 0;

function ensureCanvas(w: number, h: number) {
  if (!canvas) {
    canvas = document.createElement('canvas');
    ctx = canvas.getContext('2d', { willReadFrequently: false });
  }
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
    ctx = canvas.getContext('2d');
    if (ctx) {
      ctx.fillStyle = `rgb(${UNKNOWN.join(',')})`;
      ctx.fillRect(0, 0, w, h);
    }
  }
}

/** Convert an int8 occupancy buffer to RGBA pixels. */
function toImageData(cells: Int8Array, w: number, h: number): ImageData {
  const img = new ImageData(w, h);
  const px = img.data;
  for (let i = 0; i < cells.length; i++) {
    const v = cells[i];
    const c = v < 0 ? UNKNOWN : v >= 50 ? OCCUPIED : FREE;
    const o = i * 4;
    px[o] = c[0];
    px[o + 1] = c[1];
    px[o + 2] = c[2];
    px[o + 3] = 255;
  }
  return img;
}

export const mapStore = {
  get info() {
    return state.info;
  },
  get revision() {
    return state.revision;
  },
  get seq() {
    return state.seq;
  },
  get ready() {
    return state.ready;
  },
  get status() {
    return state.status;
  },
  get statusUpdatedAt() {
    return state.statusUpdatedAt;
  },
  get viewMode() {
    return state.viewMode;
  },
  get viewRobot() {
    return state.viewRobot;
  },
  get viewLabel() {
    return state.viewMode === 'local' && state.viewRobot
      ? `Local map · ${state.viewRobot.replace(/^robot_/, 'R')}`
      : `Global map · ${state.status?.global_members.length ?? 0} robots`;
  },
  get canvas() {
    return canvas;
  },

  setGlobalInfo(info: MapInfo) {
    globalInfo = info;
    if (state.viewMode === 'global') {
      state.info = info;
      ensureCanvas(info.width, info.height);
      state.ready = true;
      state.revision++;
    }
  },

  /** Restore the current full map on first connect or browser reload (FR-M7). */
  async loadFullPng(info: MapInfo) {
    globalInfo = info;
    const generation = loadGeneration;
    try {
      const response = await fetch('/api/map', { cache: 'no-store' });
      if (!response.ok) throw new Error(`map ${response.status}`);
      const seq = Number(response.headers.get('X-Map-Seq') ?? info.seq);
      const bitmap = await createImageBitmap(await response.blob());

      if (generation !== loadGeneration || state.viewMode !== 'global') {
        bitmap.close();
        return;
      }

      // A live patch may have arrived while the PNG was downloading. Never
      // overwrite newer map data with an older reconnect snapshot.
      if (state.seq > seq) {
        bitmap.close();
        return;
      }
      ensureCanvas(info.width, info.height);
      if (ctx) {
        ctx.clearRect(0, 0, info.width, info.height);
        ctx.drawImage(bitmap, 0, 0, info.width, info.height);
      }
      bitmap.close();
      state.info = info;
      state.seq = seq;
      state.ready = true;
      state.revision++;
    } catch (error) {
      // The built-in mock has no HTTP map endpoint; its patches still populate
      // the canvas. A real backend reconnect will retry on the next socket.
      console.warn('[swarmdeck] full map restore failed', error);
    }
  },

  /** Full grid, from GET /api/map or the mock source. */
  setFull(info: MapInfo, cells: Int8Array) {
    globalInfo = info;
    state.viewMode = 'global';
    state.viewRobot = null;
    ensureCanvas(info.width, info.height);
    state.info = info;
    if (ctx) ctx.putImageData(toImageData(cells, info.width, info.height), 0, 0);
    state.seq = info.seq;
    state.ready = true;
    state.revision++;
  },

  /** Incremental patch — the common path. Never re-fetches the whole grid. */
  applyGlobalPatch(patch: MapPatch) {
    if (state.viewMode !== 'global') return;
    if (!state.info) return;
    ensureCanvas(state.info.width, state.info.height);
    if (!ctx) return;

    const raw = Uint8Array.from(atob(patch.data), (c) => c.charCodeAt(0));
    const cells = new Int8Array(inflate(raw).buffer);
    ctx.putImageData(toImageData(cells, patch.w, patch.h), patch.x0, patch.y0);

    state.seq = patch.seq;
    state.revision++;
  },

  /** Lightweight registration health, separate from the high-rate map patches. */
  async refreshStatus() {
    if (statusLoading) return;
    statusLoading = true;
    try {
      const response = await fetch('/api/map/status', { cache: 'no-store' });
      if (!response.ok) throw new Error(`map status ${response.status}`);
      state.status = (await response.json()) as MapStatus;
      state.statusUpdatedAt = Date.now();
    } catch {
      // The opt-in UI mock does not expose HTTP endpoints. Keep the map usable
      // and simply omit registration health in that mode.
    } finally {
      statusLoading = false;
    }
  },

  /** Select the map scope implied by robot selection and registration state. */
  async selectRobotView(robotId: string | null) {
    const desiredLocal = Boolean(
      robotId && state.status?.view_by_robot?.[robotId] === 'local'
    );
    const desiredMode = desiredLocal ? 'local' : 'global';
    const desiredRobot = desiredLocal ? robotId : null;

    if (state.viewMode !== desiredMode || state.viewRobot !== desiredRobot) {
      loadGeneration++;
      state.viewMode = desiredMode;
      state.viewRobot = desiredRobot;
      state.seq = -1;
      state.ready = false;
      state.revision++;
    }

    if (!desiredLocal) {
      if (globalInfo && !state.ready) {
        state.info = globalInfo;
        ensureCanvas(globalInfo.width, globalInfo.height);
        await this.loadFullPng(globalInfo);
      }
      return;
    }

    const generation = loadGeneration;
    try {
      const infoResponse = await fetch(`/api/map/local/${encodeURIComponent(robotId!)}/info`, {
        cache: 'no-store'
      });
      if (!infoResponse.ok) throw new Error(`local map info ${infoResponse.status}`);
      const info = (await infoResponse.json()) as MapInfo;
      const mapResponse = await fetch(`/api/map/local/${encodeURIComponent(robotId!)}`, {
        cache: 'no-store'
      });
      if (!mapResponse.ok) throw new Error(`local map ${mapResponse.status}`);
      const bitmap = await createImageBitmap(await mapResponse.blob());
      if (
        generation !== loadGeneration ||
        state.viewMode !== 'local' ||
        state.viewRobot !== robotId
      ) {
        bitmap.close();
        return;
      }
      ensureCanvas(info.width, info.height);
      ctx?.clearRect(0, 0, info.width, info.height);
      ctx?.drawImage(bitmap, 0, 0, info.width, info.height);
      bitmap.close();
      state.info = info;
      state.seq = info.seq;
      state.ready = true;
      state.revision++;
    } catch (error) {
      console.warn('[swarmdeck] local map restore failed', error);
    }
  },

  /** Coordinates in the currently displayed map frame → pixel. */
  viewToGrid(x: number, y: number): { gx: number; gy: number } | null {
    const i = state.info;
    if (!i) return null;
    return {
      gx: (x - i.origin.x) / i.resolution,
      gy: i.height - (y - i.origin.y) / i.resolution
    };
  },

  /** Pixel → coordinates in the currently displayed map frame. */
  gridToView(gx: number, gy: number): { x: number; y: number } | null {
    const i = state.info;
    if (!i) return null;
    return {
      x: gx * i.resolution + i.origin.x,
      y: (i.height - gy) * i.resolution + i.origin.y
    };
  },

  /** Global world metres → currently displayed grid pixel. */
  worldToGrid(x: number, y: number): { gx: number; gy: number } | null {
    if (state.viewMode === 'local' && state.viewRobot) {
      const tf = state.status?.transforms[state.viewRobot];
      if (tf) {
        const c = Math.cos(tf.yaw);
        const s = Math.sin(tf.yaw);
        const dx = x - tf.x;
        const dy = y - tf.y;
        x = dx * c + dy * s;
        y = -dx * s + dy * c;
      }
    }
    return this.viewToGrid(x, y);
  },

  /** Displayed grid pixel → global world metres (for point navigation). */
  gridToWorld(gx: number, gy: number): { x: number; y: number } | null {
    const point = this.gridToView(gx, gy);
    if (!point) return null;
    if (state.viewMode === 'local' && state.viewRobot) {
      const tf = state.status?.transforms[state.viewRobot];
      if (tf) {
        const c = Math.cos(tf.yaw);
        const s = Math.sin(tf.yaw);
        return {
          x: tf.x + point.x * c - point.y * s,
          y: tf.y + point.x * s + point.y * c
        };
      }
    }
    return point;
  },

  worldYawToView(yaw: number): number {
    if (state.viewMode === 'local' && state.viewRobot) {
      yaw -= state.status?.transforms[state.viewRobot]?.yaw ?? 0;
    }
    return yaw;
  },

  reset() {
    state.info = null;
    state.seq = -1;
    state.ready = false;
    state.status = null;
    state.statusUpdatedAt = 0;
    state.viewMode = 'global';
    state.viewRobot = null;
    globalInfo = null;
    loadGeneration++;
    canvas = null;
    ctx = null;
    state.revision++;
  }
};
