import { inflate } from 'pako';
import type { MapInfo, MapPatch } from '$lib/types/protocol';

/**
 * Merged occupancy grid, held as an offscreen canvas.
 * Patches blit in; the full grid is only fetched on connect or reload (NFR-6).
 */

const UNKNOWN = [26, 34, 48] as const; // matches --color-surface-2
const FREE = [219, 231, 245] as const;
const OCCUPIED = [39, 56, 79] as const;

const state = $state({
  info: null as MapInfo | null,
  seq: -1,
  revision: 0, // bumped on every change so views can react
  ready: false
});

let canvas: HTMLCanvasElement | null = null;
let ctx: CanvasRenderingContext2D | null = null;

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
  get ready() {
    return state.ready;
  },
  get canvas() {
    return canvas;
  },

  setInfo(info: MapInfo) {
    state.info = info;
    ensureCanvas(info.width, info.height);
    state.ready = true;
    state.revision++;
  },

  /** Full grid, from GET /api/map or the mock source. */
  setFull(info: MapInfo, cells: Int8Array) {
    ensureCanvas(info.width, info.height);
    state.info = info;
    if (ctx) ctx.putImageData(toImageData(cells, info.width, info.height), 0, 0);
    state.seq = info.seq;
    state.ready = true;
    state.revision++;
  },

  /** Incremental patch — the common path. Never re-fetches the whole grid. */
  applyPatch(patch: MapPatch) {
    if (!state.info) return;
    ensureCanvas(state.info.width, state.info.height);
    if (!ctx) return;

    const raw = Uint8Array.from(atob(patch.data), (c) => c.charCodeAt(0));
    const cells = new Int8Array(inflate(raw).buffer);
    ctx.putImageData(toImageData(cells, patch.w, patch.h), patch.x0, patch.y0);

    state.seq = patch.seq;
    state.revision++;
  },

  /** World metres → grid pixel. */
  worldToGrid(x: number, y: number): { gx: number; gy: number } | null {
    const i = state.info;
    if (!i) return null;
    return {
      gx: (x - i.origin.x) / i.resolution,
      gy: i.height - (y - i.origin.y) / i.resolution
    };
  },

  /** Grid pixel → world metres. */
  gridToWorld(gx: number, gy: number): { x: number; y: number } | null {
    const i = state.info;
    if (!i) return null;
    return {
      x: gx * i.resolution + i.origin.x,
      y: (i.height - gy) * i.resolution + i.origin.y
    };
  },

  reset() {
    state.info = null;
    state.seq = -1;
    state.ready = false;
    canvas = null;
    ctx = null;
    state.revision++;
  }
};
