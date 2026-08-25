import { inflate } from 'pako';
import type { MapInfo, MapPatch, MapStatus, NetworkPatch, SlamGraph } from '$lib/types/protocol';

/** One entry from GET /api/map/optimized: a grid the collaborative solver posed. */
export interface OptimizedScope {
  /** `robot:<id>` or `component:<n>`. Opaque; the server does not parse it. */
  scope: string;
  robots: string[];
  resolution: number;
  width: number;
  height: number;
  origin: { x: number; y: number };
}

/**
 * Occupancy grid, held as an offscreen canvas. The displayed grid is either
 * the verified merged map or one selected robot's native SLAM map.
 * Patches blit in; the full grid is only fetched on connect or reload (NFR-6).
 */

// Darker than the application canvas so explored free space and the
// occupancy-grid boundary remain legible without turning the map into a dark UI.
// These must stay in step with `_grid_png` in server/swarmdeck_server/mapsvc/
// service.py: the same map arrives as a PNG on connect and as int8 patches
// afterwards, so a mismatch shows up as a seam after a browser reload.
const UNKNOWN = [214, 218, 224] as const;
const FREE = [255, 255, 255] as const;
const OCCUPIED = [52, 58, 68] as const;

// Any pixel darker than this is an occupied cell. Used to recover occupancy from
// the PNG paths, which deliver colour rather than int8 cells. The three palette
// entries above are far apart (red channel 52 / 214 / 255), so the threshold is
// not delicate.
const OCCUPIED_RED_MAX = 140;

const state = $state({
  info: null as MapInfo | null,
  seq: -1,
  revision: 0, // bumped on every change so views can react
  ready: false,
  status: null as MapStatus | null,
  statusUpdatedAt: 0,
  viewMode: 'global' as 'global' | 'local',
  // Orthogonal to viewMode, deliberately. 'slam' is the grid the robot's own
  // SLAM package built and the adapter uploaded; 'optimized' is the same
  // keyframes posed by the collaborative solver. Kept as a separate axis rather
  // than a third viewMode because it answers a different question -- viewMode
  // asks WHOSE map, this asks WHICH ESTIMATE of it -- and folding them together
  // would multiply out into states like "global raw" that do not exist.
  mapSource: 'optimized' as 'slam' | 'optimized',
  optimizedScopes: [] as OptimizedScope[],
  viewRobot: null as string | null,
  // What the OPERATOR asked for, as opposed to what the backend recommends.
  // 'auto' follows the backend's view_by_robot; the other two are a deliberate
  // override. Inspecting one robot's own map is a legitimate thing to want even
  // once that robot has registered — it is how you tell whether a bad merge is
  // the registration or the underlying map — and until this existed, a robot
  // joining the global map silently took the choice away.
  viewPreference: 'auto' as 'auto' | 'global' | 'local',
  // Live pose-graph state, pushed per robot rather than polled with the rest of
  // map status, because an inter-robot loop closure is the event an operator
  // most wants to see the moment it happens.
  slamGraphs: {} as Record<string, SlamGraph>
});

let canvas: HTMLCanvasElement | null = null;
let ctx: CanvasRenderingContext2D | null = null;
let networkCanvas: HTMLCanvasElement | null = null;
let networkCtx: CanvasRenderingContext2D | null = null;
let networkInfo: MapInfo | null = null;
let networkRobot: string | null = null;
let networkSeq = -1;
let statusLoading = false;
let globalInfo: MapInfo | null = null;
let loadGeneration = 0;

/**
 * How long the backend has to keep recommending the other view before `auto`
 * acts on it, in milliseconds.
 *
 * Registration is a live estimate against maps the fleet is still building, so
 * a robot can legitimately enter and leave the merged map several times a
 * minute — most visibly in the minutes after a sim reset, when every transform
 * has to be re-earned against small maps. Following that signal sample by
 * sample made the operator's map unusable: each swap refetches a full grid,
 * clears the trails and resets pan and zoom, so the map lurched between the
 * merged view and one robot's own view every few seconds.
 *
 * A dwell longer than the status poll means a recommendation has to survive
 * several consecutive samples to be believed. Anything flapping faster than
 * this leaves the view exactly where it is, which is the right answer: with
 * membership genuinely in doubt, either view is defensible and neither is worth
 * throwing the operator's zoom away for. An explicit Global/Local choice is
 * never subject to it.
 */
const AUTO_SWITCH_DWELL_MS = 12000;

// The auto decision currently on screen, and the recommendation arguing against
// it. `robot` is part of the latch because selecting a different robot is a new
// question rather than a change of mind about the old one.
let autoView: { robot: string | null; local: boolean } | null = null;
let autoPending: { local: boolean; since: number } | null = null;

/** Resolve `auto` into a concrete local/global choice, with dwell hysteresis. */
function autoWantsLocal(robotId: string | null, recommended: boolean): boolean {
  const now = Date.now();
  if (!autoView || autoView.robot !== robotId) {
    autoView = { robot: robotId, local: recommended };
    autoPending = null;
  } else if (recommended === autoView.local) {
    autoPending = null;
  } else if (!autoPending) {
    autoPending = { local: recommended, since: now };
  } else if (now - autoPending.since >= AUTO_SWITCH_DWELL_MS) {
    autoView = { robot: robotId, local: recommended };
    autoPending = null;
  }
  return autoView.local;
}

/**
 * Occupancy mask pyramid, and why it exists.
 *
 * The grid canvas is one pixel per cell, and `MapView` blits it scaled to fit.
 * Below one screen pixel per cell — which is most of the useful zoom range on a
 * 30 m map — neither `drawImage` mode renders a wall: with smoothing on the
 * browser box-filters, so a one-cell wall is averaged with its free and unknown
 * neighbours until it is a pale smudge; with smoothing off it point-samples, so
 * the wall is dropped outright wherever the sampling grid misses it. That is
 * what made real maps look like dotted fans rather than rooms.
 *
 * So occupancy is composited separately, from a mip chain built by 2x2 *max*
 * rather than by averaging. An isolated occupied cell survives every reduction,
 * and every level is drawn nearest-neighbour at >= 1 px per cell, so a wall
 * keeps full contrast at any zoom. Level cell sizes round up, so a level never
 * covers less ground than the grid it came from.
 */
let maskLevels: HTMLCanvasElement[] = [];
let occupied: Uint8Array | null = null; // 1 byte per cell of the base grid
let maskDirty = true;

const NETWORK_LOW = [210, 48, 115] as const;
const NETWORK_MID = [245, 190, 60] as const;
const NETWORK_HIGH = [31, 158, 137] as const;

function networkColor(quality: number): [number, number, number, number] {
  const q = Math.max(0, Math.min(100, quality));
  const a = q <= 50 ? NETWORK_LOW : NETWORK_MID;
  const b = q <= 50 ? NETWORK_MID : NETWORK_HIGH;
  const t = q <= 50 ? q / 50 : (q - 50) / 50;
  return [
    Math.round(a[0] + (b[0] - a[0]) * t),
    Math.round(a[1] + (b[1] - a[1]) * t),
    Math.round(a[2] + (b[2] - a[2]) * t),
    205
  ];
}

function networkImageData(values: Uint8Array, width: number, height: number): ImageData {
  const image = new ImageData(width, height);
  for (let i = 0; i < values.length; i++) {
    const offset = i * 4;
    if (values[i] === 255) {
      image.data[offset + 3] = 0;
      continue;
    }
    const color = networkColor(values[i]);
    image.data[offset] = color[0];
    image.data[offset + 1] = color[1];
    image.data[offset + 2] = color[2];
    image.data[offset + 3] = color[3];
  }
  return image;
}

function clearNetworkLayer() {
  networkCanvas = null;
  networkCtx = null;
  networkInfo = null;
  networkRobot = null;
  networkSeq = -1;
}

function ensureCanvas(w: number, h: number) {
  if (!canvas) {
    canvas = document.createElement('canvas');
    ctx = canvas.getContext('2d', { willReadFrequently: true });
  }
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
    ctx = canvas.getContext('2d', { willReadFrequently: true });
    if (ctx) {
      ctx.fillStyle = `rgb(${UNKNOWN.join(',')})`;
      ctx.fillRect(0, 0, w, h);
    }
    maskLevels = [];
    occupied = new Uint8Array(w * h);
    maskDirty = true;
  }
}

/** Convert an int8 occupancy buffer to RGBA pixels, recording occupancy. */
function toImageData(
  cells: Int8Array,
  w: number,
  h: number,
  x0 = 0,
  canvas_y0 = 0
): ImageData {
  const img = new ImageData(w, h);
  const px = img.data;
  const stride = canvas?.width ?? w;
  for (let ry = 0; ry < h; ry++) {
    const srcRow = (h - 1 - ry) * w;
    const dstRow = ry * w;
    const canvasY = canvas_y0 + ry;
    for (let rx = 0; rx < w; rx++) {
      const v = cells[srcRow + rx];
      const isOccupied = v >= 50;
      const c = v < 0 ? UNKNOWN : isOccupied ? OCCUPIED : FREE;
      const o = (dstRow + rx) * 4;
      px[o] = c[0];
      px[o + 1] = c[1];
      px[o + 2] = c[2];
      px[o + 3] = 255;
      if (occupied) {
        const gx = x0 + rx;
        occupied[canvasY * stride + gx] = isOccupied ? 1 : 0;
      }
    }
  }
  maskDirty = true;
  return img;
}

/**
 * Recover occupancy by reading the grid canvas back.
 *
 * Only the PNG paths need this — a full map fetched on connect or reload, and a
 * robot's local map — because those deliver colour rather than int8 cells. It is
 * a once-per-load cost, not a per-patch one.
 */
function readBackOccupancy() {
  if (!canvas || !ctx || !occupied) return;
  const { width: w, height: h } = canvas;
  const px = ctx.getImageData(0, 0, w, h).data;
  for (let i = 0; i < occupied.length; i++) {
    occupied[i] = px[i * 4] < OCCUPIED_RED_MAX ? 1 : 0;
  }
  maskDirty = true;
}

function levelCanvas(w: number, h: number, bits: Uint8Array): HTMLCanvasElement {
  const level = document.createElement('canvas');
  level.width = w;
  level.height = h;
  const img = new ImageData(w, h);
  const px = img.data;
  for (let i = 0; i < bits.length; i++) {
    if (!bits[i]) continue;
    const o = i * 4;
    px[o] = OCCUPIED[0];
    px[o + 1] = OCCUPIED[1];
    px[o + 2] = OCCUPIED[2];
    px[o + 3] = 255;
  }
  level.getContext('2d')?.putImageData(img, 0, 0);
  return level;
}

/**
 * Rebuild the pyramid, lazily — only when something has actually changed and a
 * frame is asking for it. Patches arrive at most at the backend's 2 Hz, so a
 * full rebuild (a few hundred thousand integer max operations on a 600x600
 * grid) is cheaper than tracking dirty rectangles through ten levels.
 */
function ensureMask() {
  if (!maskDirty || !canvas || !occupied) return;
  maskDirty = false;
  maskLevels = [];

  let bits = occupied;
  let w = canvas.width;
  let h = canvas.height;
  for (;;) {
    maskLevels.push(levelCanvas(w, h, bits));
    if (w <= 1 && h <= 1) break;
    const nw = Math.max(1, Math.ceil(w / 2));
    const nh = Math.max(1, Math.ceil(h / 2));
    const next = new Uint8Array(nw * nh);
    for (let y = 0; y < nh; y++) {
      const ya = Math.min(y * 2, h - 1) * w;
      const yb = Math.min(y * 2 + 1, h - 1) * w;
      for (let x = 0; x < nw; x++) {
        const xa = Math.min(x * 2, w - 1);
        const xb = Math.min(x * 2 + 1, w - 1);
        next[y * nw + x] = bits[ya + xa] | bits[ya + xb] | bits[yb + xa] | bits[yb + xb];
      }
    }
    bits = next;
    w = nw;
    h = nh;
  }
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
  get slamGraphs() {
    return state.slamGraphs;
  },
  /** True once any robot reports a collaborative pose graph at all. */
  get collaborative() {
    return state.status?.mode === 'cslam' || Object.keys(state.slamGraphs).length > 0;
  },

  applySlamGraph(robotId: string, graph: SlamGraph) {
    state.slamGraphs = { ...state.slamGraphs, [robotId]: graph };
    if (state.mapSource === 'optimized') {
      void this.loadOptimizedScopes().then(() => {
        if (state.viewMode === 'local' && state.viewRobot === robotId) {
          void this.selectRobotView(robotId);
        }
      });
    }
  },
  get mapSource() {
    return state.mapSource;
  },

  /** Switch between the robot's own SLAM grid and the solver-posed one. */
  async setMapSource(source: 'slam' | 'optimized') {
    if (state.mapSource === source) return;
    state.mapSource = source;
    await this.loadOptimizedScopes();
    if (state.viewMode === 'local' && state.viewRobot) {
      state.seq = -1;
      state.ready = false;
      await this.selectRobotView(state.viewRobot);
    }
    state.revision++;
  },

  /** Refresh which optimized grids exist. Cheap: metadata only, no pixels. */
  async loadOptimizedScopes() {
    try {
      const response = await fetch('/api/map/optimized', { cache: 'no-store' });
      if (!response.ok) return;
      const body = (await response.json()) as { maps?: OptimizedScope[] };
      state.optimizedScopes = body.maps ?? [];
      state.revision++;
    } catch (error) {
      // Non-fatal: the toggle simply offers nothing, and the SLAM grid shows.
      console.warn('[swarmdeck] optimized map index failed', error);
    }
  },

  get optimizedScopes() {
    return state.optimizedScopes;
  },

  /** Components holding no other robot: invisible on the merged map by design. */
  get unmergedScopes() {
    return state.optimizedScopes.filter(
      (scope) => scope.scope.startsWith('component:') && scope.robots.length < 2
    );
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
  get networkLayer() {
    // Canvas objects are deliberately module-local; revision makes this getter
    // reactive for the legend while the draw loop reads the same live object.
    void state.revision;
    if (!networkCanvas || !networkInfo || !networkRobot) return null;
    return { canvas: networkCanvas, info: networkInfo, robotId: networkRobot, seq: networkSeq };
  },
  clearNetwork(robotId: string | null = null) {
    if (robotId !== null && networkRobot !== robotId) return;
    clearNetworkLayer();
    state.revision++;
  },

  /**
   * Occupancy mask to composite over the grid at a given screen scale, or null
   * when the grid is already at or above one screen pixel per cell and renders
   * walls correctly on its own.
   *
   * Returns the coarsest level that still has at least one screen pixel per mask
   * cell, so the browser is never asked to downsample occupied cells. Draw it
   * over the exact rectangle the grid was drawn into, with smoothing off.
   */
  occupancyMask(scale: number): HTMLCanvasElement | null {
    if (scale >= 1 || !canvas) return null;
    ensureMask();
    if (!maskLevels.length) return null;
    const level = Math.min(maskLevels.length - 1, Math.ceil(-Math.log2(scale)));
    return maskLevels[Math.max(0, level)];
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
        readBackOccupancy();
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

    const patchWidth = patch.width ?? state.info.width;
    const patchHeight = patch.height ?? state.info.height;
    const patchOriginX = patch.origin.x;
    const patchOriginY = patch.origin.y;

    const dimensionsChanged =
      state.info.origin.x !== patchOriginX ||
      state.info.origin.y !== patchOriginY ||
      state.info.width !== patchWidth ||
      state.info.height !== patchHeight;

    if (dimensionsChanged) {
      const oldInfo = state.info;
      const oldCanvas = document.createElement('canvas');
      oldCanvas.width = oldInfo.width;
      oldCanvas.height = oldInfo.height;
      const oldCtx = oldCanvas.getContext('2d');
      if (canvas && oldCtx) {
        oldCtx.drawImage(canvas, 0, 0);
      }
      const oldOccupied = occupied ? new Uint8Array(occupied) : null;

      state.info = {
        resolution: patch.resolution,
        width: patchWidth,
        height: patchHeight,
        origin: patch.origin,
        seq: patch.seq
      };
      ensureCanvas(patchWidth, patchHeight);

      if (ctx && oldInfo) {
        const offX = Math.round((oldInfo.origin.x - patchOriginX) / patch.resolution);
        const offY = Math.round(
          patchHeight - oldInfo.height - (oldInfo.origin.y - patchOriginY) / patch.resolution
        );
        ctx.drawImage(oldCanvas, offX, offY);

        if (oldOccupied && occupied) {
          for (let y = 0; y < oldInfo.height; y++) {
            const srcRow = y * oldInfo.width;
            const dstRow = (y + offY) * patchWidth + offX;
            if (y + offY >= 0 && y + offY < patchHeight) {
              for (let x = 0; x < oldInfo.width; x++) {
                if (x + offX >= 0 && x + offX < patchWidth) {
                  occupied[dstRow + x] = oldOccupied[srcRow + x];
                }
              }
            }
          }
        }
      }
    } else {
      ensureCanvas(state.info.width, state.info.height);
    }
    if (!ctx) return;

    const raw = Uint8Array.from(atob(patch.data), (c) => c.charCodeAt(0));
    const cells = new Int8Array(inflate(raw).buffer);
    const canvasY0 = patchHeight - patch.y0 - patch.h;
    ctx.putImageData(
      toImageData(cells, patch.w, patch.h, patch.x0, canvasY0),
      patch.x0,
      canvasY0
    );

    state.seq = patch.seq;
    state.revision++;
  },

  /** Incremental per-robot Wi-Fi overlay, retained only for the local map on screen. */
  applyNetworkPatch(patch: NetworkPatch) {
    if (state.viewMode !== 'local' || state.viewRobot !== patch.robot_id) return;
    if (networkRobot === patch.robot_id && patch.seq < networkSeq) return;
    let values: Uint8Array;
    try {
      const compressed = Uint8Array.from(atob(patch.data), (c) => c.charCodeAt(0));
      values = inflate(compressed);
    } catch {
      console.warn('[swarmdeck] ignored malformed network heatmap patch');
      return;
    }
    if (values.length !== patch.w * patch.h) {
      console.warn('[swarmdeck] ignored malformed network heatmap patch');
      return;
    }

    const dimensionsChanged =
      networkRobot !== patch.robot_id ||
      !networkInfo ||
      networkInfo.origin.x !== patch.origin.x ||
      networkInfo.origin.y !== patch.origin.y ||
      networkInfo.resolution !== patch.resolution ||
      networkInfo.width !== patch.width ||
      networkInfo.height !== patch.height;

    if (dimensionsChanged) {
      const oldCanvas = networkCanvas;
      const oldInfo = networkInfo;
      const keepOld = networkRobot === patch.robot_id && oldCanvas && oldInfo;
      networkCanvas = document.createElement('canvas');
      networkCanvas.width = patch.width;
      networkCanvas.height = patch.height;
      networkCtx = networkCanvas.getContext('2d');
      networkCtx?.clearRect(0, 0, patch.width, patch.height);
      if (keepOld && networkCtx) {
        const offX = Math.round((oldInfo.origin.x - patch.origin.x) / patch.resolution);
        const offY = Math.round(
          patch.height - oldInfo.height - (oldInfo.origin.y - patch.origin.y) / patch.resolution
        );
        networkCtx.drawImage(oldCanvas, offX, offY);
      }
    }

    networkInfo = {
      resolution: patch.resolution,
      width: patch.width,
      height: patch.height,
      origin: patch.origin,
      seq: patch.seq
    };
    networkRobot = patch.robot_id;
    networkSeq = patch.seq;
    if (!networkCtx) return;
    networkCtx.putImageData(networkImageData(values, patch.w, patch.h), patch.x0, patch.y0);
    state.revision++;
  },

  async loadNetworkSnapshot(robotId: string, generation = loadGeneration) {
    try {
      const response = await fetch(
        `/api/map/local/${encodeURIComponent(robotId)}/network`,
        { cache: 'no-store' }
      );
      if (!response.ok) {
        if (response.status === 404) return;
        throw new Error(`network heatmap ${response.status}`);
      }
      const patch = (await response.json()) as NetworkPatch;
      if (
        generation !== loadGeneration ||
        state.viewMode !== 'local' ||
        state.viewRobot !== robotId
      ) return;
      this.applyNetworkPatch(patch);
    } catch (error) {
      console.warn('[swarmdeck] network heatmap restore failed', error);
    }
  },

  /** Lightweight registration health, separate from the high-rate map patches. */
  async refreshStatus() {
    if (statusLoading) return;
    statusLoading = true;
    try {
      // Refreshed on the same cadence as status: the set of optimized scopes
      // changes exactly when components do, which is what status reports.
      void this.loadOptimizedScopes();
      const response = await fetch('/api/map/status', { cache: 'no-store' });
      if (!response.ok) throw new Error(`map status ${response.status}`);
      state.status = (await response.json()) as MapStatus;
      state.statusUpdatedAt = Date.now();
      // The websocket carries these live; folding the polled copy in as well
      // means a client that connected mid-session, or missed a push, still
      // shows the graph rather than an empty panel.
      if (state.status.slam_graphs) {
        state.slamGraphs = { ...state.slamGraphs, ...state.status.slam_graphs };
      }
    } catch {
      // The opt-in UI mock does not expose HTTP endpoints. Keep the map usable
      // and simply omit registration health in that mode.
    } finally {
      statusLoading = false;
    }
  },

  /** Reload whichever full grid is visible after an operator map reset. */
  async reloadCurrentView() {
    loadGeneration++;
    state.seq = -1;
    state.ready = false;
    state.revision++;

    if (state.viewMode === 'local' && state.viewRobot) {
      clearNetworkLayer();
      await this.selectRobotView(state.viewRobot);
      return;
    }

    try {
      const response = await fetch('/api/map/info', { cache: 'no-store' });
      if (!response.ok) throw new Error(`map info ${response.status}`);
      const payload = (await response.json()) as { info: MapInfo };
      globalInfo = payload.info;
      await this.loadFullPng(payload.info);
    } catch (error) {
      console.warn('[swarmdeck] map reset reload failed', error);
    }
  },

  /** Select the map scope implied by robot selection and registration state. */
  get viewPreference() {
    return state.viewPreference;
  },

  /** Operator override for local/global. Re-resolves the current selection. */
  async setViewPreference(
    preference: 'auto' | 'global' | 'local',
    robotId: string | null
  ) {
    state.viewPreference = preference;
    // An explicit choice answers the question outright, and switching back to
    // `auto` asks it afresh — neither should wait out a dwell that is there to
    // absorb the backend changing its mind.
    autoView = null;
    autoPending = null;
    await this.selectRobotView(robotId);
  },

  async selectRobotView(robotId: string | null) {
    const recommended = state.status?.view_by_robot?.[robotId ?? ''] === 'local';
    const desiredLocal = Boolean(
      robotId &&
        (state.viewPreference === 'local' ||
          (state.viewPreference === 'auto' && autoWantsLocal(robotId, recommended)))
    );
    const desiredMode = desiredLocal ? 'local' : 'global';
    const desiredRobot = desiredLocal ? robotId : null;

    if (state.viewMode !== desiredMode || state.viewRobot !== desiredRobot) {
      loadGeneration++;
      clearNetworkLayer();
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
      // The optimized endpoint has no per-scope /info: its geometry comes from
      // the index, which the store already refreshes. Falling back to the SLAM
      // grid when a scope is absent matters -- a robot has no optimized map
      // until the solver has placed at least one of its keyframes, so early in
      // a run the raw grid is the only thing there is to show.
      const scopeName = `robot:${robotId!}`;
      const scope =
        state.mapSource === 'optimized'
          ? state.optimizedScopes.find((entry) => entry.scope === scopeName)
          : undefined;

      let info: MapInfo;
      let mapResponse: Response;
      if (scope) {
        info = {
          resolution: scope.resolution,
          width: scope.width,
          height: scope.height,
          origin: scope.origin,
          seq: state.seq + 1
        } as MapInfo;
        mapResponse = await fetch(`/api/map/optimized/${encodeURIComponent(scopeName)}`, {
          cache: 'no-store'
        });
      } else {
        const infoResponse = await fetch(`/api/map/local/${encodeURIComponent(robotId!)}/info`, {
          cache: 'no-store'
        });
        if (!infoResponse.ok) throw new Error(`local map info ${infoResponse.status}`);
        info = (await infoResponse.json()) as MapInfo;
        mapResponse = await fetch(`/api/map/local/${encodeURIComponent(robotId!)}`, {
          cache: 'no-store'
        });
      }
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
      readBackOccupancy();
      bitmap.close();
      state.info = info;
      state.seq = info.seq;
      state.ready = true;
      state.revision++;
      await this.loadNetworkSnapshot(robotId!, generation);
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
    if (state.viewMode === 'local' && state.viewRobot && state.mapSource !== 'optimized') {
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
    if (state.viewMode === 'local' && state.viewRobot && state.mapSource !== 'optimized') {
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
    if (state.viewMode === 'local' && state.viewRobot && state.mapSource !== 'optimized') {
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
    autoView = null;
    autoPending = null;
    globalInfo = null;
    loadGeneration++;
    canvas = null;
    ctx = null;
    clearNetworkLayer();
    maskLevels = [];
    occupied = null;
    maskDirty = true;
    state.revision++;
  }
};
