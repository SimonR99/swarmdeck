import { inflate } from 'pako';
import {
  isComponentScope,
  optimizedScopeLabel,
  selectGlobalOptimizedScope,
  robotOptimizedScope,
  unmergedRobotIds,
  type OptimizedScope
} from './optimizedScopes';
import { fleet } from '$lib/stores/fleet.svelte';
import { keepIfUnchanged } from './sameFieldValue';
import type { NetworkPatch, MapStatus, Pose, SlamGraph } from '$lib/types/protocol';

export type { OptimizedScope } from './optimizedScopes';

export interface RasterInfo {
  resolution: number;
  width: number;
  height: number;
  origin: { x: number; y: number };
  seq: number;
  /** Only the transform provenance carried by the displayed raster response. */
  transforms?: Record<string, Pose>;
}

const UNKNOWN = [214, 218, 224] as const;
const FREE = [255, 255, 255] as const;
const OCCUPIED = [52, 58, 68] as const;
const OCCUPIED_RED_MAX = 140;
const NETWORK_LOW = [210, 48, 115] as const;
const NETWORK_MID = [245, 190, 60] as const;
const NETWORK_HIGH = [31, 158, 137] as const;

const state = $state({
  info: null as RasterInfo | null,
  seq: 0,
  revision: 0,
  ready: false,
  status: null as MapStatus | null,
  statusUpdatedAt: 0,
  viewMode: 'global' as 'global' | 'local',
  viewRobot: null as string | null,
  viewPreference: 'auto' as 'auto' | 'global' | 'local',
  optimizedScopes: [] as OptimizedScope[],
  globalOptimizedScope: null as string | null,
  robotMapEpochs: {} as Record<string, string>,
  slamGraphs: {} as Record<string, SlamGraph>
});

let canvas: HTMLCanvasElement | null = null;
let ctx: CanvasRenderingContext2D | null = null;
let loadGeneration = 0;
let statusLoading = false;
let globalRefreshInFlight = false;

export interface NetworkLayerEntry {
  canvas: HTMLCanvasElement;
  ctx: CanvasRenderingContext2D;
  info: RasterInfo;
  robotId: string;
  seq: number;
}
const networkLayers = new Map<string, NetworkLayerEntry>();


function ensureCanvas(width: number, height: number) {
  if (!canvas) canvas = document.createElement('canvas');
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  ctx = canvas.getContext('2d');
}

function clearGrid() {
  if (canvas && ctx) ctx.clearRect(0, 0, canvas.width, canvas.height);
  state.info = null;
  state.ready = false;
  state.globalOptimizedScope = null;
}

function readBackOccupancy() {
  // The map renderer uses the raster directly; this hook is intentionally kept
  // small so browser implementations can recover occupancy masks if desired.
}

function parseRasterInfo(headers: Headers, fallback: RasterInfo): RasterInfo {
  const numberHeader = (name: string, value: number) => {
    const raw = headers.get(name);
    if (raw === null) return value;
    const parsed = Number(raw);
    if (!Number.isFinite(parsed)) throw new Error(`Invalid ${name}`);
    return parsed;
  };
  const transformsHeader = headers.get('X-Map-Transforms');
  let transforms = fallback.transforms;
  if (transformsHeader !== null) {
    const parsed = JSON.parse(transformsHeader) as unknown;
    if (!parsed || typeof parsed !== 'object' || Array.isArray(parsed)) throw new Error('Invalid X-Map-Transforms');
    transforms = parsed as Record<string, Pose>;
  }
  return {
    resolution: numberHeader('X-Map-Resolution', fallback.resolution),
    width: numberHeader('X-Map-Width', fallback.width),
    height: numberHeader('X-Map-Height', fallback.height),
    origin: {
      x: numberHeader('X-Map-Origin-X', fallback.origin.x),
      y: numberHeader('X-Map-Origin-Y', fallback.origin.y)
    },
    seq: numberHeader('X-Map-Seq', fallback.seq),
    transforms
  };
}

function networkColor(quality: number): [number, number, number, number] {
  if (quality >= 255) return [0, 0, 0, 0];
  const q = Math.max(0, Math.min(100, quality));
  if (q < 50) {
    const t = q / 50;
    return [
      Math.round(NETWORK_LOW[0] + (NETWORK_MID[0] - NETWORK_LOW[0]) * t),
      Math.round(NETWORK_LOW[1] + (NETWORK_MID[1] - NETWORK_LOW[1]) * t),
      Math.round(NETWORK_LOW[2] + (NETWORK_MID[2] - NETWORK_LOW[2]) * t),
      150
    ];
  }
  const t = (q - 50) / 50;
  return [
    Math.round(NETWORK_MID[0] + (NETWORK_HIGH[0] - NETWORK_MID[0]) * t),
    Math.round(NETWORK_MID[1] + (NETWORK_HIGH[1] - NETWORK_MID[1]) * t),
    Math.round(NETWORK_MID[2] + (NETWORK_HIGH[2] - NETWORK_MID[2]) * t),
    150
  ];
}

function networkImageData(values: Uint8Array, width: number, height: number): ImageData {
  const image = new ImageData(width, height);
  for (let i = 0; i < values.length; i++) image.data.set(networkColor(values[i]), i * 4);
  return image;
}


function clearNetworkLayer(robotId: string | null = null) {
  if (robotId === null) networkLayers.clear();
  else networkLayers.delete(robotId);
}

/**
 * The raster on show: a robot's own map (`robot:<id>`, its component in its
 * own frame) in local mode, the fleet map otherwise. A local view never falls
 * back to a fleet raster: showing the global product under a local label is
 * worse than waiting for the selected robot's own publication.
 */
function shownScope(): OptimizedScope | undefined {
  if (state.viewMode === 'local' && state.viewRobot) {
    return state.optimizedScopes.find(
      (scope) => scope.scope === robotOptimizedScope(state.viewRobot!)
    );
  }
  return selectGlobalOptimizedScope(state.optimizedScopes);
}

export const mapStore = {
  get info() { return state.info; },
  get revision() { return state.revision; },
  get seq() { return state.seq; },
  get ready() { return state.ready; },
  get status() { return state.status; },
  get statusUpdatedAt() { return state.statusUpdatedAt; },
  get slamGraphs() { return state.slamGraphs; },
  get collaborative() { return Object.keys(state.slamGraphs).length > 0; },
  get viewMode() { return state.viewMode; },
  get viewRobot() { return state.viewRobot; },
  get viewPreference() { return state.viewPreference; },
  get showingOptimizedGrid() { return true; },
  get optimizedScopes() { return state.optimizedScopes; },
  get globalOptimizedScope() { return state.globalOptimizedScope; },
  get globalOptimizedRobots(): readonly string[] | undefined {
    const scope = state.globalOptimizedScope;
    return scope ? state.optimizedScopes.find((entry) => entry.scope === scope)?.robots : undefined;
  },
  get globalOptimizedLabel() {
    return state.globalOptimizedScope ? optimizedScopeLabel(state.globalOptimizedScope) : null;
  },
  get unmergedScopes() {
    return state.optimizedScopes.filter((entry) => isComponentScope(entry.scope) && entry.robots.length < 2);
  },
  get unmergedRobots() {
    return unmergedRobotIds(state.optimizedScopes, state.globalOptimizedScope).filter((id) => fleet.isEnabled(id));
  },
  get viewLabel() {
    const shown = state.optimizedScopes.find((entry) => entry.scope === state.globalOptimizedScope);
    if (state.viewMode === 'local' && state.viewRobot) {
      const own = shown?.scope === robotOptimizedScope(state.viewRobot);
      return `${state.viewRobot.replace(/^robot_/, 'R')} · ${own ? 'own map' : 'own map pending'}`;
    }
    return `Deployment map · ${shown?.robots.length ?? 0} robots`;
  },
  get canvas() { return canvas; },
  get occupied() { return null as Uint8Array | null; },
  get networkLayers() { void state.revision; return Array.from(networkLayers.values()); },
  get networkLayer() {
    void state.revision;
    const id = state.viewMode === 'local' && state.viewRobot
      ? state.viewRobot : fleet.selected[0] ?? fleet.robots[0]?.robot_id;
    return id ? networkLayers.get(id) ?? null : null;
  },
  clearNetwork(robotId: string | null = null) { clearNetworkLayer(robotId); state.revision++; },
  get robotMapEpochs() { return state.robotMapEpochs; },

  applySlamGraph(robotId: string, graph: SlamGraph) {
    state.slamGraphs = { ...state.slamGraphs, [robotId]: graph };
    state.revision++;
  },

  applyRobotMapReset(robotId: string, missionId: string, epoch: number) {
    const identity = `${missionId}:${epoch}`;
    if (state.robotMapEpochs[robotId] === identity) return;
    state.robotMapEpochs = { ...state.robotMapEpochs, [robotId]: identity };
    clearNetworkLayer(robotId);
    delete state.slamGraphs[robotId];
    state.optimizedScopes = state.optimizedScopes.filter((entry) => !entry.robots.includes(robotId));
    if (state.viewMode === 'global' || state.viewRobot === robotId) void this.reloadCurrentView();
    state.revision++;
  },


  applyNetworkPatch(patch: NetworkPatch) {
    let values: Uint8Array;
    try {
      const encoded = Uint8Array.from(atob(patch.data), (c) => c.charCodeAt(0));
      values = inflate(encoded);
      if (values.length !== patch.w * patch.h) throw new Error('size mismatch');
    } catch {
      console.warn('[swarmdeck] ignored malformed network heatmap patch');
      return;
    }
    let entry = networkLayers.get(patch.robot_id);
    if (entry && patch.seq < entry.seq) return;
    if (!entry || entry.info.width !== patch.width || entry.info.height !== patch.height) {
      const layerCanvas = document.createElement('canvas');
      layerCanvas.width = patch.width; layerCanvas.height = patch.height;
      const layerCtx = layerCanvas.getContext('2d');
      if (!layerCtx) return;
      entry = { canvas: layerCanvas, ctx: layerCtx,
        info: { resolution: patch.resolution, width: patch.width, height: patch.height, origin: patch.origin, seq: patch.seq },
        robotId: patch.robot_id, seq: patch.seq };
      networkLayers.set(patch.robot_id, entry);
    }
    entry.info.resolution = patch.resolution;
    entry.info.origin = patch.origin;
    entry.info.seq = patch.seq;
    entry.seq = patch.seq;
    entry.ctx.putImageData(networkImageData(values, patch.w, patch.h), patch.x0, patch.y0);
    state.revision++;
  },

  /**
   * One pass of the merged map poll. Each part is asked for by the scheduler
   * in `mapPollScheduler.ts`, which owns the cadences.
   */
  async poll(work: { scopes: boolean; status: boolean; raster: boolean }) {
    if (work.scopes || work.raster) await this.loadOptimizedScopes();
    if (work.status) await this.loadStatus();
    if (work.raster) await this.refreshGlobalOptimizedView({ indexLoaded: true });
  },

  async refreshStatus() {
    await this.loadOptimizedScopes();
    await this.loadStatus();
  },

  async loadStatus() {
    if (statusLoading) return;
    statusLoading = true;
    try {
      const response = await fetch('/api/map/status', { cache: 'no-store' });
      if (!response.ok) throw new Error(`map status ${response.status}`);
      const status = (await response.json()) as MapStatus;
      // A status that says what the last one said keeps its reference, so the
      // panels and the map layers built from it are left alone.
      state.status = keepIfUnchanged(state.status, status);
      state.statusUpdatedAt = Date.now();
      if (status.slam_graphs) {
        state.slamGraphs = keepIfUnchanged(state.slamGraphs, {
          ...state.slamGraphs,
          ...status.slam_graphs
        });
      }
      if (!state.ready) await this.loadGlobalOptimized();
    } catch {
      // The explicit mock profile has no HTTP map status endpoint.
    } finally {
      statusLoading = false;
    }
  },

  async loadOptimizedScopes() {
    try {
      const response = await fetch('/api/map/optimized', { cache: 'no-store' });
      if (!response.ok) return;
      const body = (await response.json()) as { maps?: OptimizedScope[] };
      // Keep per-robot rasters in the catalogue: local mode resolves the
      // selected robot to `robot:<id>`. Global ranking deliberately ignores
      // these scopes, so retaining them cannot contaminate fleet selection.
      const maps = keepIfUnchanged(state.optimizedScopes, body.maps ?? []);
      if (maps === state.optimizedScopes) return;
      state.optimizedScopes = maps;
      state.revision++;
    } catch (error) {
      console.warn('[swarmdeck] optimized map index failed', error);
    }
  },

  async loadGlobalOptimized(): Promise<boolean> {
    const scope = shownScope();
    if (!scope || state.viewMode !== 'global' && state.viewMode !== 'local') return false;
    const generation = loadGeneration;
    try {
      const response = await fetch(`/api/map/optimized/${encodeURIComponent(scope.scope)}`, { cache: 'no-store' });
      if (!response.ok) throw new Error(`optimized map ${response.status}`);
      const fallback: RasterInfo = { ...scope, seq: scope.seq ?? state.seq };
      const info = parseRasterInfo(response.headers, fallback);
      const bitmap = await createImageBitmap(await response.blob());
      if (generation !== loadGeneration || scope.scope !== shownScope()?.scope) { bitmap.close(); return false; }
      ensureCanvas(info.width, info.height);
      ctx?.clearRect(0, 0, info.width, info.height);
      ctx?.drawImage(bitmap, 0, 0, info.width, info.height);
      readBackOccupancy();
      bitmap.close();
      state.info = info;
      state.seq = info.seq;
      state.globalOptimizedScope = scope.scope;
      state.ready = true;
      state.revision++;
      return true;
    } catch (error) {
      console.warn('[swarmdeck] optimized map restore failed', error);
      return false;
    }
  },

  async refreshGlobalOptimizedView(options?: { indexLoaded?: boolean }) {
    if (globalRefreshInFlight) return;
    globalRefreshInFlight = true;
    try {
      const previous = state.globalOptimizedScope;
      if (!options?.indexLoaded) await this.loadOptimizedScopes();
      const next = shownScope();
      if (!next) {
        if (state.ready) {
          clearGrid();
          state.revision++;
        }
        return;
      }
      const seqChanged = next.seq !== undefined && next.seq !== state.seq;
      if (next.scope !== previous || seqChanged || !state.ready) await this.loadGlobalOptimized();
    } finally {
      globalRefreshInFlight = false;
    }
  },

  async refreshLocalView() {
    // Local shows the robot's own raster; the same refresh path selects it.
    if (state.viewMode === 'local') await this.refreshGlobalOptimizedView();
  },

  async reloadCurrentView() {
    loadGeneration++;
    clearGrid();
    state.revision++;
    await this.loadOptimizedScopes();
    await this.loadGlobalOptimized();
  },

  async setViewPreference(preference: 'auto' | 'global' | 'local', robotId: string | null) {
    state.viewPreference = preference;
    state.viewMode = preference === 'local' || (preference === 'auto' && robotId !== null) ? 'local' : 'global';
    state.viewRobot = state.viewMode === 'local' ? robotId : null;
    loadGeneration++;
    clearGrid();
    state.revision++;
    await this.loadOptimizedScopes();
    await this.loadGlobalOptimized();
  },
  async selectRobotView(robotId: string | null, force = false) {
    const mode = robotId && state.viewPreference === 'local' ? 'local' : 'global';
    const nextRobot = mode === 'local' ? robotId : null;
    const changed = state.viewMode !== mode || state.viewRobot !== nextRobot;
    state.viewMode = mode;
    state.viewRobot = nextRobot;
    if (changed || force || !state.ready) {
      loadGeneration++;
      clearGrid();
      await this.loadOptimizedScopes();
      await this.loadGlobalOptimized();
    }
  },

  viewToGrid(x: number, y: number) {
    const info = state.info;
    return info ? { gx: (x - info.origin.x) / info.resolution, gy: info.height - (y - info.origin.y) / info.resolution } : null;
  },
  gridToView(gx: number, gy: number) {
    const info = state.info;
    return info ? { x: gx * info.resolution + info.origin.x, y: (info.height - gy) * info.resolution + info.origin.y } : null;
  },
  occupancyMask(_scale: number) { return null as HTMLCanvasElement | null; },
  overlayUsesRobotSlamFrame() { return false; },
  worldToGrid(x: number, y: number) { return this.viewToGrid(x, y); },
  gridToWorld(gx: number, gy: number) { return this.gridToView(gx, gy); },
  worldYawToView(yaw: number) { return yaw; },
  reset() {
    state.info = null; state.seq = 0; state.revision++; state.ready = false; state.status = null;
    state.statusUpdatedAt = 0; state.viewMode = 'global'; state.viewRobot = null;
    state.viewPreference = 'auto'; state.optimizedScopes = []; state.globalOptimizedScope = null;
    state.robotMapEpochs = {}; state.slamGraphs = {};
  }
};
