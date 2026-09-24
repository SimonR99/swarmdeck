/**
 * Everything the 3D scene is drawn from.
 *
 * The map draws on demand, so a value that is used while drawing but never
 * read here is a value that can change without anything redrawing. Reading it
 * from the stores inside the render effect is what subscribes to it; the list
 * is kept in one place, and named, so that it can be audited against what the
 * frame actually reads and pinned by a test.
 *
 * What reads each of these while drawing:
 *
 * - `fleet.sceneRevision` — poses, goals, planner routes, nav status and mode,
 *   for the robot markers (`robot3d.ts`) and the goal, path and closure layers.
 * - `fleet.selected` — the selection reticle and the route widths.
 * - `settings.value` — robot colours and enablement (`fleet.colorOf`,
 *   `fleet.isEnabled`) and the single-target detection colour
 *   (`detectionCatalog.colorOf`).
 * - `trails.revision` — the recorded movement history.
 * - `mapStore.revision` — the network heatmap layers and their patches.
 * - `mapStore.info` — the transform the network decal is placed with, and the
 *   raster transform header that names the global map's members when the
 *   catalogue scope does not.
 * - `mapStore.status` — `global_members`, the SLAM merge's members. A status
 *   poll can change membership on its own, without a network patch, a SLAM
 *   graph or any telemetry.
 * - `mapStore.optimizedScopes` / `mapStore.globalOptimizedScope` — the
 *   displayed map's catalogue scope, which decides who is on the global map
 *   (`mapStore.globalMapMembers`, shared with the 2D canvas). A catalogue poll
 *   can change it without anything else changing.
 * - `mapStore.viewMode` / `mapStore.viewRobot` — local versus global
 *   membership, and the robot identity the markers are keyed by.
 * - `mapStore.slamGraphs` — the inter-robot loop closure lines.
 * - `review.*` — the detection crystals and the selected detection popover.
 * - `replicaTactical.selection` / `preference` — which cloud is displayed,
 *   whether live overlays are drawn on it, and whether follow mode applies.
 * - `liveReplicaRevision` — the live robot frame drawn on a replica. It is
 *   polled every second with new freshness ages, so it is counted only when
 *   what it draws changed (`liveReplicaDrawChanged`).
 *
 * Not listed, deliberately: `navigation.goalMode` only moves the ground
 * reticle, which follows the pointer and is redrawn by the pointer handler.
 */
export interface SceneDrawStores {
  fleet: { readonly sceneRevision: number; readonly selected: readonly string[] };
  settings: { readonly value: unknown };
  trails: { readonly revision: number };
  mapStore: {
    readonly revision: number;
    readonly info: unknown;
    readonly status: unknown;
    readonly optimizedScopes: unknown;
    readonly globalOptimizedScope: string | null;
    readonly viewMode: string;
    readonly viewRobot: string | null;
    readonly slamGraphs: unknown;
  };
  review: {
    readonly proposals: unknown;
    readonly entities: unknown;
    readonly selected: string | null;
    readonly focused: string | null;
  };
  replicaTactical: { readonly selection: unknown; readonly preference: string };
}

/** The component's own drawn state: display options and loaded replica data. */
export interface SceneDrawState {
  liveReplicaRevision: number;
  replicaCloud: unknown;
  follow: boolean;
  showGrid: boolean;
  showTrails: boolean;
  showLabels: boolean;
  showSensors: boolean;
  showPlans: boolean;
  showNetwork: boolean;
  quality: string;
  renderMode: string;
  colorMode: string;
  pointSize: number;
}

export function sceneDrawInputs(stores: SceneDrawStores, state: SceneDrawState): unknown[] {
  return [
    stores.fleet.sceneRevision,
    stores.fleet.selected,
    stores.settings.value,
    stores.trails.revision,
    stores.mapStore.revision,
    stores.mapStore.info,
    stores.mapStore.status,
    stores.mapStore.optimizedScopes,
    stores.mapStore.globalOptimizedScope,
    stores.mapStore.viewMode,
    stores.mapStore.viewRobot,
    stores.mapStore.slamGraphs,
    stores.review.proposals,
    stores.review.entities,
    stores.review.selected,
    stores.review.focused,
    stores.replicaTactical.selection,
    stores.replicaTactical.preference,
    state.liveReplicaRevision,
    state.replicaCloud,
    state.follow,
    state.showGrid,
    state.showTrails,
    state.showLabels,
    state.showSensors,
    state.showPlans,
    state.showNetwork,
    state.quality,
    state.renderMode,
    state.colorMode,
    state.pointSize
  ];
}

/** Whether the scene has to be drawn again since these inputs were last read. */
export function sceneInputsChanged(previous: readonly unknown[], next: readonly unknown[]) {
  if (previous.length !== next.length) return true;
  for (let i = 0; i < next.length; i++) if (!Object.is(previous[i], next[i])) return true;
  return false;
}
