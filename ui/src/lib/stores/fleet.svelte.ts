import type { RobotState, Capability, RobotConnectionSettings } from '$lib/types/protocol';
import { settings, DEFAULT_ROBOT_COLORS, colorForRobot } from './settings.svelte';
import { mergeRobotState } from './robotStateMerge';
import { trails } from './trails.svelte';

/** Identity colour per robot. Named hardware robots keep a fixed colour; others use settings or the theme palette. */
export const ROBOT_COLORS = DEFAULT_ROBOT_COLORS;

const state = $state({
  robots: {} as Record<string, RobotState>,
  order: [] as string[],
  selected: [] as string[],
  activeCamera: null as string | null,
  /**
   * Bumped whenever the roster or a robot's telemetry actually changed. The
   * derived views below are rebuilt from it instead of on every read: they are
   * read once per robot per rendered frame, and a fresh array each time makes
   * change detection by identity impossible.
   */
  revision: 0,
  /**
   * Bumped only when something the maps draw changed. The record timestamps
   * and the unattended timer advance in every broadcast, so a map that watched
   * `revision` would redraw at the broadcast rate for ever.
   */
  sceneRevision: 0
});

let robotsView: RobotState[] = [];
let robotsViewRevision = -1;
let robotsViewConfigs: RobotConnectionSettings[] | null = null;

let idsView: string[] = [];
let idsViewSource: string[] | null = null;
let idsViewConfigs: RobotConnectionSettings[] | null = null;

let selectedView: string[] = [];
let selectedViewSource: string[] | null = null;
let selectedViewConfigs: RobotConnectionSettings[] | null = null;

let configIndex = new Map<string, RobotConnectionSettings>();
let configIndexSource: RobotConnectionSettings[] | null = null;

/** Per-robot connection settings, indexed once per settings broadcast. */
function configOf(id: string): RobotConnectionSettings | undefined {
  const configured = settings.value.robots;
  if (configured !== configIndexSource) {
    configIndexSource = configured;
    configIndex = new Map(configured.map((robot) => [robot.id, robot]));
  }
  return configIndex.get(id);
}

function getStoredTargetRobot(): string | null {
  if (typeof window === 'undefined') return null;
  try {
    const param = new URLSearchParams(window.location.search).get('robot');
    if (param) return param;
    return localStorage.getItem('swarmdeck_selected_robot');
  } catch {
    return null;
  }
}

function persistTargetRobot(id: string | null) {
  if (typeof window === 'undefined') return;
  try {
    const url = new URL(window.location.href);
    if (id) {
      url.searchParams.set('robot', id);
      localStorage.setItem('swarmdeck_selected_robot', id);
    } else {
      url.searchParams.delete('robot');
      localStorage.removeItem('swarmdeck_selected_robot');
    }
    window.history.replaceState({}, '', url.toString());
  } catch {}
}

let targetRobot = getStoredTargetRobot();
let targetMatched = false;

export const fleet = {
  get robots() {
    const revision = state.revision;
    const configured = settings.value.robots;
    if (revision !== robotsViewRevision || configured !== robotsViewConfigs) {
      robotsViewRevision = revision;
      robotsViewConfigs = configured;
      robotsView = state.order
        .map((id) => state.robots[id])
        .filter((robot) => robot && this.isEnabled(robot.robot_id));
    }
    return robotsView;
  },
  /**
   * Roster only: changes when a robot joins, leaves or is disabled, never when
   * one moves. Readers that care about membership use this so that telemetry
   * does not re-run their effects.
   */
  get robotIds() {
    const order = state.order;
    const configured = settings.value.robots;
    if (order !== idsViewSource || configured !== idsViewConfigs) {
      idsViewSource = order;
      idsViewConfigs = configured;
      idsView = order.filter((id) => this.isEnabled(id));
    }
    return idsView;
  },
  get revision() {
    return state.revision;
  },
  /** Advances when a robot's drawn state changed: pose, goal, route, status. */
  get sceneRevision() {
    return state.sceneRevision;
  },
  get count() {
    return this.robots.length;
  },
  get selected() {
    const source = state.selected;
    const configured = settings.value.robots;
    if (source !== selectedViewSource || configured !== selectedViewConfigs) {
      selectedViewSource = source;
      selectedViewConfigs = configured;
      selectedView = source.filter((id) => this.isEnabled(id));
    }
    return selectedView;
  },
  get activeCamera() {
    const cam = state.activeCamera;
    if (cam && this.isEnabled(cam) && this.can(cam, 'camera')) return cam;
    return this.robots.find((robot) => robot.capabilities?.includes('camera'))?.robot_id ?? null;
  },
  get online() {
    return this.robots.filter((robot) => robot.online).length;
  },
  get(id: string): RobotState | undefined {
    return state.robots[id];
  },

  isEnabled(id: string): boolean {
    return configOf(id)?.enabled !== false;
  },

  colorOf(id: string): string {
    const i = state.order.indexOf(id);
    return colorForRobot(id, i < 0 ? 0 : i, configOf(id)?.color);
  },

  indexOf(id: string): number {
    return state.order.indexOf(id);
  },

  can(id: string, cap: Capability): boolean {
    return this.isEnabled(id) && (state.robots[id]?.capabilities?.includes(cap) ?? false);
  },

  isSelected(id: string): boolean {
    return this.selected.includes(id);
  },

  /** Upsert from a robot_state message. */
  apply(msg: RobotState) {
    if (!state.robots[msg.robot_id]) {
      state.order = [...state.order, msg.robot_id];
      if (targetRobot && msg.robot_id === targetRobot && this.isEnabled(msg.robot_id)) {
        state.selected = [msg.robot_id];
        targetMatched = true;
        if (msg.capabilities?.includes('camera')) {
          state.activeCamera = msg.robot_id;
        }
        persistTargetRobot(msg.robot_id);
      } else if (!targetMatched && state.selected.length === 0 && this.isEnabled(msg.robot_id)) {
        state.selected = [msg.robot_id];
        if (
          this.activeCamera === null &&
          msg.capabilities?.includes('camera')
        ) {
          state.activeCamera = msg.robot_id;
        }
        if (!targetRobot) {
          persistTargetRobot(msg.robot_id);
        }
      }
    } else if (targetRobot && !targetMatched && msg.robot_id === targetRobot && this.isEnabled(msg.robot_id)) {
      state.selected = [msg.robot_id];
      targetMatched = true;
      if (msg.capabilities?.includes('camera')) {
        state.activeCamera = msg.robot_id;
      }
      persistTargetRobot(msg.robot_id);
    }
    // A message that repeats what the store already holds is dropped here:
    // the server sends a keep-alive for robots that have not changed, and a
    // write would invalidate every reader of the fleet at that cadence.
    const update = mergeRobotState(state.robots[msg.robot_id], msg);
    if (!update.changed) return;
    state.robots[msg.robot_id] = update.value;
    state.revision++;
    if (update.drawable) state.sceneRevision++;
    // Where the robot has been is recorded from its telemetry, so it is the
    // same history whichever map view is on screen.
    trails.record(msg.robot_id, msg.pose.x, msg.pose.y, msg.navigation_transform);
  },

  sync(robots: RobotState[]) {
    const presentIds = new Set(robots.map((r) => r.robot_id));
    for (const id of state.order) {
      if (!presentIds.has(id)) {
        this.remove(id);
      }
    }
    robots.forEach((r) => this.apply(r));
  },

  remove(id: string) {
    delete state.robots[id];
    trails.clear(id);
    state.revision++;
    state.sceneRevision++;
    state.order = state.order.filter((r) => r !== id);
    state.selected = state.selected.filter((r) => r !== id);
    if (state.activeCamera === id) {
      state.activeCamera = state.order[0] ?? null;
    }
    if (targetRobot === id) {
      targetRobot = state.selected[0] ?? null;
      targetMatched = Boolean(targetRobot);
      persistTargetRobot(targetRobot);
    }
  },

  select(id: string, additive = false) {
    if (!this.isEnabled(id)) return;
    if (additive) {
      state.selected = state.selected.includes(id)
        ? state.selected.filter((r) => r !== id)
        : [...state.selected, id];
    } else {
      state.selected = state.selected.includes(id) && state.selected.length === 1 ? [] : [id];
    }
    // Selecting a robot also brings up its camera, if it has one.
    if (state.selected.includes(id) && this.can(id, 'camera')) {
      state.activeCamera = id;
    }
    targetRobot = state.selected[0] ?? null;
    targetMatched = Boolean(targetRobot);
    persistTargetRobot(targetRobot);
  },

  selectAll() {
    const ids = this.robots.map((robot) => robot.robot_id);
    state.selected = this.selected.length === ids.length ? [] : ids;
    targetRobot = state.selected[0] ?? null;
    targetMatched = Boolean(targetRobot);
    persistTargetRobot(targetRobot);
  },

  setCamera(id: string) {
    if (!this.isEnabled(id) || !this.can(id, 'camera')) return;
    state.activeCamera = id;
  },

  /** Make one robot the operator focus for camera, map, and navigation. */
  focus(id: string) {
    if (!this.isEnabled(id)) return;
    state.selected = [id];
    if (this.can(id, 'camera')) state.activeCamera = id;
    targetRobot = id;
    targetMatched = true;
    persistTargetRobot(id);
  },

  reset() {
    state.robots = {};
    state.order = [];
    state.selected = [];
    state.activeCamera = null;
    trails.clear();
    state.revision++;
    state.sceneRevision++;
  }
};
