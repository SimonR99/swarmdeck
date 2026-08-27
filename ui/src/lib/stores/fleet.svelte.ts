import type { RobotState, Capability } from '$lib/types/protocol';
import { settings, DEFAULT_ROBOT_COLORS, colorForRobot } from './settings.svelte';

/** Identity colour per robot. Named hardware robots keep a fixed colour; others use settings or the theme palette. */
export const ROBOT_COLORS = DEFAULT_ROBOT_COLORS;

const state = $state({
  robots: {} as Record<string, RobotState>,
  order: [] as string[],
  selected: [] as string[],
  activeCamera: null as string | null
});

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
    return state.order.map((id) => state.robots[id]).filter((robot) => robot && this.isEnabled(robot.robot_id));
  },
  get count() {
    return this.robots.length;
  },
  get selected() {
    return state.selected.filter((id) => this.isEnabled(id));
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
    const config = settings.value.robots.find((r) => r.id === id);
    return config?.enabled !== false;
  },

  colorOf(id: string): string {
    const config = settings.value.robots.find((r) => r.id === id);
    const i = state.order.indexOf(id);
    return colorForRobot(id, i < 0 ? 0 : i, config?.color);
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
    state.robots[msg.robot_id] = { ...state.robots[msg.robot_id], ...msg };
  },

  remove(id: string) {
    delete state.robots[id];
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
  }
};
