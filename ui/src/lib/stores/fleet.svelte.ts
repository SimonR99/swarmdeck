import type { RobotState, Capability } from '$lib/types/protocol';

/** Identity colour per robot, assigned by join order. Max 5 (see requirements). */
export const ROBOT_COLORS = [
  'var(--color-robot-0)',
  'var(--color-robot-1)',
  'var(--color-robot-2)',
  'var(--color-robot-3)',
  'var(--color-robot-4)'
];

const state = $state({
  robots: {} as Record<string, RobotState>,
  order: [] as string[],
  selected: [] as string[],
  activeCamera: null as string | null
});

export const fleet = {
  get robots() {
    return state.order.map((id) => state.robots[id]).filter(Boolean);
  },
  get count() {
    return state.order.length;
  },
  get selected() {
    return state.selected;
  },
  get activeCamera() {
    return state.activeCamera;
  },
  get online() {
    return state.order.filter((id) => state.robots[id]?.online).length;
  },

  get(id: string): RobotState | undefined {
    return state.robots[id];
  },

  colorOf(id: string): string {
    const i = state.order.indexOf(id);
    return ROBOT_COLORS[i < 0 ? 0 : i % ROBOT_COLORS.length];
  },

  indexOf(id: string): number {
    return state.order.indexOf(id);
  },

  can(id: string, cap: Capability): boolean {
    return state.robots[id]?.capabilities?.includes(cap) ?? false;
  },

  isSelected(id: string): boolean {
    return state.selected.includes(id);
  },

  /** Upsert from a robot_state message. */
  apply(msg: RobotState) {
    if (!state.robots[msg.robot_id]) {
      state.order = [...state.order, msg.robot_id];
      // First robot to arrive becomes the default camera.
      if (state.activeCamera === null && msg.capabilities?.includes('camera')) {
        state.activeCamera = msg.robot_id;
      }
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
  },

  select(id: string, additive = false) {
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
  },

  selectAll() {
    state.selected = state.selected.length === state.order.length ? [] : [...state.order];
  },

  setCamera(id: string) {
    state.activeCamera = id;
  },

  reset() {
    state.robots = {};
    state.order = [];
    state.selected = [];
    state.activeCamera = null;
  }
};
