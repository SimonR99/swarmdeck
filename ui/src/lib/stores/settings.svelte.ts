import type { AppSettings } from '$lib/types/protocol';
import { session } from '$lib/stores/session.svelte';

export const DEFAULT_ROBOT_COLORS = [
  '#007aff',
  '#8944ab',
  '#008f87',
  '#c93400',
  '#d30f72',
  '#b26a00',
  '#5865f2',
  '#2d8a3f'
];

/**
 * Stable DEFAULT colours for named hardware robots, independent of join order.
 * Mirrors IDENTITY_COLORS in the backend's config/settings.py.
 */
export const ROBOT_IDENTITY_COLORS: Record<string, string> = {
  spot: '#c9a000',
  botman: '#007aff',
  aslan: '#e07000'
};

/**
 * A configured colour always wins. The identity table is only what a robot
 * gets when nobody has chosen for it — checking it first made the settings
 * colour picker do nothing at all for spot, botman and aslan.
 */
export function colorForRobot(id: string, index = 0, configured?: string): string {
  if (configured) return configured;
  const stem = id.replace(/_\d+$/, '');
  if (ROBOT_IDENTITY_COLORS[stem]) return ROBOT_IDENTITY_COLORS[stem];
  return DEFAULT_ROBOT_COLORS[index % DEFAULT_ROBOT_COLORS.length];
}

const fallback: AppSettings = {
  unattended_threshold_s: 45,
  alert_suppress_s: 30,
  robot_count: 4,
  drive_control_mode: 'arrows',
  detection_enabled: true,
  detection_sensitivity: 0.55,
  // Overwritten by the backend's own catalog on first load; this is only what
  // the dialog shows in the moment before that arrives.
  detection_classes: [
    'rubber_duck',
    'wooden_block',
    'disc_cone',
    'filament_spool',
    'pool_noodle'
  ],
  detection_same_radius_m: 0.5,
  detection_ask_radius_m: 1.5,
  detection_class_floors: {
    rubber_duck: 0.25,
    wooden_block: 0.35,
    disc_cone: 0.2,
    filament_spool: 0.25,
    pool_noodle: 0.25
  },
  detection_robot_floors: {},
  // Derived by the backend; mirrored here only so the type is satisfied before
  // the first load. Nothing in the dialog writes it.
  detection_capture_floors: {
    rubber_duck: 0.25,
    wooden_block: 0.35,
    disc_cone: 0.2,
    filament_spool: 0.25,
    pool_noodle: 0.25
  },
  robots: Array.from({ length: 4 }, (_, index) => ({
    id: `robot_${index}`,
    enabled: true,
    color: colorForRobot(`robot_${index}`, index)
  }))
};

const state = $state({ value: structuredClone(fallback), loaded: false, saving: false });

export const settings = {
  get value() {
    return state.value;
  },
  get loaded() {
    return state.loaded;
  },
  get saving() {
    return state.saving;
  },

  apply(value: AppSettings) {
    state.value = structuredClone(value);
    session.applyDetectionSettings(value.detection_enabled, value.detection_classes);
    state.loaded = true;
  },

  async load() {
    const response = await fetch('/api/settings', { cache: 'no-store' });
    if (!response.ok) throw new Error(`settings ${response.status}`);
    const message = (await response.json()) as { settings: AppSettings };
    this.apply(message.settings);
  },

  async save(value: AppSettings) {
    state.saving = true;
    try {
      const response = await fetch('/api/settings', {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(value)
      });
      if (!response.ok) throw new Error(`settings ${response.status}`);
      const message = (await response.json()) as { settings: AppSettings };
      this.apply(message.settings);
    } finally {
      state.saving = false;
    }
  }
};
