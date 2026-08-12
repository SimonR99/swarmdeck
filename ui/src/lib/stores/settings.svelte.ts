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

const fallback: AppSettings = {
  unattended_threshold_s: 45,
  alert_suppress_s: 30,
  robot_count: 4,
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
  robots: Array.from({ length: 4 }, (_, index) => ({
    id: `robot_${index}`,
    enabled: true,
    type: 'ros2',
    endpoint: 'ws://localhost:8080/adapter',
    color: DEFAULT_ROBOT_COLORS[index % DEFAULT_ROBOT_COLORS.length]
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
