import type { DetectionClass } from '$lib/types/protocol';
import { settings } from '$lib/stores/settings.svelte';

/**
 * Marker colour per detection class.
 *
 * One colour per class, used identically by the camera overlay, the settings
 * toggles and the map. The map is what constrains the choice: a detection
 * there is a 7 px ringed dot with no label (see MapView), so hue is the only
 * thing telling one class from another.
 *
 * Hues are therefore spread deliberately -- 0, 45, 85, 199, 280 degrees, no
 * two closer than 40 -- rather than picked one at a time. Three of these were
 * originally inside a single 45 degree arc of red/orange/amber, which is fine
 * on a labelled overlay box and unreadable as a dot.
 *
 * Where an object has a real colour it keeps it, because a mnemonic marker is
 * one the operator does not have to decode: the cone is red-orange, the duck
 * yellow, the noodle blue. The two objects with no colour of their own (a bare
 * wooden block, a spool of black filament) take the leftover hues.
 *
 * These are all brighter than the robot palette in `settings.svelte.ts`, which
 * is what separates a detection from a robot where the hues run close.
 */
const CLASS_COLORS: Record<string, string> = {
  disc_cone: '#ef4444',      // red
  rubber_duck: '#fbbf24',    // amber
  wooden_block: '#84cc16',   // lime
  pool_noodle: '#38bdf8',    // sky
  filament_spool: '#c084fc'  // purple
};

const FALLBACK_COLOR = '#fbbf24';

const state = $state({ classes: [] as DetectionClass[], loaded: false });

export const detectionCatalog = {
  get classes() {
    return state.classes;
  },
  get loaded() {
    return state.loaded;
  },

  /**
   * Fetch the catalog once at startup.
   *
   * Failure is survivable and deliberately quiet: the dashboard falls back to
   * raw class names, which is worse-looking but never wrong.
   */
  async load() {
    try {
      const response = await fetch('/api/detection/classes', { cache: 'no-store' });
      if (!response.ok) return;
      const message = (await response.json()) as { classes: DetectionClass[] };
      if (Array.isArray(message.classes)) {
        state.classes = message.classes;
        state.loaded = true;
      }
    } catch {
      /* keep the raw names */
    }
  },

  rawLabelOf(name: string): string {
    return state.classes.find((c) => c.name === name)?.label ?? name;
  },

  rawColorOf(name: string): string {
    return CLASS_COLORS[name] ?? FALLBACK_COLOR;
  },

  labelOf(name: string): string {
    if (settings.value.detection_single_mode) {
      return settings.value.detection_single_name || 'Target';
    }
    return this.rawLabelOf(name);
  },

  colorOf(name: string): string {
    if (settings.value.detection_single_mode) {
      return settings.value.detection_single_color || '#fbbf24';
    }
    return this.rawColorOf(name);
  }
};

