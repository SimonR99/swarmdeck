import type {
  DetectionEntity,
  DetectionProposal,
  DetectionReview
} from '$lib/types/protocol';
import { settings } from '$lib/stores/settings.svelte';

/**
 * Operator review of map detections.
 *
 * Kept apart from `session.detections` on purpose. That store holds live camera
 * tracks — what a robot can see this instant, retracted the moment the object
 * leaves frame. This one holds what the fleet has agreed is *there*, which
 * outlives any sighting and is the thing the operator is being asked about.
 */
const state = $state({
  entities: [] as DetectionEntity[],
  proposals: [] as DetectionProposal[],
  ignored: 0,
  radii: { same: 0.5, ask: 1.5, ignore: 1.0 },
  /** Proposal the operator is pointing at, so the map can highlight it. */
  focused: null as string | null,
  /** Detection the operator selected from the map or notification panel. */
  selected: null as string | null
});

export const review = {
  get entities() {
    return state.entities;
  },
  get proposals() {
    return state.proposals;
  },
  get ignored() {
    return state.ignored;
  },
  get radii() {
    return state.radii;
  },
  get focused() {
    return state.focused;
  },
  get selected() {
    return state.selected;
  },
  get highlighted() {
    return state.selected ?? state.focused;
  },
  get pending() {
    return state.proposals.length;
  },

  entityOf(id: string | null): DetectionEntity | undefined {
    return id ? state.entities.find((e) => e.id === id) : undefined;
  },

  proposalOf(id: string | null): DetectionProposal | undefined {
    return id ? state.proposals.find((p) => p.id === id) : undefined;
  },

  /**
   * Other confirmed objects of the same class (or all objects when cross-class
   * merging is active), nearest first — the candidates a merge could target
   * when the backend's own suggestion is not the one the operator means.
   */
  mergeCandidates(proposal: DetectionProposal): DetectionEntity[] {
    const crossClass =
      settings.value.detection_single_mode ||
      settings.value.detection_cross_class_merge;
    return state.entities
      .filter((e) => crossClass || e.class === proposal.class)
      .map((e) => ({
        entity: e,
        d: Math.hypot(e.position.x - proposal.position.x, e.position.y - proposal.position.y)
      }))
      .sort((a, b) => a.d - b.d)
      .map((row) => row.entity);
  },

  focus(id: string | null) {
    state.focused = id;
  },

  select(id: string | null) {
    state.selected = id;
  },

  apply(message: DetectionReview) {
    state.entities = message.entities ?? [];
    state.proposals = message.proposals ?? [];
    state.ignored = message.ignored ?? 0;
    if (message.radii) state.radii = message.radii;
    // A proposal that was answered elsewhere must not stay highlighted on the
    // map pointing at nothing.
    if (
      state.focused &&
      !state.proposals.some((p) => p.id === state.focused) &&
      !state.entities.some((e) => e.id === state.focused)
    ) {
      state.focused = null;
    }
    if (
      state.selected &&
      !state.proposals.some((p) => p.id === state.selected) &&
      !state.entities.some((e) => e.id === state.selected)
    ) {
      state.selected = null;
    }
  },

  clear() {
    state.entities = [];
    state.proposals = [];
    state.ignored = 0;
    state.focused = null;
    state.selected = null;
  }
};
