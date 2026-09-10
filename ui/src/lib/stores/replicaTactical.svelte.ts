import type { ReplicaTacticalSelection } from '$lib/components/map3d/replicaTactical';

export type { ReplicaTacticalSelection };

const state = $state<{ selection: ReplicaTacticalSelection | null }>({ selection: null });

export const replicaTactical = {
  get selection() {
    return state.selection;
  },
  show(selection: ReplicaTacticalSelection) {
    state.selection = { ...selection };
  },
  clear() {
    state.selection = null;
  }
};
