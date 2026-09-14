import type { ReplicaTacticalSelection } from '$lib/components/map3d/replicaTactical';

export type { ReplicaTacticalSelection };
export type ReplicaTacticalPreference = 'auto' | 'live' | 'component';
export type ReplicaTacticalAutoStatus = 'idle' | 'waiting-global';

const state = $state<{
  selection: ReplicaTacticalSelection | null;
  preference: ReplicaTacticalPreference;
  autoStatus: ReplicaTacticalAutoStatus;
  explicitShowRevision: number;
}>({ selection: null, preference: 'auto', autoStatus: 'idle', explicitShowRevision: 0 });

export const replicaTactical = {
  get selection() {
    return state.selection;
  },
  get preference() {
    return state.preference;
  },
  get autoStatus() {
    return state.autoStatus;
  },
  setAutoStatus(status: ReplicaTacticalAutoStatus) {
    state.autoStatus = status;
  },
  get explicitShowRevision() {
    return state.explicitShowRevision;
  },
  useAutomatic() {
    state.preference = 'auto';
  },
  show(selection: ReplicaTacticalSelection, preference: ReplicaTacticalPreference = 'component') {
    state.selection = { ...selection };
    state.preference = preference;
    if (preference !== 'auto') state.autoStatus = 'idle';
    if (preference === 'component') state.explicitShowRevision += 1;
  },
  clear(preference: ReplicaTacticalPreference = 'live') {
    state.selection = null;
    state.preference = preference;
    if (preference !== 'auto') state.autoStatus = 'idle';
  }
};
