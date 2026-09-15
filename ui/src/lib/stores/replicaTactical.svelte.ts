import type { ReplicaTacticalSelection } from '$lib/components/map3d/replicaTactical';

export type { ReplicaTacticalSelection };
export type ReplicaTacticalPreference = 'auto' | 'live' | 'component';
export type ReplicaTacticalAutoStatus = 'idle' | 'waiting-global';

const state = $state<{
  selection: ReplicaTacticalSelection | null;
  preference: ReplicaTacticalPreference;
  autoStatus: ReplicaTacticalAutoStatus;
  mergedRobotIds: string[] | null;
  activeMissionPresent: boolean | null;
  explicitShowRevision: number;
}>({ selection: null, preference: 'auto', autoStatus: 'idle', mergedRobotIds: null, activeMissionPresent: null, explicitShowRevision: 0 });

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
  get mergedRobotIds() {
    return state.mergedRobotIds;
  },
  setMergedRobotIds(robotIds: string[] | null) {
    state.mergedRobotIds = robotIds ? [...robotIds] : null;
  },
  get activeMissionPresent() {
    return state.activeMissionPresent;
  },
  setActiveMissionPresent(present: boolean) {
    state.activeMissionPresent = present;
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
