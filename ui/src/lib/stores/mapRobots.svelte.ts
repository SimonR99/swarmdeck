import { untrack } from 'svelte';
import { fleet } from './fleet.svelte';
import { mapStore } from './mapstore.svelte';
import { replicaCatalogue } from './replicaCatalogue.svelte';
import { replicaTactical } from './replicaTactical.svelte';
import { LiveReplicaPoll } from '$lib/components/map3d/liveReplicaPoll';
import { replicaSelectionKey } from '$lib/components/map3d/replicaTactical';
import { localRobotOf, qualifiedMapRobotIds, type MapReplicaRegistration } from '$lib/components/map/mapMembership';

let revision = $state(0);
let freshnessRevision = $state(0);
let cloudRequired = $state(false);
let cloud = $state<{ key: string; registration: MapReplicaRegistration | null } | null>(null);
const changed = () => { revision = untrack(() => revision) + 1; };
const poll = new LiveReplicaPoll({
  onDrawChange: changed,
  onExpire: () => {
    freshnessRevision = untrack(() => freshnessRevision) + 1;
    changed();
  },
  stillWanted: () => replicaTactical.preference === 'auto' && Boolean(replicaTactical.selection)
});

const ids = $derived.by(() => {
  void revision;
  const selection = replicaTactical.selection;
  const entry = replicaCatalogue.catalogue?.components.find((entry) =>
    entry.session_id === selection?.sessionId && entry.component_id === selection?.componentId);
  // When 3D is visible, its actually committed geometry is the registration
  // authority for BOTH views. A 2D-only session uses the verified catalogue.
  const registration = cloudRequired
    ? (selection && cloud?.key === replicaSelectionKey(selection) ? cloud.registration : null)
    : entry ? { frameId: entry.frame_id, solutionOrder: entry.solution_order } : null;
  return qualifiedMapRobotIds(fleet.robotIds.map((robot_id) => ({ robot_id })), {
    localRobot: localRobotOf(mapStore.viewMode, mapStore.viewRobot),
    members: mapStore.globalMapMembers,
    isEnabled: (id) => fleet.isEnabled(id),
    selection,
    readOnly: replicaTactical.preference === 'component',
    live: poll.current,
    registration,
    now: performance.now()
  });
});

/** MapView owns this poll's lifecycle; dimension switches never stop telemetry. */
export const mapRobots = {
  get ids() { return ids; },
  get revision() { return revision; },
  get freshnessRevision() { return freshnessRevision; },
  get live() { return poll.current; },
  select() {
    const selection = replicaTactical.selection;
    poll.select(replicaTactical.preference === 'auto' && selection ? replicaSelectionKey(selection) : '');
  },
  refresh() {
    const selection = replicaTactical.selection;
    if (replicaTactical.preference === 'auto' && selection) void poll.refresh(selection);
  },
  setCloud(required: boolean, key: string, registration: MapReplicaRegistration | null) {
    cloudRequired = required;
    cloud = { key, registration };
  },
  clear() { poll.set(null); },
  dispose() { poll.select(''); poll.dispose(); }
};
