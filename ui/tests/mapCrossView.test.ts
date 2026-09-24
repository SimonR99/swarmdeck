import assert from 'node:assert/strict';
import test from 'node:test';
import { readFileSync } from 'node:fs';
import ts from 'typescript';
import { automaticCatalogueEntry, catalogueSelection, parseReplicaCatalogue } from '../src/lib/components/replicas/replicaCatalogue.ts';
import { selectGlobalOptimizedScope } from '../src/lib/stores/optimizedScopes.ts';
import { parseLiveReplicaFrame } from '../src/lib/components/map3d/liveReplicaFrame.ts';
import { LiveReplicaPoll } from '../src/lib/components/map3d/liveReplicaPoll.ts';
import { replicaSelectionKey } from '../src/lib/components/map3d/replicaTactical.ts';
import * as membership from '../src/lib/components/map/mapMembership.ts';
import { projectRobotToRaster } from '../src/lib/components/map2d/mapFrames.ts';

const components = [
  { component_id: 'component:large', robot_ids: ['a', 'b', 'c'] },
  { component_id: 'component:small', robot_ids: ['d', 'e'] }
];
const catalogue = parseReplicaCatalogue({
  version: 1, active_session_id: 'mission', components: components.map((entry) => ({
    ...entry, session_id: 'mission', frame_id: entry.component_id,
    source_count: 1, submap_count: 1, point_count: 1, available: true,
    status: 'ready', solution_order: [1, 2], sources: []
  }))
});
const scopes = components.map((entry) => ({
  scope: entry.component_id, robots: entry.robot_ids, width: 10, height: 10,
  resolution: 1, origin: { x: 0, y: 0 }
}));

// Execute the actual component consumers, not parallel reimplementations of
// their membership rules. Three.js and the canvas itself need no DOM here.
function componentFunction(path: string, start: string, end: string, bindings: Record<string, unknown>) {
  const source = readFileSync(new URL(path, import.meta.url), 'utf8');
  const body = source.slice(source.indexOf(start), source.indexOf(end, source.indexOf(start)));
  const { outputText } = ts.transpileModule(body, { compilerOptions: { target: ts.ScriptTarget.ESNext } });
  return new Function(...Object.keys(bindings), `${outputText}\nreturn ${start.match(/function (\w+)/)![1]};`)(...Object.values(bindings));
}

test('global auto follows the raster scope even when fleet selection changes components', () => {
  const scope = selectGlobalOptimizedScope(scopes)!.scope;
  for (const selected of ['a', 'd', 'e', null]) {
    const entry = automaticCatalogueEntry(catalogue, selected, true, 2, true, scope);
    assert.equal(entry?.component_id, 'component:large');
    let shown: unknown;
    const apply = componentFunction('../src/lib/components/replicas/ReplicaCatalogueAuto.svelte', 'function apply()', '\n  // Re-evaluate', {
      replicaCatalogue: { catalogue },
      replicaTactical: { selection: null, preference: 'auto', setActiveMissionPresent() {},
        setMergedRobotIds() {}, setAutoStatus() {}, show: (selection: unknown) => { shown = selection; } },
      mapStore: { viewMode: 'global', selectedGlobalScope: scope },
      preferredRobotId: () => selected, enabled: true, automaticCatalogueEntry, catalogueSelection,
      activeMergedRobotIds: () => null
    });
    apply();
    assert.deepEqual(shown, { scope: 'fleet', robotId: 'fleet', sessionId: 'mission', componentId: 'component:large' });
  }
  assert.equal(automaticCatalogueEntry(catalogue, 'd', true, 2, true, 'component:missing'), null, 'wait rather than choose another component');
  assert.equal(automaticCatalogueEntry(catalogue, 'd', false, 1, false)?.component_id, 'component:small', 'Local still follows its robot');
});

test('actual 2D and live 3D consumers share membership through selection, expiry and read-only inspection', () => {
  let now = 1000;
  const robots = ['a', 'b', 'c', 'd', 'e'].map((robot_id) => ({ robot_id,
    pose: { x: 1, y: 2, yaw: 0 }, goal: null, planned_path: [] }));
  let selection = catalogueSelection(catalogue.components[0]);
  let localRobot: string | null = null;
  let readOnly = false;
  let registration = { frameId: selection.componentId, solutionOrder: [1, 2] };
  const poll = new LiveReplicaPoll({ onDrawChange() {}, onExpire() {}, stillWanted: () => true }, {
    clock: () => now, timers: { set: () => 0, clear() {} }
  });
  const disabled = new Set<string>();
  const fleet = { robots, get: (id: string) => robots.find((r) => r.robot_id === id), isEnabled: (id: string) => !disabled.has(id) };
  const mapStore = { viewMode: 'global', viewRobot: null, globalMapMembers: ['a', 'b', 'c'], info: { transforms: undefined } };
  const mapRobots = {
    get ids() { return membership.qualifiedMapRobotIds(robots, {
      localRobot, members: localRobot ? null : ['a', 'b', 'c'], isEnabled: fleet.isEnabled,
      selection, readOnly, live: poll.current, registration, now
    }); },
    get live() { return poll.current; }
  };
  const bindings = { fleet, mapStore, mapRobots, ...membership, performance: { now: () => now },
    overlayCache: { project: projectRobotToRaster } };
  const membersOfMap = componentFunction('../src/lib/components/map2d/MapView.svelte', 'function membersOfMap()', '\n  function robotsOnMap()', bindings);
  const raster = componentFunction('../src/lib/components/map2d/MapView.svelte', 'function robotsOnMap()', '\n  function centreOnFleet()', { ...bindings, membersOfMap });
  // Legacy bindings let the test expose the old cross-view mismatch as well.
  const scene = () => componentFunction('../src/lib/components/map3d/Map3D.svelte', 'function robotsOnMap()', '\n  // Public camera', {
    ...bindings, tacticalReplica: selection, liveTactical: !readOnly, liveReplicaPoll: poll,
    replicaCloud: { view: { solution_order_known: true, solution_order: [1, 2], selected: { frame_id: selection.componentId } } },
    ...liveBindings
  })();
  function publish(ids: string[]) {
    poll.select(replicaSelectionKey(selection));
    poll.set({ receivedAt: now, frame: parseLiveReplicaFrame({
      version: 1, mission_id: 'mission', session_id: 'mission', component_id: selection.componentId,
      frame_id: selection.componentId, solution_order: [1, 2], robots: ids.map((robot_id) => ({
        robot_id, mission_id: 'mission', component_id: selection.componentId, navigation_frame: robot_id,
        T_component_navigation: [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
        pose: { x: 1, y: 2, yaw: 0 }, goal: null, planned_path: [],
        freshness: { pose_s: robot_id === 'b' ? 2 : 0, goal_s: null, path_s: null }
      }))
    }) });
  }
  function both(expected: string[]) {
    assert.deepEqual(raster().map((r: { robot_id: string }) => r.robot_id), expected, '2D');
    assert.deepEqual(scene().map((r: { robot_id: string }) => r.robot_id), expected, '3D');
  }
  publish(['a', 'b', 'c']); both(['a', 'b', 'c']);
  disabled.add('a'); both(['b', 'c']); disabled.clear();
  now += 1001; both(['a', 'c']);
  now += 2000; both([]);
  selection = catalogueSelection(catalogue.components[1], 'robot', 'd');
  localRobot = 'd'; registration = { frameId: selection.componentId, solutionOrder: [1, 2] };
  poll.select(replicaSelectionKey(selection)); both([]);
  publish(['d', 'e']); both(['d']);
  registration = { frameId: 'wrong-frame', solutionOrder: [1, 2] }; both([]);
  registration = { frameId: selection.componentId, solutionOrder: [1, 3] }; both([]);
  registration = { frameId: selection.componentId, solutionOrder: [1, 2] };
  readOnly = true; both([]);
  readOnly = false; both(['d']);
  Object.assign(robots[3], { navigation_transform: { x: 0, y: 0, yaw: 0 } });
  assert.deepEqual(raster(), [], 'approved raster placement exception');
  assert.deepEqual(scene().map((r: { robot_id: string }) => r.robot_id), ['d']);
  poll.dispose();
});

import * as liveBindings from '../src/lib/components/map3d/liveReplicaFrame.ts';
