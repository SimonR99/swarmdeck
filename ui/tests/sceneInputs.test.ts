import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  sceneDrawInputs,
  sceneInputsChanged,
  type SceneDrawState,
  type SceneDrawStores
} from '../src/lib/components/map3d/sceneInputs.ts';

/** Mutable stand-ins for the stores, shaped like the ones Map3D reads. */
function stores() {
  return {
    fleet: { sceneRevision: 3, selected: ['robot_0'] },
    settings: { value: { robots: [{ id: 'robot_0', enabled: true }] } },
    trails: { revision: 7 },
    mapStore: {
      revision: 11,
      info: { resolution: 0.05, transforms: { robot_0: { x: 0, y: 0, yaw: 0 } } },
      status: { global_members: ['robot_0'], reference: 'robot_0' },
      optimizedScopes: [{ scope: 'deployment:composite', robots: ['robot_0'] }] as unknown,
      globalOptimizedScope: 'deployment:composite' as string | null,
      viewMode: 'global',
      viewRobot: null as string | null,
      slamGraphs: {}
    },
    review: {
      proposals: [],
      entities: [],
      selected: null as string | null,
      focused: null as string | null
    },
    replicaTactical: { selection: null as unknown, preference: 'auto' }
  } satisfies SceneDrawStores;
}

function displayState(): SceneDrawState {
  return {
    liveReplicaRevision: 0,
    replicaCloud: null,
    follow: true,
    showGrid: true,
    showTrails: true,
    showLabels: true,
    showSensors: false,
    showPlans: true,
    showNetwork: false,
    quality: 'balanced',
    renderMode: 'voxels',
    colorMode: 'elevation',
    pointSize: 0.07
  };
}

test('a membership-only status update marks the scene dirty', () => {
  // The status poll can change who is on the global map without a network
  // patch, a SLAM graph or a single robot_state: nothing else would redraw.
  const sources = stores();
  const before = sceneDrawInputs(sources, displayState());
  sources.mapStore.status = { global_members: ['robot_0', 'robot_1'], reference: 'robot_0' };
  assert.equal(sceneInputsChanged(before, sceneDrawInputs(sources, displayState())), true);
});

test('stores that said nothing new leave the scene alone', () => {
  const sources = stores();
  const before = sceneDrawInputs(sources, displayState());
  assert.equal(sceneInputsChanged(before, sceneDrawInputs(sources, displayState())), false);
});

test('every store the frame is drawn from can make it dirty', () => {
  const change: Record<string, (sources: ReturnType<typeof stores>) => void> = {
    'robot telemetry': (s) => (s.fleet.sceneRevision += 1),
    selection: (s) => (s.fleet.selected = ['robot_1']),
    'robot colours and enablement': (s) => (s.settings.value = { robots: [] }),
    trails: (s) => (s.trails.revision += 1),
    'network patches': (s) => (s.mapStore.revision += 1),
    'raster transforms': (s) => (s.mapStore.info = { resolution: 0.1, transforms: {} }),
    'map membership': (s) => (s.mapStore.status = { global_members: [], reference: null }),
    'catalogue scopes': (s) => (s.mapStore.optimizedScopes = []),
    'displayed map scope': (s) => (s.mapStore.globalOptimizedScope = null),
    'view mode': (s) => (s.mapStore.viewMode = 'local'),
    'view robot': (s) => (s.mapStore.viewRobot = 'robot_1'),
    'loop closures': (s) => (s.mapStore.slamGraphs = { robot_0: { inter_robot: [] } }),
    'detection proposals': (s) => (s.review.proposals = [{ id: 'p1' }]),
    'detection entities': (s) => (s.review.entities = [{ id: 'e1' }]),
    'detection selection': (s) => (s.review.selected = 'e1'),
    'detection focus': (s) => (s.review.focused = 'p1'),
    'displayed replica': (s) => (s.replicaTactical.selection = { componentId: 'component:0' }),
    'replica preference': (s) => (s.replicaTactical.preference = 'component')
  };
  for (const [name, mutate] of Object.entries(change)) {
    const sources = stores();
    const before = sceneDrawInputs(sources, displayState());
    mutate(sources);
    assert.equal(
      sceneInputsChanged(before, sceneDrawInputs(sources, displayState())),
      true,
      `${name} did not mark the scene dirty`
    );
  }
});

test('every display option the frame is drawn from can make it dirty', () => {
  const change: ((state: SceneDrawState) => void)[] = [
    (s) => (s.liveReplicaRevision += 1),
    (s) => (s.replicaCloud = { partial: true }),
    (s) => (s.follow = false),
    (s) => (s.showGrid = false),
    (s) => (s.showTrails = false),
    (s) => (s.showLabels = false),
    (s) => (s.showSensors = true),
    (s) => (s.showPlans = false),
    (s) => (s.showNetwork = true),
    (s) => (s.quality = 'high'),
    (s) => (s.renderMode = 'mesh'),
    (s) => (s.colorMode = 'robot'),
    (s) => (s.pointSize = 0.2)
  ];
  const sources = stores();
  for (const mutate of change) {
    const next = displayState();
    mutate(next);
    assert.equal(sceneInputsChanged(sceneDrawInputs(sources, displayState()), sceneDrawInputs(sources, next)), true);
  }
});
