import assert from 'node:assert/strict';
import test from 'node:test';
import {
  fetchLiveReplicaFrame,
  liveRobotToMapRobot,
  postLiveReplicaGoal,
  parseLiveReplicaFrame
} from '../src/lib/components/map3d/liveReplicaFrame.ts';
import {
  catalogueSelection,
  fetchReplicaCatalogue,
  parseReplicaCatalogue
} from '../src/lib/components/replicas/replicaCatalogue.ts';
import { ReplicaTacticalLoader, type ReplicaView } from '../src/lib/components/map3d/replicaTactical.ts';

const session = '12345678-1234-4234-8234-567812345678';
const component = 'component:merged';
const localDigest = 'a'.repeat(64);

function xyz() {
  const bytes = new Uint8Array(28);
  bytes.set(new TextEncoder().encode('SDXYZ1\0\0'));
  new DataView(bytes.buffer).setBigUint64(8, 1n, true);
  new DataView(bytes.buffer).setFloat32(16, 1, true);
  new DataView(bytes.buffer).setFloat32(20, 2, true);
  new DataView(bytes.buffer).setFloat32(24, 3, true);
  return bytes;
}

function view(scope: 'robot' | 'fleet', robotId: string): ReplicaView {
  const chunk = { sha256: localDigest, point_count: 1, size_bytes: 28 };
  const selected = {
    component_id: component,
    frame_id: 'component_frame',
    graph_revision: null,
    geometry_revision: 1,
    submaps: [{
      submap_id: 'robot_7/session/submap/1',
      geometry_revision: 1,
      pose_revision: 1,
      T_component_submap: [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
      chunks: [chunk]
    }]
  };
  return {
    robot_id: robotId,
    session_id: session,
    revision: scope === 'fleet' ? null : 1,
    snapshot_id: 'snapshot',
    solution_order: [1, 1],
    solution_order_known: true,
    component_id: component,
    components: [selected],
    selected,
    chunks: [chunk],
    source_age_s: null,
    scope
  };
}

function liveBody() {
  return {
    version: 1,
    mission_id: session,
    session_id: session,
    component_id: component,
    frame_id: 'component_frame',
    solution_order: [1, 1],
    robots: [{
      robot_id: 'robot_7', mission_id: session, component_id: component,
      navigation_frame: 'robot_7/map',
      T_component_navigation: [[1, 0, 0, 4], [0, 1, 0, 5], [0, 0, 1, 0], [0, 0, 0, 1]],
      pose: { x: 1, y: 2, z: 0, yaw: 0 }, goal: { x: 2, y: 2, z: 0, yaw: 0 },
      planned_path: [{ x: 1, y: 2 }, { x: 2, y: 2 }],
      global_planned_path: [], local_planned_path: [],
      freshness: { pose_s: 0, goal_s: 0, path_s: 0 }, nav_status: 'active', mode: 'nav'
    }]
  };
}

test('bounded mocked UI contract covers Global/Local source scopes, live icons, and component goal POST', async () => {
  const oldFetch = globalThis.fetch;
  const requests: string[] = [];
  globalThis.fetch = async (input, init) => {
    const url = String(input);
    requests.push(`${init?.method ?? 'GET'} ${url}`);
    if (url === '/api/autonomy/replicas/components') {
      return Response.json({
        version: 1, active_session_id: session,
        components: [{
          session_id: session, component_id: component, frame_id: 'component_frame',
          robot_ids: ['robot_7', 'robot_8'], source_count: 2, submap_count: 2,
          point_count: 1, available: true, status: 'ready', detail: '', solution_order: [1, 1],
          sources: []
        }]
      });
    }
    if (url.includes('/chunks/')) return new Response(xyz());
    if (url.includes('/components/live/') && init?.method === 'POST') return Response.json({ ok: true });
    if (url.includes('/components/live/')) return Response.json(liveBody());
    if (url.includes('/components/view/')) return Response.json(view('fleet', 'fleet'));
    if (url.includes('/replicas/view/robot_7/')) return Response.json(view('robot', 'robot_7'));
    throw new Error(`Unexpected fixture request ${url}`);
  };
  try {
    const catalogue = parseReplicaCatalogue(await fetchReplicaCatalogue());
    const entry = catalogue.components[0];
    assert.ok(entry);
    const globalSelection = catalogueSelection(entry, 'fleet', 'fleet');
    const localSelection = catalogueSelection(entry, 'robot', 'robot_7');

    const global = await new ReplicaTacticalLoader(1024, 10)
      .load(globalSelection, new AbortController().signal);
    const local = await new ReplicaTacticalLoader(1024, 10)
      .load(localSelection, new AbortController().signal);
    assert.equal(global?.view.scope, 'fleet');
    assert.equal(local?.view.scope, 'robot');

    const frame = await fetchLiveReplicaFrame(globalSelection);
    assert.ok(frame);
    const robot = liveRobotToMapRobot(parseLiveReplicaFrame(frame).robots[0]);
    assert.deepEqual(robot.pose, { x: 5, y: 7, yaw: 0 });
    await postLiveReplicaGoal(globalSelection, 'robot_7', [1, 1], { x: 5, y: 7, z: 0, yaw: 0 });

    assert.ok(requests.some((request) => request.startsWith('GET /api/autonomy/replicas/components/view/')));
    assert.ok(requests.some((request) => request.startsWith('GET /api/autonomy/replicas/view/robot_7/')));
    assert.ok(requests.some((request) => request.startsWith('POST /api/autonomy/replicas/components/live/')));
  } finally {
    globalThis.fetch = oldFetch;
  }
});
