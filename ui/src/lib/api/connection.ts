import type { ClientMessage, ServerMessage, Point } from '$lib/types/protocol';
import { fleet } from '$lib/stores/fleet.svelte';
import { mapStore } from '$lib/stores/mapstore.svelte';
import { session } from '$lib/stores/session.svelte';
import { settings } from '$lib/stores/settings.svelte';
import { detectionCatalog } from '$lib/stores/detection.svelte';
import { review } from '$lib/stores/review.svelte';
import { MockFleet } from './mock';

/**
 * Single connection to the backend. The local simulator is opt-in with
 * `?mock=1`; a live dashboard must never silently replace Gazebo data with
 * synthetic robots when the backend is unavailable.
 *
 * Every operator action goes through sendAction() — the one chokepoint that
 * stamps and logs, so the event log cannot drift from what the UI did.
 */

const MOCK_PARAM = new URLSearchParams(location.search).get('mock');
const FORCE_MOCK = MOCK_PARAM !== null && MOCK_PARAM !== '0';
const MOCK_ROBOTS = Number(new URLSearchParams(location.search).get('robots') ?? 4);

let ws: WebSocket | null = null;
let mock: MockFleet | null = null;
let retry = 0;
let retryTimer: number | null = null;
let tickTimer: number | null = null;
let started = false;

function dispatch(msg: ServerMessage) {
  switch (msg.type) {
    case 'robot_state':
      fleet.apply(msg);
      break;
    case 'fleet_change':
      msg.robots.forEach((r) => fleet.apply(r));
      break;
    case 'map_info':
      mapStore.setGlobalInfo(msg.info);
      void mapStore.loadFullPng(msg.info);
      break;
    case 'map_patch':
      mapStore.applyGlobalPatch(msg);
      break;
    case 'network_patch':
      mapStore.applyNetworkPatch(msg);
      break;
    case 'costmap':
      mapStore.applyCostmap(msg);
      break;
    case 'network_clear':
      mapStore.clearNetwork(msg.robot_id);
      break;
    case 'costmap_clear':
      mapStore.clearCostmaps(msg.robot_id);
      break;
    case 'detection':
      session.addDetection(msg.detection);
      break;
    case 'alert':
      session.addAlert(msg.alert);
      break;
    case 'alert_clear':
      session.clearAlert(msg.id);
      break;
    case 'session_state':
      session.setSession(msg);
      break;
    case 'settings_state':
      settings.apply(msg.settings);
      break;
    case 'detection_review':
      review.apply(msg);
      break;
    case 'slam_graph':
      mapStore.applySlamGraph(msg.robot_id, msg.graph);
      break;
    case 'sim_reset':
      session.applySimReset(msg);
      break;
  }
}

function startMock() {
  if (!started || mock) return;
  session.setConnection('mock');
  mock = new MockFleet(dispatch, MOCK_ROBOTS);
  mock.start();
}

function connect() {
  if (!started || ws?.readyState === WebSocket.CONNECTING || ws?.readyState === WebSocket.OPEN) {
    return;
  }
  if (FORCE_MOCK) {
    startMock();
    return;
  }
  session.setConnection('connecting');
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  try {
    ws = new WebSocket(`${proto}://${location.host}/ws`);
  } catch {
    session.setConnection('lost');
    retryTimer = window.setTimeout(connect, 1000);
    return;
  }

  ws.onopen = () => {
    retry = 0;
    if (retryTimer) clearTimeout(retryTimer);
    retryTimer = null;
    session.setConnection('live');
    // Re-announce what this dashboard is showing.
    //
    // The backend keys camera interest by the websocket, so the old socket
    // dying took the record with it, and nothing else re-sends it: the camera
    // panel only emits `switch_camera` when the SELECTED robot changes, which a
    // reconnect does not do. The result was every robot left on its 2 s idle
    // cadence while one was on screen, which the panel correctly reported as
    // "Link congested — not live" for the rest of the session. Measured: frame
    // age oscillating 200-2400 ms on all four robots, collapsing to 95-300 ms
    // the moment this message was sent by hand.
    if (fleet.activeCamera) {
      sendAction({ type: 'switch_camera', robot_id: fleet.activeCamera });
    }
    // Selection is per-socket for the same reason.
    if (fleet.selected.length) {
      sendAction({ type: 'select_robots', robot_ids: [...fleet.selected] });
    }
  };

  ws.onmessage = (e) => {
    try {
      dispatch(JSON.parse(e.data) as ServerMessage);
    } catch (err) {
      console.warn('[swarmdeck] bad message', err);
    }
  };

  ws.onclose = () => {
    ws = null;
    if (!started) return;
    session.setConnection('lost');
    retry++;
    retryTimer = setTimeout(connect, Math.min(1000 * retry, 5000)) as unknown as number;
  };

  ws.onerror = () => ws?.close();
}

function send(msg: ClientMessage) {
  if (ws?.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(msg));
  } else if (mock) {
    // Mirror commands into the simulator so the UI behaves identically.
    if (msg.type === 'set_goal') mock.command(msg.robot_id, 'set_goal', msg.payload);
    else if (msg.type === 'cancel_goal') mock.command(msg.robot_id, 'cancel_goal');
    else if (msg.type === 'drive' && msg.payload.linear === 0 && msg.payload.angular === 0)
      mock.command(msg.robot_id, 'cancel_goal');
    else if (msg.type === 'stop_all') mock.stopAll();
  }
}

/** THE chokepoint. Nothing else in the UI may talk to the backend. */
export function sendAction(msg: ClientMessage) {
  send(msg);
}

export const actions = {
  setGoal(robotId: string, p: Point) {
    if (!fleet.isEnabled(robotId)) return;
    sendAction({ type: 'set_goal', robot_id: robotId, payload: p });
  },
  cancelGoal(robotId: string) {
    sendAction({ type: 'cancel_goal', robot_id: robotId });
  },
  drive(robotId: string, linear: number, angular: number) {
    if (!fleet.isEnabled(robotId)) return;
    sendAction({ type: 'drive', robot_id: robotId, payload: { linear, angular } });
  },
  selectRobots(ids: string[]) {
    sendAction({ type: 'select_robots', robot_ids: ids });
  },
  switchCamera(robotId: string) {
    sendAction({ type: 'switch_camera', robot_id: robotId });
  },
  /** Focus a detection's source robot in the fleet, map, and camera. */
  focusRobot(robotId: string) {
    if (!fleet.isEnabled(robotId)) return;
    fleet.focus(robotId);
    sendAction({ type: 'select_robots', robot_ids: [...fleet.selected] });
    if (fleet.can(robotId, 'camera')) {
      sendAction({ type: 'switch_camera', robot_id: robotId });
    }
    // A detection is a world location; the source robot's local map is the
    // unambiguous frame to inspect after choosing it from a notification.
    void mapStore.setViewPreference('local', robotId);
  },
  acknowledgeAlert(id: string) {
    session.acknowledge(id);
    sendAction({ type: 'acknowledge_alert', id });
  },
  reportTarget(robotId: string, p: Point) {
    sendAction({ type: 'report_target', robot_id: robotId, payload: p });
  },
  async resetMap(robotId?: string) {
    const endpoint = robotId
      ? `/api/map/reset/${encodeURIComponent(robotId)}`
      : '/api/map/reset';
    const response = await fetch(endpoint, { method: 'POST' });
    const result = (await response.json()) as { error?: string; robots?: string[] };
    if (!response.ok) throw new Error(result.error ?? `map reset ${response.status}`);
    return result;
  },
  stopAll() {
    sendAction({ type: 'stop_all' });
  },
  /**
   * Simulation only, and fleet-wide. Nothing is set optimistically: the robots
   * report `explore` themselves once the adapter has actually started the
   * process, so a start that failed shows as robots that stay idle rather than
   * as a button that lies.
   */
  startExplore() {
    sendAction({ type: 'start_explore' });
  },
  stopExplore() {
    sendAction({ type: 'stop_explore' });
  },
  bodyCommand(robotId: string, action: 'claim' | 'release' | 'sit' | 'stand') {
    if (!fleet.isEnabled(robotId)) return;
    sendAction({ type: 'body_command', robot_id: robotId, action });
  },
  /**
   * Simulation only. The map clears through the ordinary patch path once the
   * adapters confirm, so nothing is cleared optimistically here — a reset that
   * fails must leave the map it failed to clear on screen.
   */
  resetSim() {
    sendAction({ type: 'reset_sim' });
  },

  /**
   * Detection review. Nothing is applied optimistically: the backend owns the
   * queue, two operators can answer the same proposal, and the authoritative
   * `detection_review` broadcast is what resolves that race.
   */
  acceptDetection(proposalId: string) {
    sendAction({ type: 'detection_accept', proposal_id: proposalId });
  },
  ignoreDetection(proposalId: string) {
    sendAction({ type: 'detection_ignore', proposal_id: proposalId });
  },
  mergeDetection(proposalId: string, entityId: string) {
    sendAction({ type: 'detection_merge', proposal_id: proposalId, entity_id: entityId });
  },
  forgetDetection(entityId: string) {
    sendAction({ type: 'detection_forget', entity_id: entityId });
  },
  forgetProposal(proposalId: string) {
    sendAction({ type: 'detection_forget', proposal_id: proposalId });
  },
  forgetAllDetections() {
    sendAction({ type: 'detection_forget_all' });
  },
  clearProposals() {
    sendAction({ type: 'detection_clear_proposals' });
  },
  deleteAllDetections() {
    sendAction({ type: 'detection_delete_all' });
  },
  clearIgnoredDetections() {
    sendAction({ type: 'detection_unignore' });
  }
};

export function startConnection() {
  if (started) return;
  started = true;
  // Static for the life of the backend, so it is fetched once rather than
  // pushed: the websocket carries what changes, not what a class is called.
  void detectionCatalog.load();
  connect();
  tickTimer = setInterval(() => session.tick(1), 1000) as unknown as number;
}

export function teardown() {
  if (!started) return;
  started = false;
  if (retryTimer) clearTimeout(retryTimer);
  if (tickTimer) clearInterval(tickTimer);
  retryTimer = null;
  tickTimer = null;
  if (ws) ws.onclose = null;
  ws?.close();
  ws = null;
  mock?.stop();
  mock = null;
  retry = 0;
}
