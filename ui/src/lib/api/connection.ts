import type { ClientMessage, ServerMessage, Point } from '$lib/types/protocol';
import { fleet } from '$lib/stores/fleet.svelte';
import { mapStore } from '$lib/stores/mapstore.svelte';
import { session } from '$lib/stores/session.svelte';
import { MockFleet } from './mock';

/**
 * Single connection to the backend, with automatic fallback to the local
 * simulator so the GUI is always usable.
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

function dispatch(msg: ServerMessage) {
  switch (msg.type) {
    case 'robot_state':
      fleet.apply(msg);
      break;
    case 'fleet_change':
      msg.robots.forEach((r) => fleet.apply(r));
      break;
    case 'map_info':
      mapStore.setInfo(msg.info);
      break;
    case 'map_patch':
      mapStore.applyPatch(msg);
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
  }
}

function startMock() {
  if (mock) return;
  session.setConnection('mock');
  mock = new MockFleet(dispatch, MOCK_ROBOTS);
  mock.start();
}

function connect() {
  if (FORCE_MOCK) {
    startMock();
    return;
  }
  session.setConnection('connecting');
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  try {
    ws = new WebSocket(`${proto}://${location.host}/ws`);
  } catch {
    startMock();
    return;
  }

  ws.onopen = () => {
    retry = 0;
    session.setConnection('live');
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
    session.setConnection('lost');
    // Two failed attempts and we fall back to the simulator rather than
    // leaving the operator with a dead screen.
    if (++retry >= 2) {
      startMock();
      return;
    }
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
    else if (msg.type === 'stop_all') mock.stopAll();
  }
}

/** THE chokepoint. Nothing else in the UI may talk to the backend. */
export function sendAction(msg: ClientMessage) {
  send(msg);
}

export const actions = {
  setGoal(robotId: string, p: Point) {
    sendAction({ type: 'set_goal', robot_id: robotId, payload: p });
  },
  cancelGoal(robotId: string) {
    sendAction({ type: 'cancel_goal', robot_id: robotId });
  },
  selectRobots(ids: string[]) {
    sendAction({ type: 'select_robots', robot_ids: ids });
  },
  switchCamera(robotId: string) {
    sendAction({ type: 'switch_camera', robot_id: robotId });
  },
  acknowledgeAlert(id: string) {
    session.acknowledge(id);
    sendAction({ type: 'acknowledge_alert', id });
  },
  reportTarget(robotId: string, p: Point) {
    sendAction({ type: 'report_target', robot_id: robotId, payload: p });
  },
  stopAll() {
    sendAction({ type: 'stop_all' });
  }
};

export function startConnection() {
  connect();
  setInterval(() => session.tick(1), 1000);
}

export function teardown() {
  if (retryTimer) clearTimeout(retryTimer);
  ws?.close();
  mock?.stop();
}
