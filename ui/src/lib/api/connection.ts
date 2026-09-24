import type { ClientMessage, ServerMessage, Point } from '$lib/types/protocol';
import { fleet } from '$lib/stores/fleet.svelte';
import { mapStore } from '$lib/stores/mapstore.svelte';
import { session } from '$lib/stores/session.svelte';
import { settings } from '$lib/stores/settings.svelte';
import { detectionCatalog } from '$lib/stores/detection.svelte';
import { review } from '$lib/stores/review.svelte';
import type { MockFleet } from './mock';
import { canResetSimulation, fetchJsonWithTimeout, resetRequestId, resetRobotMap } from './resetHttp';

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
let resetPoll: Promise<import('$lib/types/protocol').SimResetSupervisorStatus> | null = null;
let resetPollRequestId: string | null = null;

type ResetStatus = import('$lib/types/protocol').SimResetSupervisorStatus & {
  supervisor_available?: boolean;
};
const RESET_POLL_TIMEOUT_MS = 600_000;
const RESET_FETCH_TIMEOUT_MS = 5_000;

async function fetchResetStatus(): Promise<[Response, ResetStatus]> {
  try {
    const result = await fetchJsonWithTimeout<ResetStatus>(
      '/api/sim/reset', { cache: 'no-store' }, RESET_FETCH_TIMEOUT_MS
    );
    session.setResetSupervisorAvailable(result[0].ok && canResetSimulation(result[1]));
    return result;
  } catch (error) {
    session.setResetSupervisorAvailable(false);
    throw error;
  }
}

function publishResetStatus(status: ResetStatus) {
  const active = ['accepted', 'stopping', 'starting', 'verifying'].includes(status.phase);
  session.applySimReset({
    type: 'sim_reset', phase: active ? 'start' : 'done', skipped: [],
    request_id: status.request_id,
    ok: status.phase === 'done' && status.ok === true,
    failed: status.phase === 'failed' ? ['supervisor'] : [],
    error: status.error
  });
}

function pollReset(
  requestId: string,
  deadline = Date.now() + RESET_POLL_TIMEOUT_MS
): Promise<ResetStatus> {
  if (resetPoll && resetPollRequestId === requestId) return resetPoll;
  resetPollRequestId = requestId;
  resetPoll = (async () => {
    while (Date.now() < deadline) {
      await new Promise((resolve) => setTimeout(resolve, 1_000));
      try {
        const [response, status] = await fetchResetStatus();
        if (!response.ok) continue;
        if (status.request_id !== requestId) continue;
        publishResetStatus(status);
        if (status.phase === 'done' || status.phase === 'failed') return status;
      } catch {
        // The server is deliberately recreated. The loaded UI remains the
        // lifecycle owner and resumes as soon as its proxy has an upstream.
      }
    }
    const timedOut: ResetStatus = {
      version: 1, phase: 'failed', ok: false, request_id: requestId,
      error: 'simulation reset did not complete within 600 seconds'
    };
    publishResetStatus(timedOut);
    throw new Error(timedOut.error);
  })().finally(() => {
    if (resetPollRequestId === requestId) {
      resetPoll = null;
      resetPollRequestId = null;
    }
  });
  return resetPoll;
}

async function submitReset(requestId: string): Promise<[Response, ResetStatus]> {
  return fetchJsonWithTimeout<ResetStatus>(
    `/api/sim/reset?request_id=${encodeURIComponent(requestId)}`,
    { method: 'POST' },
    RESET_FETCH_TIMEOUT_MS
  );
}

async function recoverLostResetResponse(
  requestId: string,
  deadline: number
): Promise<ResetStatus> {
  // The host may stop the backend after it durably accepts the request but
  // before its HTTP response reaches this browser. Retry the same idempotency
  // key after reconnect: a completed request is returned, never run again.
  while (Date.now() < deadline) {
    await new Promise((resolve) => setTimeout(resolve, 1_000));
    let response: Response;
    let status: ResetStatus;
    try {
      [response, status] = await submitReset(requestId);
    } catch {
      // A transport failure is expected until the replacement server is live.
      continue;
    }
    publishResetStatus(status);
    if (!response.ok || status.phase === 'failed') {
      throw new Error(status.error ?? `simulation reset ${response.status}`);
    }
    if (!status.request_id || status.phase === 'done') {
      return status;
    }
    return pollReset(status.request_id, deadline);
  }
  const timedOut: ResetStatus = {
    version: 1, phase: 'failed', ok: false, request_id: requestId,
    error: 'simulation reset request could not be recovered within 600 seconds'
  };
  publishResetStatus(timedOut);
  throw new Error(timedOut.error);
}

async function resumeReset() {
  try {
    const [response, status] = await fetchResetStatus();
    if (!response.ok) return;
    if (
      status.request_id &&
      ['accepted', 'stopping', 'starting', 'verifying'].includes(status.phase)
    ) {
      publishResetStatus(status);
      void pollReset(status.request_id).catch((error) =>
        console.warn('[swarmdeck] reset monitoring failed', error)
      );
    }
  } catch {
    // Startup during a server recreation is expected; websocket reconnect will
    // invoke this again once the API is reachable.
  }
}

function dispatch(msg: ServerMessage) {
  switch (msg.type) {
    case 'robot_state':
      fleet.apply(msg);
      break;
    case 'fleet_change':
      fleet.sync(msg.robots);
      break;
    case 'network_patch':
      mapStore.applyNetworkPatch(msg);
      break;
    case 'network_clear':
      mapStore.clearNetwork(msg.robot_id);
      break;
    case 'robot_map_reset':
      mapStore.applyRobotMapReset(msg.robot_id, msg.mission_id, msg.map_epoch);
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
    case 'sim_reset':
      session.applySimReset(msg);
      break;
  }
}

/**
 * The local simulator, fetched only when `?mock` asked for it. A live
 * dashboard must never load synthetic robots, let alone ship them in the
 * bundle it parses at startup.
 */
async function startMock() {
  if (!started || mock) return;
  const { MockFleet } = await import('./mock');
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
    void startMock();
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
    void resumeReset();
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
    if (msg.type === 'cancel_goal') mock.command(msg.robot_id, 'cancel_goal');
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
  explore(enabled: boolean) {
    for (const robot of fleet.robots) {
      if (!fleet.can(robot.robot_id, 'explore') || (enabled && !robot.online)) continue;
      sendAction({ type: enabled ? 'start_explore' : 'stop_explore', robot_id: robot.robot_id });
    }
  },
  cancelGoal(robotId: string) {
    sendAction({ type: 'cancel_goal', robot_id: robotId });
  },
  returnHome(robotId: string) {
    if (!fleet.isEnabled(robotId)) return;
    sendAction({ type: 'return_home', robot_id: robotId });
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
    if (robotId) return resetRobotMap(robotId);
    const response = await fetch('/api/map/reset', { method: 'POST' });
    const result = (await response.json()) as { error?: string; robots?: string[] };
    if (!response.ok) throw new Error(result.error ?? `map reset ${response.status}`);
    return result;
  },
  stopAll() {
    sendAction({ type: 'stop_all' });
  },
  bodyCommand(robotId: string, action: string, height?: number) {
    if (!fleet.isEnabled(robotId)) return;
    sendAction({
      type: 'body_command',
      robot_id: robotId,
      action,
      ...(height !== undefined ? { height } : {})
    });
  },
  /**
   * Simulation only. The map clears through the ordinary patch path once the
   * adapters confirm, so nothing is cleared optimistically here — a reset that
   * fails must leave the map it failed to clear on screen.
   */
  async resetSim() {
    const requestId = resetRequestId();
    const deadline = Date.now() + RESET_POLL_TIMEOUT_MS;
    let response: Response;
    let accepted: ResetStatus;
    try {
      [response, accepted] = await submitReset(requestId);
    } catch {
      accepted = { version: 1, request_id: requestId, phase: 'accepted', ok: null };
      publishResetStatus(accepted);
      return recoverLostResetResponse(requestId, deadline);
    }
    if (!response.ok || accepted.phase === 'failed') {
      publishResetStatus(accepted);
      throw new Error(accepted.error ?? `simulation reset ${response.status}`);
    }
    publishResetStatus(accepted);
    if (accepted.phase === 'done') return accepted;
    return pollReset(accepted.request_id ?? requestId, deadline);
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
  },
  discardRobot(robotId: string) {
    fleet.remove(robotId);
    mapStore.clearNetwork(robotId);
    sendAction({ type: 'discard_robot', robot_id: robotId });
    void fetch(`/api/fleet/${encodeURIComponent(robotId)}`, { method: 'DELETE' }).catch(() => {});
  }
};

export function startConnection() {
  if (started) return;
  started = true;
  // Static for the life of the backend, so it is fetched once rather than
  // pushed: the websocket carries what changes, not what a class is called.
  void detectionCatalog.load();
  void resumeReset();
  connect();
  tickTimer = setInterval(() => session.tick(1), 1000) as unknown as number;
  // The map view owns the map's polling: it is the only thing that displays a
  // raster, and it knows when one is on screen. See mapPollScheduler.ts.
}

export function teardown() {
  if (!started) return;
  started = false;
  clearTimeout(retryTimer ?? undefined);
  clearInterval(tickTimer ?? undefined);
  retryTimer = null;
  tickTimer = null;
  if (ws) ws.onclose = null;
  ws?.close();
  ws = null;
  mock?.stop();
  mock = null;
  retry = 0;
}
