import type { Alert, Detection, SimReset } from '$lib/types/protocol';

type ConnState = 'connecting' | 'live' | 'mock' | 'lost';

/**
 * A detection plus the browser-clock instant it arrived.
 *
 * Freshness is judged against this rather than the protocol's `last_seen`,
 * which is stamped from the SERVER's clock. An operator watching a remote fleet
 * shares no clock with it, and a few seconds of skew either hides every box or
 * makes them all immortal.
 */
type TrackedDetection = Detection & { received_at: number };

// Kept outside the reactive state because these values gate writes; rendering
// only depends on the filtered `detections` array. `null` is the compatibility
// mode used when the backend has no detector catalog and therefore accepts the
// adapter's own class list.
let detectionEnabled = true;
let selectedDetectionClasses: Set<string> | null = null;

const state = $state({
  connection: 'connecting' as ConnState,
  running: false,
  name: null as string | null,
  elapsed_s: 0,
  recording: false,
  alerts: [] as Alert[],
  detections: [] as TrackedDetection[],
  // Browser clock, republished once a second so that time-windowed reads
  // (`bboxesFor`) have something that changes to depend on.
  now: Date.now() / 1000,
  // True between a reset's `start` and `done` broadcasts. Driven by the server
  // rather than set optimistically on click, so every operator watching the same
  // fleet sees the reset, not just the one who pressed the button.
  resetting: false,
  lastReset: null as SimReset | null
});

export const session = {
  get connection() {
    return state.connection;
  },
  get running() {
    return state.running;
  },
  get name() {
    return state.name;
  },
  get elapsed_s() {
    return state.elapsed_s;
  },
  get recording() {
    return state.recording;
  },
  get alerts() {
    return state.alerts.filter((a) => !a.acknowledged);
  },
  get allAlerts() {
    return state.alerts;
  },
  get detections() {
    return state.detections;
  },
  get criticalCount() {
    return state.alerts.filter((a) => !a.acknowledged && a.level === 'critical').length;
  },
  get resetting() {
    return state.resetting;
  },
  get lastReset() {
    return state.lastReset;
  },

  applySimReset(msg: SimReset) {
    state.resetting = msg.phase === 'start';
    if (msg.phase !== 'done') return;
    state.lastReset = msg;
    // Detections describe a world that has just been put back to the start, so
    // they go with it. Alerts are NOT cleared here: the server clears them as
    // part of the reset and then re-raises anything still true, so dropping them
    // locally would race that and could hide a fresh warning about the reset
    // itself.
    state.detections = [];
  },

  setConnection(c: ConnState) {
    state.connection = c;
    // A reset in progress cannot be observed across a dropped socket: the
    // `done` broadcast that would clear this went to a connection that no
    // longer exists. Left set, it disables the button until a page reload.
    if (c !== 'live') state.resetting = false;
  },

  setSession(s: { running: boolean; name: string | null; elapsed_s: number; recording: boolean }) {
    state.running = s.running;
    state.name = s.name;
    state.elapsed_s = s.elapsed_s;
    state.recording = s.recording;
  },

  tick(dt: number) {
    // Unconditional, unlike the session timer below: detection boxes have to
    // expire whether or not a run is in progress.
    state.now = Date.now() / 1000;
    if (state.running) state.elapsed_s += dt;
  },

  addAlert(a: Alert) {
    const existing = state.alerts.findIndex((x) => x.id === a.id);
    if (existing >= 0) state.alerts[existing] = a;
    else state.alerts = [a, ...state.alerts].slice(0, 50);
  },

  clearAlert(id: string) {
    state.alerts = state.alerts.filter((a) => a.id !== id);
  },

  acknowledge(id: string) {
    const a = state.alerts.find((x) => x.id === id);
    if (a) a.acknowledged = true;
  },

  addDetection(d: Detection) {
    // A detection broadcast that was already in flight can arrive just after
    // the settings broadcast which removed its category. Do not let that old
    // message recreate the entity the settings reconciliation just deleted.
    if (!detectionEnabled || (selectedDetectionClasses && !selectedDetectionClasses.has(d.class))) {
      return;
    }
    const tracked: TrackedDetection = { ...d, received_at: Date.now() / 1000 };
    const i = state.detections.findIndex((x) => x.id === d.id);
    if (i >= 0) state.detections[i] = tracked;
    else state.detections = [...state.detections, tracked];
  },

  /**
   * Reconcile persistent map entities with the operator's detector selection.
   *
   * Camera boxes naturally expire, but map positions deliberately do not. A
   * settings save therefore has to delete entities in categories that are no
   * longer selected; merely hiding them would make the old positions reappear
   * if that category were enabled again later.
   */
  applyDetectionSettings(enabled: boolean, classes: string[]) {
    detectionEnabled = enabled;
    selectedDetectionClasses = classes.length ? new Set(classes) : null;
    if (!enabled) {
      state.detections = [];
      return;
    }
    // An empty list is the backend's compatibility mode when no class catalog
    // is available. In that case the adapter owns the allowed class set.
    if (!classes.length) return;
    state.detections = state.detections.filter((d) => selectedDetectionClasses!.has(d.class));
  },

  /**
   * Detections currently visible in a given robot's camera frame.
   *
   * The server retracts a box the moment its camera stops reporting the object,
   * so this window is the backstop for the case it cannot cover: an adapter
   * that went silent, whose last batch is therefore never superseded. Reading
   * `state.now` is what makes that backstop work — a `$derived` built on this
   * has nothing else to invalidate it once the messages stop, and would hold
   * the last rectangle on screen indefinitely.
   */
  bboxesFor(robotId: string, withinMs = 2000): Detection[] {
    return state.detections.filter(
      (d) => d.robot_id === robotId && d.bbox && state.now - d.received_at < withinMs / 1000
    );
  },

  reset() {
    state.alerts = [];
    state.detections = [];
    state.running = false;
    state.elapsed_s = 0;
    state.resetting = false;
    state.lastReset = null;
  }
};
