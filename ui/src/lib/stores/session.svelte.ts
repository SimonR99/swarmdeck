import type { Alert, Detection } from '$lib/types/protocol';

type ConnState = 'connecting' | 'live' | 'mock' | 'lost';

const state = $state({
  connection: 'connecting' as ConnState,
  running: false,
  name: null as string | null,
  elapsed_s: 0,
  recording: false,
  alerts: [] as Alert[],
  detections: [] as Detection[]
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

  setConnection(c: ConnState) {
    state.connection = c;
  },

  setSession(s: { running: boolean; name: string | null; elapsed_s: number; recording: boolean }) {
    state.running = s.running;
    state.name = s.name;
    state.elapsed_s = s.elapsed_s;
    state.recording = s.recording;
  },

  tick(dt: number) {
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
    const i = state.detections.findIndex((x) => x.id === d.id);
    if (i >= 0) state.detections[i] = d;
    else state.detections = [...state.detections, d];
  },

  /** Detections currently visible in a given robot's camera frame. */
  bboxesFor(robotId: string, withinMs = 2000): Detection[] {
    const now = Date.now() / 1000;
    return state.detections.filter(
      (d) => d.robot_id === robotId && d.bbox && now - d.last_seen < withinMs / 1000
    );
  },

  reset() {
    state.alerts = [];
    state.detections = [];
    state.running = false;
    state.elapsed_s = 0;
  }
};
