/**
 * SwarmDeck protocol v1 — GUI side.
 * Mirrors adapters/protocol/README.md. Keep in sync with server/schemas/.
 */

export const PROTOCOL_VERSION = 1;

/**
 * `reset` is simulation-only: it means "teleport to spawn and forget the map",
 * which no physical robot can honour. The reset control is gated on some robot
 * advertising it, which is what keeps it off a hardware dashboard.
 */
export type Capability = 'navigate' | 'map' | 'camera' | 'battery' | 'estop' | 'reset' | 'body';
export type NavStatus = 'idle' | 'active' | 'succeeded' | 'failed' | 'cancelled';
// `recover` is the adapter reversing a robot out of a pose Nav2 could not plan
// from. It moves on its own, briefly and without an operator command, so it has
// to be a mode the GUI can name rather than an unexplained lurch.
export type RobotMode = 'idle' | 'nav' | 'teleop' | 'estop' | 'recover';
export type AlertLevel = 'info' | 'warn' | 'critical';

export interface Pose {
  x: number;
  y: number;
  yaw: number;
}

export interface Point {
  x: number;
  y: number;
}

export interface Stamps {
  t_mono: number;
  t_wall: number;
  t_sess: number;
}

export interface RobotState extends Stamps {
  type: 'robot_state';
  robot_id: string;
  robot_type: string;
  pose: Pose;
  battery: number | null;
  mode: RobotMode;
  nav_status: NavStatus;
  goal: Point | null;
  planned_path: Point[];
  capabilities: Capability[];
  unattended_s: number;
  online: boolean;
  /** Circumscribed chassis radius, metres, as declared by the adapter at `hello`. */
  footprint_radius?: number;
}

export interface Detection {
  id: string;
  class: string;
  score: number;
  /**
   * Strongest score this entity has ever produced. The backend judges the
   * operator's floor against this rather than `score`, so a marker sitting
   * near its floor does not blink as the model's confidence wanders.
   */
  best_score: number;
  /**
   * Below the operator's floor for this robot and class. The backend only
   * sends these on the transition, so that raising a floor can retract
   * markers already on the map and lowering it can bring them back.
   */
  hidden?: boolean;
  robot_id: string;
  camera: string;
  bbox: [number, number, number, number] | null;
  /** Normalized segmentation outline, when the detector produced a mask. */
  polygon: [number, number][] | null;
  map_position: Point | null;
  first_seen: number;
  last_seen: number;
  observations: number;
}

/** One entry of the detector's class catalog, from `/api/detection/classes`. */
export interface DetectionClass {
  name: string;
  label: string;
  /** Catalog default score floor. Operators can raise it per class in settings. */
  min_score?: number;
}

export interface MapInfo {
  resolution: number;
  width: number;
  height: number;
  origin: Point;
  seq: number;
}

export interface MapPatch {
  type: 'map_patch';
  seq: number;
  resolution: number;
  origin: Point;
  x0: number;
  y0: number;
  w: number;
  h: number;
  /** base64(zlib(int8[])) row-major, -1 unknown / 0 free / 100 occupied */
  data: string;
}

export interface MapRegistration {
  score: number;
  overlap: number;
  /** Runner-up translation peak over the best, at the winning yaw. */
  ratio: number;
  /** Best rival at a different rotation over the best; catches symmetric aliases. */
  yaw_ratio: number;
  /** Fraction of the smaller map's known area shared with the reference. */
  support: number;
  confident: boolean;
  /** Whether this robot is contributing to the merged map right now. */
  accepted: boolean;
  /**
   * Consecutive ambiguous results since the last decisive one. Non-zero with
   * `accepted` still true means the merge is being held on an older transform.
   */
  misses: number;
  rejection: string | null;
  dyaw_deg: number;
  /** True once accepted, when the search refines instead of sweeping all rotations. */
  locked: boolean;
}

/** One robot's view of a collaborative pose graph (Swarm-SLAM / cslam). */
export interface SlamGraph {
  keyframes: number;
  /** True once the collaborative back end has placed this robot in the common frame. */
  in_common_frame: boolean;
  /** Optimiser residual, or null if the back end does not report one. */
  residual: number | null;
  /** Loop closures against each other robot — the thing that makes it swarm SLAM. */
  inter_robot: { other: string; count: number; last_t?: number }[];
  /**
   * This robot's SLAM frame expressed in the collaborative back end's common
   * frame. In `cslam` merge mode this REPLACES grid registration as the source
   * of the merge transform — it falls out of the loop closures rather than
   * being re-estimated from finished maps. `frame` names the cluster: robots
   * that have never met report different frames and must not be overlaid.
   */
  origin?: { x: number; y: number; yaw: number; frame?: string | null };
  t_mono?: number;
}

/**
 * How far independent grid correlation disagrees with the pose graph's
 * alignment. In `cslam` mode registration no longer produces the transform, so
 * what it reports is a cross-check drawn from evidence the loop closures did
 * not use. `confident: false` means the correlation could not separate rival
 * hypotheses — indicative, not a verdict.
 */
export interface CslamDisagreement {
  metres: number;
  degrees: number;
  confident: boolean;
}

export interface MapStatus {
  mode: 'static' | 'auto' | 'cslam';
  reference: string | null;
  transforms: Record<string, Pose>;
  registrations: Record<string, MapRegistration>;
  global_members: string[];
  view_by_robot: Record<string, 'global' | 'local'>;
  slam_graphs?: Record<string, SlamGraph>;
  cslam_disagreement?: Record<string, CslamDisagreement>;
}

export interface RobotConnectionSettings {
  id: string;
  enabled: boolean;
  type: string;
  endpoint: string;
  color?: string;
}

export interface AppSettings {
  unattended_threshold_s: number;
  /** After clearing an alert, same id stays suppressed for this many seconds. */
  alert_suppress_s: number;
  robot_count: number;
  detection_enabled: boolean;
  detection_sensitivity: number;
  /** Catalog class names the detector should look for. */
  detection_classes: string[];
  /**
   * Fleet-wide per-class minimum model score, defaulting to each class's
   * catalog floor. Enforced by the backend against stored detections, so a
   * change here applies immediately and retroactively rather than waiting for
   * the robots to notice.
   */
  detection_class_floors: Record<string, number>;
  /**
   * Sparse per-robot overrides of the above, keyed by robot id then class.
   * A robot with no entry follows the fleet value, including later changes
   * to it — so only classes an operator actually moved are stored.
   */
  detection_robot_floors: Record<string, Record<string, number>>;
  /**
   * Derived by the backend, never set here: the lowest floor any robot needs,
   * which is what the detectors actually capture at. Read-only for the UI.
   */
  detection_capture_floors: Record<string, number>;
  robots: RobotConnectionSettings[];
}

export interface Alert {
  id: string;
  level: AlertLevel;
  kind: 'unattended' | 'nav_failure' | 'adapter_disconnect' | 'stream_loss' | 'fault';
  robot_id: string | null;
  message: string;
  t_wall: number;
  acknowledged: boolean;
}

export interface SessionState {
  type: 'session_state';
  running: boolean;
  name: string | null;
  started_at: number | null;
  elapsed_s: number;
  recording: boolean;
}

/**
 * Progress of a simulation reset, broadcast to every client rather than returned
 * to the one that asked — a reset changes what everyone is looking at.
 *
 * `phase: 'done'` distinguishes three ways a robot can fail to reset, because
 * they need different responses: `unreachable` never got the command,
 * `no_response` got it and went quiet, and `partial` answered with the steps
 * that failed. Any robot in `failed` will push its old map back within seconds.
 */
export interface SimReset {
  type: 'sim_reset';
  phase: 'start' | 'done';
  skipped: string[];
  /** Robots the reset was addressed to. Present on `start`. */
  robots?: string[];
  ok?: boolean;
  reset?: string[];
  unreachable?: string[];
  no_response?: string[];
  partial?: Record<string, Record<string, boolean>>;
  failed?: string[];
  timed_out?: boolean;
}

/* ---------- server → GUI ---------- */

export type ServerMessage =
  | RobotState
  | MapPatch
  | SessionState
  | { type: 'fleet_change'; robots: RobotState[] }
  | { type: 'detection'; detection: Detection }
  | { type: 'alert'; alert: Alert }
  | { type: 'alert_clear'; id: string }
  | { type: 'settings_state'; settings: AppSettings }
  | { type: 'slam_graph'; robot_id: string; graph: SlamGraph }
  | SimReset
  | { type: 'map_info'; info: MapInfo };

/* ---------- GUI → server ---------- */

export type ClientMessage =
  | { type: 'set_goal'; robot_id: string; payload: Point }
  | { type: 'cancel_goal'; robot_id: string }
  | { type: 'drive'; robot_id: string; payload: { linear: number; angular: number } }
  | { type: 'select_robots'; robot_ids: string[] }
  | { type: 'switch_camera'; robot_id: string }
  | { type: 'acknowledge_alert'; id: string }
  | { type: 'report_target'; robot_id: string; payload: Point }
  | { type: 'stop_all' }
  | { type: 'reset_sim' }
  | { type: 'body_command'; robot_id: string; action: 'claim' | 'release' | 'sit' | 'stand' };

export type ClientAction = ClientMessage['type'];
