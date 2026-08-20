import { deflate } from 'pako';
import type { ServerMessage, RobotState, MapInfo } from '$lib/types/protocol';

/**
 * Standalone fleet simulator.
 * Lets the whole GUI run with no backend, no ROS, no Gazebo — used for UI work
 * and only runs when explicitly requested with `?mock=1`.
 */

const RES = 0.05;
const W = 800;
const H = 800;
const ORIGIN = { x: -20, y: -20 };
const REVEAL_R = 45; // cells

interface MockRobot {
  id: string;
  type: string;
  x: number;
  y: number;
  yaw: number;
  target: { x: number; y: number } | null;
  battery: number;
  quality: number;
  navStatus: RobotState['nav_status'];
  mode: RobotState['mode'];
  lastAttended: number;
  caps: RobotState['capabilities'];
}

/** Ground-truth world: outer walls, rooms, corridors. */
function buildTruth(): Int8Array {
  const t = new Int8Array(W * H).fill(0);
  const wall = (x0: number, y0: number, x1: number, y1: number) => {
    for (let y = y0; y <= y1; y++)
      for (let x = x0; x <= x1; x++) if (x >= 0 && x < W && y >= 0 && y < H) t[y * W + x] = 100;
  };
  wall(60, 60, 740, 68); // outer
  wall(60, 732, 740, 740);
  wall(60, 60, 68, 740);
  wall(732, 60, 740, 740);
  // interior rooms
  wall(250, 60, 258, 320);
  wall(250, 440, 258, 740);
  wall(500, 60, 508, 260);
  wall(500, 380, 508, 740);
  wall(258, 380, 500, 388);
  wall(508, 500, 740, 508);
  wall(60, 250, 180, 258);
  return t;
}

/** Mirrors the backend catalog; the mock never reaches `/api/detection/classes`. */
const MOCK_DETECTION_CLASSES = [
  'rubber_duck',
  'wooden_block',
  'disc_cone',
  'filament_spool',
  'pool_noodle'
];

export class MockFleet {
  private truth = buildTruth();
  private known = new Int8Array(W * H).fill(-1);
  private robots: MockRobot[] = [];
  private timers: number[] = [];
  private seq = 0;
  private t0 = performance.now();
  private detectionSeq = 0;
  private network = new Map<string, Uint8Array>();
  private networkDirty = new Map<string, { x0: number; y0: number; x1: number; y1: number }>();
  private networkSeq = new Map<string, number>();

  constructor(
    private emit: (m: ServerMessage) => void,
    private count = 4
  ) {}

  start() {
    const starts = [
      { x: -14, y: -14 },
      { x: 10, y: -14 },
      { x: -14, y: 10 },
      { x: 10, y: 10 },
      { x: 0, y: 0 }
    ];
    for (let i = 0; i < Math.min(this.count, 5); i++) {
      this.robots.push({
        id: `robot_${i}`,
        type: i === 0 ? 'spot' : 'diffdrive',
        x: starts[i].x,
        y: starts[i].y,
        yaw: Math.random() * Math.PI * 2,
        target: null,
        battery: 0.7 + Math.random() * 0.3,
        quality: 75,
        navStatus: 'idle',
        mode: 'idle',
        lastAttended: 0,
        caps: ['navigate', 'map', 'camera', 'battery', 'network', 'estop']
      });
      this.network.set(`robot_${i}`, new Uint8Array(W * H).fill(255));
    }

    const info: MapInfo = { resolution: RES, width: W, height: H, origin: ORIGIN, seq: 0 };
    this.emit({ type: 'map_info', info });

    this.timers.push(setInterval(() => this.step(0.2), 200) as unknown as number);
    this.timers.push(setInterval(() => this.pushPatch(), 600) as unknown as number);
    this.timers.push(setInterval(() => this.pushNetworkPatches(), 1000) as unknown as number);
    this.timers.push(setInterval(() => this.maybeDetect(), 5200) as unknown as number);

    this.emit({
      type: 'session_state',
      running: true,
      name: `mock_${this.count}robot`,
      started_at: Date.now() / 1000,
      elapsed_s: 0,
      recording: true
    });
  }

  stop() {
    this.timers.forEach(clearInterval);
    this.timers = [];
  }

  command(robotId: string, kind: string, payload?: { x: number; y: number }) {
    const r = this.robots.find((x) => x.id === robotId);
    if (!r) return;
    r.lastAttended = 0;
    if (kind === 'set_goal' && payload) {
      r.target = { ...payload };
      r.navStatus = 'active';
      r.mode = 'nav';
    } else if (kind === 'cancel_goal') {
      r.target = null;
      r.navStatus = 'cancelled';
      r.mode = 'idle';
    } else if (kind === 'stop') {
      r.target = null;
      r.navStatus = 'idle';
      r.mode = 'estop';
    }
  }

  stopAll() {
    this.robots.forEach((r) => {
      r.target = null;
      r.navStatus = 'idle';
      r.mode = 'estop';
    });
  }

  private step(dt: number) {
    const tMono = (performance.now() - this.t0) / 1000;
    for (const r of this.robots) {
      if (r.mode !== 'estop') {
        if (!r.target) {
          // idle wander so the map keeps growing without operator input
          if (Math.random() < 0.02) {
            r.target = { x: -16 + Math.random() * 32, y: -16 + Math.random() * 32 };
            r.navStatus = 'active';
            r.mode = 'nav';
          }
        } else {
          const dx = r.target.x - r.x;
          const dy = r.target.y - r.y;
          const d = Math.hypot(dx, dy);
          if (d < 0.25) {
            r.target = null;
            r.navStatus = 'succeeded';
            r.mode = 'idle';
          } else {
            const speed = 1.1;
            r.yaw = Math.atan2(dy, dx);
            r.x += (dx / d) * speed * dt;
            r.y += (dy / d) * speed * dt;
          }
        }
      }
      r.battery = Math.max(0.05, r.battery - dt * 0.00035);
      const apDistance = Math.hypot(r.x - 3, r.y + 2);
      r.quality = Math.max(4, Math.min(100, 98 - apDistance * 3.2 + Math.sin(tMono * 0.7 + r.x) * 4));
      r.lastAttended += dt;
      this.reveal(r.x, r.y);
      this.recordNetwork(r);

      this.emit({
        type: 'robot_state',
        robot_id: r.id,
        robot_type: r.type,
        t_mono: tMono,
        t_wall: Date.now() / 1000,
        t_sess: tMono,
        pose: { x: r.x, y: r.y, yaw: r.yaw },
        battery: r.battery,
        mode: r.mode,
        nav_status: r.navStatus,
        goal: r.target,
        planned_path: r.target ? [{ x: r.x, y: r.y }, r.target] : [],
        capabilities: r.caps,
        unattended_s: r.lastAttended,
        online: true,
        network: {
          interface: 'mock-wlan0',
          quality_pct: r.quality,
          rssi_dbm: -90 + r.quality * 0.5
        }
      });

      if (r.lastAttended > 45 && r.lastAttended < 45.3) {
        this.emit({
          type: 'alert',
          alert: {
            id: `unattended_${r.id}`,
            level: 'warn',
            kind: 'unattended',
            robot_id: r.id,
            message: `${r.id} unattended for 45 s`,
            t_wall: Date.now() / 1000,
            acknowledged: false
          }
        });
      }
    }
  }

  private recordNetwork(robot: MockRobot) {
    const grid = this.network.get(robot.id);
    if (!grid) return;
    const cx = Math.round((robot.x - ORIGIN.x) / RES);
    const cy = Math.round(H - (robot.y - ORIGIN.y) / RES);
    const radius = 14;
    const x0 = Math.max(0, cx - radius);
    const y0 = Math.max(0, cy - radius);
    const x1 = Math.min(W, cx + radius + 1);
    const y1 = Math.min(H, cy + radius + 1);
    for (let y = y0; y < y1; y++) {
      for (let x = x0; x < x1; x++) {
        if ((x - cx) ** 2 + (y - cy) ** 2 <= radius * radius) {
          grid[y * W + x] = Math.round(robot.quality);
        }
      }
    }
    const dirty = this.networkDirty.get(robot.id);
    this.networkDirty.set(
      robot.id,
      dirty
        ? {
            x0: Math.min(x0, dirty.x0),
            y0: Math.min(y0, dirty.y0),
            x1: Math.max(x1, dirty.x1),
            y1: Math.max(y1, dirty.y1)
          }
        : { x0, y0, x1, y1 }
    );
  }

  private pushNetworkPatches() {
    for (const [robotId, bounds] of this.networkDirty) {
      const grid = this.network.get(robotId);
      if (!grid) continue;
      const w = bounds.x1 - bounds.x0;
      const h = bounds.y1 - bounds.y0;
      const sub = new Uint8Array(w * h);
      for (let y = 0; y < h; y++) {
        sub.set(
          grid.subarray((bounds.y0 + y) * W + bounds.x0, (bounds.y0 + y) * W + bounds.x1),
          y * w
        );
      }
      const compressed = deflate(sub);
      let binary = '';
      for (let i = 0; i < compressed.length; i++) binary += String.fromCharCode(compressed[i]);
      const seq = (this.networkSeq.get(robotId) ?? 0) + 1;
      this.networkSeq.set(robotId, seq);
      this.emit({
        type: 'network_patch',
        robot_id: robotId,
        seq,
        resolution: RES,
        origin: ORIGIN,
        width: W,
        height: H,
        x0: bounds.x0,
        y0: bounds.y0,
        w,
        h,
        data: btoa(binary)
      });
    }
    this.networkDirty.clear();
  }

  /** Reveal ground truth within sensor radius, simulating SLAM. */
  private reveal(wx: number, wy: number) {
    const cx = Math.round((wx - ORIGIN.x) / RES);
    const cy = Math.round(H - (wy - ORIGIN.y) / RES);
    for (let y = cy - REVEAL_R; y <= cy + REVEAL_R; y++) {
      if (y < 0 || y >= H) continue;
      for (let x = cx - REVEAL_R; x <= cx + REVEAL_R; x++) {
        if (x < 0 || x >= W) continue;
        if ((x - cx) ** 2 + (y - cy) ** 2 > REVEAL_R * REVEAL_R) continue;
        this.known[y * W + x] = this.truth[y * W + x];
      }
    }
  }

  /** Push the bounding box around all robots as a patch. */
  private pushPatch() {
    if (!this.robots.length) return;
    let x0 = W,
      y0 = H,
      x1 = 0,
      y1 = 0;
    let anyInside = false;
    for (const r of this.robots) {
      const cx = Math.round((r.x - ORIGIN.x) / RES);
      const cy = Math.round(H - (r.y - ORIGIN.y) / RES);
      const rx0 = cx - REVEAL_R - 2;
      const ry0 = cy - REVEAL_R - 2;
      const rx1 = cx + REVEAL_R + 2;
      const ry1 = cy + REVEAL_R + 2;
      if (rx1 > 0 && rx0 < W && ry1 > 0 && ry0 < H) {
        x0 = Math.min(x0, Math.max(0, rx0));
        y0 = Math.min(y0, Math.max(0, ry0));
        x1 = Math.max(x1, Math.min(W, rx1));
        y1 = Math.max(y1, Math.min(H, ry1));
        anyInside = true;
      }
    }
    if (!anyInside) return;
    const w = x1 - x0;
    const h = y1 - y0;
    if (w <= 0 || h <= 0) return;

    const sub = new Int8Array(w * h);
    for (let y = 0; y < h; y++)
      for (let x = 0; x < w; x++) sub[y * w + x] = this.known[(y0 + y) * W + (x0 + x)];

    const deflated = deflate(new Uint8Array(sub.buffer));
    let bin = '';
    for (let i = 0; i < deflated.length; i++) bin += String.fromCharCode(deflated[i]);

    this.emit({
      type: 'map_patch',
      seq: ++this.seq,
      resolution: RES,
      origin: ORIGIN,
      x0,
      y0,
      w,
      h,
      data: btoa(bin)
    });
  }

  private maybeDetect() {
    if (!this.robots.length || Math.random() > 0.55) return;
    const r = this.robots[Math.floor(Math.random() * this.robots.length)];
    const now = Date.now() / 1000;
    const cls = MOCK_DETECTION_CLASSES[
      Math.floor(Math.random() * MOCK_DETECTION_CLASSES.length)
    ];
    const [bx, by, bw, bh] = [
      0.18 + Math.random() * 0.5,
      0.22 + Math.random() * 0.4,
      0.16,
      0.22
    ] as [number, number, number, number];
    const score = 0.72 + Math.random() * 0.26;
    this.emit({
      type: 'detection',
      detection: {
        id: `${cls}_${++this.detectionSeq}`,
        class: cls,
        score,
        // The mock emits each sighting once, so there is no earlier evidence
        // for this to exceed.
        best_score: score,
        robot_id: r.id,
        camera: 'front',
        bbox: [bx, by, bw, bh],
        // A lozenge rather than the box itself, so the overlay's mask path is
        // exercised by `?mock=1` instead of only against real hardware.
        polygon: [
          [bx + bw * 0.5, by],
          [bx + bw, by + bh * 0.35],
          [bx + bw * 0.72, by + bh],
          [bx + bw * 0.28, by + bh],
          [bx, by + bh * 0.35]
        ],
        map_position: { x: r.x + (Math.random() - 0.5) * 3, y: r.y + (Math.random() - 0.5) * 3 },
        first_seen: now,
        last_seen: now,
        observations: 1
      }
    });
  }
}
