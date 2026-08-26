/* Canvas layers for the 2D map.

MapView owns interaction state and the Svelte lifecycle.  The drawing code is
kept here as a small renderer boundary so adding a visual layer does not also
make pointer handling, map selection, and reset controls harder to navigate.
The functions intentionally receive the current viewport and layer toggles;
they remain stateless apart from the trail history supplied by the component. */

import { fleet } from '$lib/stores/fleet.svelte';
import { mapStore } from '$lib/stores/mapstore.svelte';
import { detectionCatalog } from '$lib/stores/detection.svelte';
import { review } from '$lib/stores/review.svelte';
import { robotDisplayName } from '$lib/robotDisplayName';
import type { DetectionEntity, DetectionProposal, Footprint } from '$lib/types/protocol';

export type ScreenPoint = { sx: number; sy: number };
export type GridPoint = { gx: number; gy: number };
export type Viewport = { scale: number; tx: number; ty: number; rotation?: number };
export type ScreenOf = (gx: number, gy: number) => ScreenPoint;

export interface MapRobot {
  robot_id: string;
  robot_type?: string;
  pose: { x: number; y: number; yaw: number };
  planned_path?: { x: number; y: number }[];
  global_planned_path?: { x: number; y: number }[];
  local_planned_path?: { x: number; y: number }[];
  footprint_radius?: number;
  footprint?: Footprint | null;
  goal?: { x: number; y: number } | null;
}

// Keep the map useful while an older adapter is reconnecting and has not yet
// sent the optional polygon in `hello`. These are the same base-frame polygons
// used by the hardware adapter/Nav2 profiles; a missing live declaration must
// never make a known square robot look circular.
const KNOWN_FOOTPRINTS: Record<string, Footprint> = {
  agilex_bunker: [
    [0.362, 0.389],
    [0.362, -0.389],
    [-0.662, -0.389],
    [-0.662, 0.389]
  ],
  boston_dynamics_spot: [
    [0.55, 0.25],
    [0.55, -0.25],
    [-0.55, -0.25],
    [-0.55, 0.25]
  ],
  scout_mini: [
    [0.31, 0.293],
    [0.31, -0.293],
    [-0.31, -0.293],
    [-0.31, 0.293]
  ]
};

export interface MapInfo {
  width: number;
  height: number;
  resolution: number;
  origin: { x: number; y: number };
}

export function drawMetricGrid(
  ctx: CanvasRenderingContext2D,
  width: number,
  height: number,
  info: MapInfo | null,
  view: Viewport,
  screenOf: ScreenOf,
  showGrid: boolean
) {
  if (!showGrid) return;
  const resolution = info?.resolution ?? 0.05;
  const originX = info?.origin.x ?? 0;
  const originY = info?.origin.y ?? 0;
  const gridHeight = info?.height ?? 0;

  const pixelsPerMetre = view.scale / resolution;
  const spacing = pixelsPerMetre >= 24 ? 1 : pixelsPerMetre >= 7 ? 5 : 10;

  // Calculate visible world bounds from viewport corners
  const minWorldX = originX + ((0 - view.tx) / view.scale) * resolution;
  const maxWorldX = originX + ((width - view.tx) / view.scale) * resolution;
  const minWorldY = originY + (gridHeight - ((height - view.ty) / view.scale)) * resolution;
  const maxWorldY = originY + (gridHeight - ((0 - view.ty) / view.scale)) * resolution;

  const startX = Math.floor(Math.min(minWorldX, maxWorldX) / spacing) * spacing;
  const endX = Math.ceil(Math.max(minWorldX, maxWorldX) / spacing) * spacing;
  const startY = Math.floor(Math.min(minWorldY, maxWorldY) / spacing) * spacing;
  const endY = Math.ceil(Math.max(minWorldY, maxWorldY) / spacing) * spacing;

  ctx.save();
  ctx.lineWidth = 1;
  ctx.strokeStyle = 'rgba(60, 68, 80, 0.12)';
  ctx.beginPath();

  // Vertical lines (constant X)
  for (let x = startX; x <= endX; x += spacing) {
    const gx = (x - originX) / resolution;
    const sx = view.tx + gx * view.scale;
    if (sx >= -1 && sx <= width + 1) {
      ctx.moveTo(sx, 0);
      ctx.lineTo(sx, height);
    }
  }

  // Horizontal lines (constant Y)
  for (let y = startY; y <= endY; y += spacing) {
    const gy = gridHeight - (y - originY) / resolution;
    const sy = view.ty + gy * view.scale;
    if (sy >= -1 && sy <= height + 1) {
      ctx.moveTo(0, sy);
      ctx.lineTo(width, sy);
    }
  }
  ctx.stroke();

  // Origin axes crosshairs at world (0, 0)
  const originGx = (0 - originX) / resolution;
  const originGy = gridHeight - (0 - originY) / resolution;
  const originSx = view.tx + originGx * view.scale;
  const originSy = view.ty + originGy * view.scale;

  ctx.strokeStyle = 'rgba(11, 92, 173, 0.35)';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  if (originSx >= 0 && originSx <= width) {
    ctx.moveTo(originSx, 0);
    ctx.lineTo(originSx, height);
  }
  if (originSy >= 0 && originSy <= height) {
    ctx.moveTo(0, originSy);
    ctx.lineTo(width, originSy);
  }
  ctx.stroke();

  ctx.restore();
}

function reviewVisible(robotIds: string[]): boolean {
  const enabled = robotIds.filter((id) => fleet.isEnabled(id));
  if (!enabled.length) return false;
  if (mapStore.viewMode === 'local') return enabled.includes(mapStore.viewRobot ?? '');
  return true;
}

export interface ReviewedObjectHit {
  id: string;
  kind: 'entity' | 'proposal';
  robotIds: string[];
  /** The robot that most recently contributed a retained sample, when known. */
  robotId: string | null;
  distance: number;
}

function mostRecentRobot(object: DetectionEntity | DetectionProposal): string | null {
  if ('samples' in object && object.samples.length) {
    return object.samples.reduce((latest, sample) =>
      sample.t > latest.t ? sample : latest
    ).robot_id;
  }
  return object.robot_ids[0] ?? null;
}

/** Find the reviewed map marker under a click, using the same world transform as drawing. */
export function hitTestReviewedObject(
  sx: number,
  sy: number,
  screenOf: ScreenOf,
  maxDistance = 18
): ReviewedObjectHit | null {
  let nearest: ReviewedObjectHit | null = null;

  const consider = (
    object: DetectionEntity | DetectionProposal,
    kind: ReviewedObjectHit['kind']
  ) => {
    if (!reviewVisible(object.robot_ids)) return;
    const grid = mapStore.worldToGrid(object.position.x, object.position.y);
    if (!grid) return;
    const screen = screenOf(grid.gx, grid.gy);
    const distance = Math.hypot(screen.sx - sx, screen.sy - sy);
    if (distance > maxDistance || (nearest && distance >= nearest.distance)) return;
    nearest = {
      id: object.id,
      kind,
      robotIds: object.robot_ids,
      robotId: mostRecentRobot(object),
      distance
    };
  };

  // Proposals are actionable notifications, so inspect them first when two
  // markers overlap exactly. A confirmed entity still wins when it is closer.
  for (const proposal of review.proposals) consider(proposal, 'proposal');
  for (const entity of review.entities) consider(entity, 'entity');
  return nearest;
}

/** Draw operator-reviewed detections over the occupancy layer. */
export function drawReviewedObjects(ctx: CanvasRenderingContext2D, screenOf: ScreenOf) {
  ctx.save();

  // Draw unselected proposals first, then selected on top
  const proposals = [
    ...review.proposals.filter((p) => review.highlighted !== p.id && review.selected !== p.id),
    ...review.proposals.filter((p) => review.highlighted === p.id || review.selected === p.id)
  ];

  ctx.setLineDash([3, 3]);
  for (const proposal of proposals) {
    if (!reviewVisible(proposal.robot_ids)) continue;
    const g = mapStore.worldToGrid(proposal.position.x, proposal.position.y);
    if (!g) continue;
    const { sx, sy } = screenOf(g.gx, g.gy);
    const color = detectionCatalog.colorOf(proposal.class);
    const focused = review.highlighted === proposal.id || review.selected === proposal.id;
    ctx.globalAlpha = focused ? 1 : 0.55;
    ctx.beginPath();
    ctx.arc(sx, sy, focused ? 13 : 8, 0, Math.PI * 2);
    ctx.lineWidth = focused ? 2.5 : 1.5;
    ctx.strokeStyle = color;
    ctx.stroke();
    ctx.globalAlpha = focused ? 0.35 : 0.12;
    ctx.fillStyle = color;
    ctx.fill();
    ctx.globalAlpha = 1;

    if (focused) {
      const labelText = `${detectionCatalog.labelOf(proposal.class)} · ${Math.round(proposal.best_score * 100)}%`;
      ctx.font = '600 11px ui-sans-serif, system-ui';
      const textMetrics = ctx.measureText(labelText);
      const pillW = textMetrics.width + 16;
      const pillH = 22;
      const pillX = sx - pillW / 2;
      const pillY = sy - 30;

      ctx.fillStyle = 'rgba(25, 32, 42, 0.92)';
      ctx.beginPath();
      if (typeof ctx.roundRect === 'function') {
        ctx.roundRect(pillX, pillY, pillW, pillH, 11);
      } else {
        ctx.rect(pillX, pillY, pillW, pillH);
      }
      ctx.fill();
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.stroke();

      ctx.fillStyle = '#ffffff';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(labelText, sx, pillY + pillH / 2);
    }
  }

  // Draw unselected entities first, then selected on top
  const entities = [
    ...review.entities.filter((e) => review.highlighted !== e.id && review.selected !== e.id),
    ...review.entities.filter((e) => review.highlighted === e.id || review.selected === e.id)
  ];

  ctx.setLineDash([]);
  for (const entity of entities) {
    if (!reviewVisible(entity.robot_ids)) continue;
    const g = mapStore.worldToGrid(entity.position.x, entity.position.y);
    if (!g) continue;
    const { sx, sy } = screenOf(g.gx, g.gy);
    const color = detectionCatalog.colorOf(entity.class);
    const focused = review.highlighted === entity.id || review.selected === entity.id;

    ctx.beginPath();
    ctx.arc(sx, sy, focused ? 9 : 6, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.fill();
    ctx.lineWidth = focused ? 2.5 : 2;
    ctx.strokeStyle = '#ffffff';
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(sx, sy, focused ? 14 : 8.5, 0, Math.PI * 2);
    ctx.lineWidth = focused ? 2.5 : 1.5;
    ctx.strokeStyle = color;
    ctx.stroke();

    if (focused) {
      const labelText = detectionCatalog.labelOf(entity.class);
      ctx.font = '600 11px ui-sans-serif, system-ui';
      const textMetrics = ctx.measureText(labelText);
      const pillW = textMetrics.width + 16;
      const pillH = 22;
      const pillX = sx - pillW / 2;
      const pillY = sy - 30;

      ctx.fillStyle = 'rgba(25, 32, 42, 0.92)';
      ctx.beginPath();
      if (typeof ctx.roundRect === 'function') {
        ctx.roundRect(pillX, pillY, pillW, pillH, 11);
      } else {
        ctx.rect(pillX, pillY, pillW, pillH);
      }
      ctx.fill();
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.stroke();

      ctx.fillStyle = '#ffffff';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(labelText, sx, pillY + pillH / 2);
    }
  }
  ctx.restore();
}

/** Draw inter-robot loop closures once per pair. */
export function drawLoopClosures(
  ctx: CanvasRenderingContext2D,
  screenOf: ScreenOf,
  showPlans: boolean
) {
  if (!showPlans) return;
  const seen = new Set<string>();
  ctx.save();
  ctx.setLineDash([1, 3]);
  ctx.lineWidth = 1.5;
  for (const [robotId, graph] of Object.entries(mapStore.slamGraphs)) {
    const a = fleet.get(robotId);
    if (!a) continue;
    for (const link of graph.inter_robot) {
      const key = [robotId, link.other].sort().join('|');
      if (seen.has(key)) continue;
      seen.add(key);
      const b = fleet.get(link.other);
      if (!b) continue;
      const ga = mapStore.worldToGrid(a.pose.x, a.pose.y);
      const gb = mapStore.worldToGrid(b.pose.x, b.pose.y);
      if (!ga || !gb) continue;
      const pa = screenOf(ga.gx, ga.gy);
      const pb = screenOf(gb.gx, gb.gy);
      ctx.beginPath();
      ctx.moveTo(pa.sx, pa.sy);
      ctx.lineTo(pb.sx, pb.sy);
      ctx.globalAlpha = Math.min(0.75, 0.25 + link.count * 0.06);
      ctx.strokeStyle = '#0b5cad';
      ctx.stroke();
    }
  }
  ctx.restore();
  ctx.globalAlpha = 1;
  ctx.setLineDash([]);
}

export function drawNetworkHeatmap(
  ctx: CanvasRenderingContext2D,
  screenOf: ScreenOf,
  view: Viewport,
  showNetwork: boolean
) {
  const layer = mapStore.networkLayer;
  if (
    !showNetwork ||
    mapStore.viewMode !== 'local' ||
    !layer ||
    layer.robotId !== mapStore.viewRobot
  ) return;

  const maxY = layer.info.origin.y + layer.info.height * layer.info.resolution;
  const topLeft = mapStore.viewToGrid(layer.info.origin.x, maxY);
  if (!topLeft) return;
  const screen = screenOf(topLeft.gx, topLeft.gy);
  const width =
    (layer.info.width * layer.info.resolution / (mapStore.info?.resolution ?? 1)) * view.scale;
  const height =
    (layer.info.height * layer.info.resolution / (mapStore.info?.resolution ?? 1)) * view.scale;
  ctx.save();
  ctx.globalAlpha = 0.62;
  ctx.imageSmoothingEnabled = true;
  ctx.drawImage(layer.canvas, screen.sx, screen.sy, width, height);
  ctx.restore();
}

export interface RobotLayerOptions {
  info: MapInfo;
  view: Viewport;
  screenOf: ScreenOf;
  trails: Map<string, { x: number; y: number }[]>;
  showTrails: boolean;
  showPlans: boolean;
  showSensors: boolean;
  showLabels: boolean;
}

/** Transform an adapter-declared base-frame polygon into map pixels. */
function footprintOnScreen(robot: MapRobot, screenOf: ScreenOf): ScreenPoint[] | null {
  let footprint = robot.footprint;
  if (!footprint || footprint.length < 3) {
    footprint = KNOWN_FOOTPRINTS[robot.robot_type ?? ''];
  }
  if (!footprint || footprint.length < 3) {
    // Unknown/legacy robots still get a square conservative marker. The
    // declared polygon remains authoritative whenever one is available.
    const radius = Math.max(0.05, robot.footprint_radius ?? 0.42);
    const halfSide = radius / Math.SQRT2;
    footprint = [
      [halfSide, halfSide],
      [halfSide, -halfSide],
      [-halfSide, -halfSide],
      [-halfSide, halfSide]
    ];
  }
  const c = Math.cos(robot.pose.yaw);
  const s = Math.sin(robot.pose.yaw);
  const points: ScreenPoint[] = [];
  for (const [x, y] of footprint) {
    const grid = mapStore.worldToGrid(
      robot.pose.x + c * x - s * y,
      robot.pose.y + s * x + c * y
    );
    if (!grid) return null;
    points.push(screenOf(grid.gx, grid.gy));
  }
  return points;
}

/** Draw trails, plans, sensors, goals, selection halos, bodies, and labels. */
export function drawRobots(
  ctx: CanvasRenderingContext2D,
  robots: MapRobot[],
  options: RobotLayerOptions
) {
  const { info, view, screenOf, trails, showTrails, showPlans, showSensors, showLabels } = options;
  for (const robot of robots) {
    const color = fleet.colorOf(robot.robot_id);
    const g = mapStore.worldToGrid(robot.pose.x, robot.pose.y);
    if (!g) continue;
    const { sx, sy } = screenOf(g.gx, g.gy);

    let trail = trails.get(robot.robot_id);
    if (!trail) trails.set(robot.robot_id, (trail = []));
    const last = trail[trail.length - 1];
    const distFromLast = last ? Math.hypot(last.x - robot.pose.x, last.y - robot.pose.y) : 0;
    if (distFromLast > 3.0) {
      trail.length = 0;
    }
    if (!last || distFromLast > 0.08) {
      trail.push({ x: robot.pose.x, y: robot.pose.y });
      if (trail.length > 600) trail.shift();
    }
    if (showTrails && trail.length > 1) {
      ctx.beginPath();
      let started = false;
      let prevPt: { x: number; y: number } | null = null;
      for (const point of trail) {
        if (prevPt && Math.hypot(point.x - prevPt.x, point.y - prevPt.y) > 2.0) {
          started = false;
        }
        prevPt = point;
        const ptGrid = mapStore.worldToGrid(point.x, point.y);
        if (!ptGrid) continue;
        const p = screenOf(ptGrid.gx, ptGrid.gy);
        if (!started) {
          ctx.moveTo(p.sx, p.sy);
          started = true;
        } else {
          ctx.lineTo(p.sx, p.sy);
        }
      }
      if (started) {
        ctx.strokeStyle = color;
        ctx.globalAlpha = 0.5;
        ctx.lineWidth = 2.5;
        ctx.stroke();
        ctx.globalAlpha = 1;
      }
    }

    const isNavActive =
      robot.nav_status === 'active' ||
      robot.nav_status === 'nav' ||
      robot.mode === 'nav' ||
      Boolean(robot.goal);

    const hasSplitPaths =
      Boolean((robot.global_planned_path && robot.global_planned_path.length > 0) ||
      (robot.local_planned_path && robot.local_planned_path.length > 0));
    const globalPath = isNavActive && hasSplitPaths
      ? (robot.global_planned_path && robot.global_planned_path.length > 0 ? robot.global_planned_path : robot.planned_path)
      : isNavActive ? robot.planned_path : undefined;
    const localPath = isNavActive && robot.local_planned_path && robot.local_planned_path.length > 0 ? robot.local_planned_path : undefined;

    const drawPath = (
      path: { x: number; y: number }[] | undefined,
      dash: number[],
      alpha: number,
      lineWidth: number
    ) => {
      if (!showPlans || !isNavActive || !path || path.length < 2) return false;
      const visiblePath = path
        .map((point) => mapStore.worldToGrid(point.x, point.y))
        .filter((point): point is GridPoint => point !== null);
      if (visiblePath.length < 2) return false;

      // Dark halo for unmistakable contrast over occupied/free/unknown areas
      ctx.beginPath();
      const first = screenOf(visiblePath[0].gx, visiblePath[0].gy);
      ctx.moveTo(first.sx, first.sy);
      for (const gridPoint of visiblePath.slice(1)) {
        const p = screenOf(gridPoint.gx, gridPoint.gy);
        ctx.lineTo(p.sx, p.sy);
      }
      ctx.setLineDash([]);
      ctx.strokeStyle = '#000000';
      ctx.globalAlpha = 0.5;
      ctx.lineWidth = lineWidth + 2.5;
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';
      ctx.stroke();

      // Colored route stroke
      ctx.beginPath();
      ctx.moveTo(first.sx, first.sy);
      for (const gridPoint of visiblePath.slice(1)) {
        const p = screenOf(gridPoint.gx, gridPoint.gy);
        ctx.lineTo(p.sx, p.sy);
      }
      ctx.setLineDash(dash);
      ctx.strokeStyle = color;
      ctx.globalAlpha = alpha;
      ctx.lineWidth = lineWidth;
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.globalAlpha = 1;
      return true;
    };

    // Global planner route: thick and bright with halo. Local controller trajectory:
    // solid and vibrant on top.
    const globalPathVisible = drawPath(globalPath, [8, 5], 0.9, 3.5);
    const localPathVisible = drawPath(localPath, [], 1.0, 4.0);
    const anyPathVisible = globalPathVisible || localPathVisible;

    if (showSensors) {
      const sensorPx = (2.0 / info.resolution) * view.scale;
      ctx.save();
      ctx.translate(sx, sy);
      ctx.rotate(-mapStore.worldYawToView(robot.pose.yaw) + (view.rotation ?? 0));
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.arc(0, 0, sensorPx, -0.6, 0.6);
      ctx.closePath();
      ctx.fillStyle = color;
      ctx.globalAlpha = 0.055;
      ctx.fill();
      ctx.globalAlpha = 0.45;
      ctx.strokeStyle = color;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(Math.min(sensorPx, 54), 0);
      ctx.stroke();
      ctx.restore();

      const polygon = footprintOnScreen(robot, screenOf);
      ctx.setLineDash([3, 3]);
      ctx.strokeStyle = color;
      ctx.globalAlpha = 0.32;
      ctx.lineWidth = 1;
      ctx.beginPath();
      if (polygon) {
        ctx.moveTo(polygon[0].sx, polygon[0].sy);
        for (const point of polygon.slice(1)) ctx.lineTo(point.sx, point.sy);
        ctx.closePath();
        ctx.fillStyle = color;
        ctx.globalAlpha = 0.06;
        ctx.fill();
        ctx.globalAlpha = 0.55;
        ctx.stroke();
      }
      ctx.setLineDash([]);
      ctx.globalAlpha = 1;
    }

    if (showPlans && isNavActive && robot.goal) {
      const goal = mapStore.worldToGrid(robot.goal.x, robot.goal.y);
      if (goal) {
        const s = screenOf(goal.gx, goal.gy);
        // Check if the planner's global route already reaches the goal.
        // If not (e.g. on robots running only local reactive avoidance like Scout,
        // or while the planner is still computing), draw the route guide line to the goal.
        const lastPt = globalPath && globalPath.length > 0 ? globalPath[globalPath.length - 1] : null;
        const reachesGoal = Boolean(
          globalPathVisible &&
          lastPt &&
          Math.hypot(lastPt.x - robot.goal.x, lastPt.y - robot.goal.y) < 1.0
        );

        if (!reachesGoal) {
          ctx.beginPath();
          ctx.moveTo(sx, sy);
          ctx.lineTo(s.sx, s.sy);
          ctx.strokeStyle = '#000000';
          ctx.globalAlpha = 0.4;
          ctx.lineWidth = 4.0;
          ctx.stroke();

          ctx.setLineDash([6, 4]);
          ctx.beginPath();
          ctx.moveTo(sx, sy);
          ctx.lineTo(s.sx, s.sy);
          ctx.strokeStyle = color;
          ctx.globalAlpha = 0.85;
          ctx.lineWidth = 2.5;
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.globalAlpha = 1;
        }

        ctx.beginPath();
        ctx.arc(s.sx, s.sy, 6, 0, Math.PI * 2);
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(s.sx - 9, s.sy);
        ctx.lineTo(s.sx + 9, s.sy);
        ctx.moveTo(s.sx, s.sy - 9);
        ctx.lineTo(s.sx, s.sy + 9);
        ctx.globalAlpha = 0.6;
        ctx.stroke();
        ctx.globalAlpha = 1;
      }
    }

    if (fleet.isSelected(robot.robot_id)) {
      ctx.beginPath();
      ctx.arc(sx, sy, 16, 0, Math.PI * 2);
      ctx.strokeStyle = color;
      ctx.globalAlpha = 0.45;
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.globalAlpha = 1;
    }

    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-mapStore.worldYawToView(robot.pose.yaw) + (view.rotation ?? 0));
    ctx.beginPath();
    ctx.moveTo(11, 0);
    ctx.lineTo(-7, 7);
    ctx.lineTo(-4, 0);
    ctx.lineTo(-7, -7);
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.fill();
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.restore();

    if (showLabels) {
      ctx.font = '600 10px ui-sans-serif, system-ui';
      ctx.fillStyle = color;
      ctx.textAlign = 'center';
      ctx.fillText(robotDisplayName(robot.robot_id), sx, sy - 18);
    }
  }
}

export function drawScaleBar(
  ctx: CanvasRenderingContext2D,
  info: MapInfo | null,
  view: Viewport,
  width: number,
  height: number
) {
  if (!info) return;
  const metres = 5;
  const px = (metres / info.resolution) * view.scale;
  if (px <= 24 || px >= width * 0.6) return;
  ctx.strokeStyle = '#98989d';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(16, height - 18);
  ctx.lineTo(16 + px, height - 18);
  ctx.moveTo(16, height - 22);
  ctx.lineTo(16, height - 14);
  ctx.moveTo(16 + px, height - 22);
  ctx.lineTo(16 + px, height - 14);
  ctx.stroke();
  ctx.font = '500 10px ui-sans-serif, system-ui';
  ctx.fillStyle = '#6e6e73';
  ctx.textAlign = 'left';
  ctx.fillText(`${metres} m`, 16, height - 26);
}
