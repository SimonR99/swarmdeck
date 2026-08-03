<script lang="ts">
  import {
    Crosshair,
    Focus,
    Grid3X3,
    Layers3,
    Locate,
    Maximize2,
    Minus,
    Plus,
    Route,
    ScanLine,
    Tags,
    Trash2
  } from 'lucide-svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { mapStore } from '$lib/stores/mapstore.svelte';
  import { session } from '$lib/stores/session.svelte';
  import { navigation } from '$lib/stores/navigation.svelte';
  import { settings } from '$lib/stores/settings.svelte';
  import { actions } from '$lib/api/connection';
  import type { MapRegistration } from '$lib/types/protocol';

  let host = $state<HTMLDivElement | null>(null);
  let canvas = $state<HTMLCanvasElement | null>(null);
  let view = $state({ scale: 0.55, tx: 0, ty: 0, initialised: false });
  let follow = $state(true);
  let cursorWorld = $state<{ x: number; y: number } | null>(null);
  let layersOpen = $state(false);
  let showGrid = $state(true);
  let showTrails = $state(true);
  let showLabels = $state(true);
  let showSensors = $state(false);
  let showPlans = $state(true);

  const trails = new Map<string, { x: number; y: number }[]>();
  const pointers = new Map<number, { x: number; y: number }>();
  let pinchStart = 0;
  let scaleStart = 1;
  let dragged = false;

  /** Screen px per grid cell, then per metre. */
  function screenOf(gx: number, gy: number) {
    return { sx: gx * view.scale + view.tx, sy: gy * view.scale + view.ty };
  }

  function gridOf(sx: number, sy: number) {
    return { gx: (sx - view.tx) / view.scale, gy: (sy - view.ty) / view.scale };
  }

  function robotsOnMap() {
    if (mapStore.viewMode === 'local' && mapStore.viewRobot) {
      const robot = fleet.get(mapStore.viewRobot);
      return robot ? [robot] : [];
    }
    const members = mapStore.status?.global_members;
    if (mapStore.status?.mode === 'auto' && members) {
      return fleet.robots.filter((robot) => members.includes(robot.robot_id));
    }
    return fleet.robots;
  }

  function centreOnFleet() {
    const info = mapStore.info;
    if (!info || !canvas || fleet.count === 0) return;
    let sx = 0,
      sy = 0,
      n = 0;
    for (const r of robotsOnMap()) {
      const g = mapStore.worldToGrid(r.pose.x, r.pose.y);
      if (!g) continue;
      sx += g.gx;
      sy += g.gy;
      n++;
    }
    if (!n) return;
    const cx = sx / n;
    const cy = sy / n;
    view.tx = canvas.width / (2 * devicePixelRatio) - cx * view.scale;
    view.ty = canvas.height / (2 * devicePixelRatio) - cy * view.scale;
  }

  function centreOnSelected() {
    const info = mapStore.info;
    if (!info || !canvas) return;
    const selected = fleet.selected.map((id) => fleet.get(id)).filter((r) => r !== undefined);
    if (!selected.length) return;
    let gx = 0;
    let gy = 0;
    for (const robot of selected) {
      const p = mapStore.worldToGrid(robot.pose.x, robot.pose.y);
      if (!p) continue;
      gx += p.gx;
      gy += p.gy;
    }
    gx /= selected.length;
    gy /= selected.length;
    view.tx = canvas.width / (2 * devicePixelRatio) - gx * view.scale;
    view.ty = canvas.height / (2 * devicePixelRatio) - gy * view.scale;
    follow = false;
  }

  function clearTrails() {
    trails.clear();
  }

  function fitMap() {
    const info = mapStore.info;
    if (!info || !host) return;
    const padding = 32;
    const width = Math.max(1, host.clientWidth - padding * 2);
    const height = Math.max(1, host.clientHeight - padding * 2);
    view.scale = Math.max(0.12, Math.min(6, Math.min(width / info.width, height / info.height)));
    view.tx = (host.clientWidth - info.width * view.scale) / 2;
    view.ty = (host.clientHeight - info.height * view.scale) / 2;
    follow = false;
  }

  function zoomBy(factor: number, ax?: number, ay?: number) {
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const px = ax ?? rect.width / 2;
    const py = ay ?? rect.height / 2;
    const before = gridOf(px, py);
    view.scale = Math.max(0.12, Math.min(6, view.scale * factor));
    const after = screenOf(before.gx, before.gy);
    view.tx += px - after.sx;
    view.ty += py - after.sy;
    follow = false;
  }

  function drawMetricGrid(ctx: CanvasRenderingContext2D, width: number, height: number) {
    const info = mapStore.info;
    if (!info || !showGrid) return;
    const pixelsPerMetre = view.scale / info.resolution;
    const spacing = pixelsPerMetre >= 24 ? 1 : pixelsPerMetre >= 7 ? 5 : 10;
    const maxX = info.origin.x + info.width * info.resolution;
    const maxY = info.origin.y + info.height * info.resolution;

    ctx.save();
    ctx.beginPath();
    ctx.rect(0, 0, width, height);
    ctx.clip();
    ctx.lineWidth = 1;
    ctx.strokeStyle = 'rgba(60, 68, 80, 0.10)';

    for (let x = Math.ceil(info.origin.x / spacing) * spacing; x <= maxX; x += spacing) {
      const a = mapStore.viewToGrid(x, info.origin.y);
      const b = mapStore.viewToGrid(x, maxY);
      if (!a || !b) continue;
      const sa = screenOf(a.gx, a.gy);
      const sb = screenOf(b.gx, b.gy);
      ctx.moveTo(sa.sx, sa.sy);
      ctx.lineTo(sb.sx, sb.sy);
    }
    for (let y = Math.ceil(info.origin.y / spacing) * spacing; y <= maxY; y += spacing) {
      const a = mapStore.viewToGrid(info.origin.x, y);
      const b = mapStore.viewToGrid(maxX, y);
      if (!a || !b) continue;
      const sa = screenOf(a.gx, a.gy);
      const sb = screenOf(b.gx, b.gy);
      ctx.moveTo(sa.sx, sa.sy);
      ctx.lineTo(sb.sx, sb.sy);
    }
    ctx.stroke();

    // World origin remains distinct from the ordinary metric grid.
    const origin = mapStore.viewToGrid(0, 0);
    if (origin) {
      const s = screenOf(origin.gx, origin.gy);
      ctx.strokeStyle = 'rgba(11, 92, 173, 0.24)';
      ctx.beginPath();
      ctx.moveTo(s.sx, 0);
      ctx.lineTo(s.sx, height);
      ctx.moveTo(0, s.sy);
      ctx.lineTo(width, s.sy);
      ctx.stroke();
    }
    ctx.restore();
  }

  function draw() {
    if (!canvas || !host) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const dpr = window.devicePixelRatio || 1;
    const w = host.clientWidth;
    const h = host.clientHeight;
    if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
      canvas.width = w * dpr;
      canvas.height = h * dpr;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    // background
    ctx.fillStyle = '#f5f5f7';
    ctx.fillRect(0, 0, w, h);

    const info = mapStore.info;
    const grid = mapStore.canvas;

    if (!view.initialised && info && fleet.count) {
      view.initialised = true;
      fitMap();
    }
    if (follow) centreOnFleet();

    // occupancy grid
    if (grid && info) {
      ctx.imageSmoothingEnabled = view.scale < 1;
      ctx.drawImage(grid, view.tx, view.ty, info.width * view.scale, info.height * view.scale);
    }

    drawMetricGrid(ctx, w, h);

    // detections
    for (const d of session.detections) {
      if (mapStore.viewMode === 'local' && d.robot_id !== mapStore.viewRobot) continue;
      if (!d.map_position) continue;
      const g = mapStore.worldToGrid(d.map_position.x, d.map_position.y);
      if (!g) continue;
      const { sx, sy } = screenOf(g.gx, g.gy);
      ctx.beginPath();
      ctx.arc(sx, sy, 7, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(251,191,36,0.22)';
      ctx.fill();
      ctx.lineWidth = 1.5;
      ctx.strokeStyle = '#fbbf24';
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(sx, sy, 2, 0, Math.PI * 2);
      ctx.fillStyle = '#fbbf24';
      ctx.fill();
    }

    // per robot: trail, goal, body
    for (const r of robotsOnMap()) {
      const color = fleet.colorOf(r.robot_id);
      const g = mapStore.worldToGrid(r.pose.x, r.pose.y);
      if (!g) continue;
      const { sx, sy } = screenOf(g.gx, g.gy);

      // trail
      let t = trails.get(r.robot_id);
      if (!t) trails.set(r.robot_id, (t = []));
      const last = t[t.length - 1];
      if (!last || Math.hypot(last.x - g.gx, last.y - g.gy) > 3) {
        t.push({ x: g.gx, y: g.gy });
        if (t.length > 400) t.shift();
      }
      if (showTrails && t.length > 1) {
        ctx.beginPath();
        const p0 = screenOf(t[0].x, t[0].y);
        ctx.moveTo(p0.sx, p0.sy);
        for (const p of t.slice(1)) {
          const s = screenOf(p.x, p.y);
          ctx.lineTo(s.sx, s.sy);
        }
        ctx.strokeStyle = color;
        ctx.globalAlpha = 0.28;
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.globalAlpha = 1;
      }

      // Nav2's current global plan (predicted route), visually stronger and
      // dashed so it cannot be confused with the path already travelled.
      if (showPlans && r.planned_path?.length > 1) {
        ctx.beginPath();
        const first = mapStore.worldToGrid(r.planned_path[0].x, r.planned_path[0].y);
        if (first) {
          const p0 = screenOf(first.gx, first.gy);
          ctx.moveTo(p0.sx, p0.sy);
          for (const point of r.planned_path.slice(1)) {
            const gridPoint = mapStore.worldToGrid(point.x, point.y);
            if (!gridPoint) continue;
            const p = screenOf(gridPoint.gx, gridPoint.gy);
            ctx.lineTo(p.sx, p.sy);
          }
          ctx.setLineDash([7, 4]);
          ctx.strokeStyle = color;
          ctx.globalAlpha = 0.78;
          ctx.lineWidth = 2.5;
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.globalAlpha = 1;
        }
      }

      // Optional sensor/footprint layer: useful for checking heading, camera
      // coverage and whether a proposed route has reasonable wall clearance.
      if (showSensors && info) {
        const footprintPx = (0.3 / info.resolution) * view.scale;
        const sensorPx = (2.0 / info.resolution) * view.scale;
        ctx.save();
        ctx.translate(sx, sy);
        ctx.rotate(-mapStore.worldYawToView(r.pose.yaw));
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

        ctx.beginPath();
        ctx.arc(sx, sy, footprintPx, 0, Math.PI * 2);
        ctx.setLineDash([3, 3]);
        ctx.strokeStyle = color;
        ctx.globalAlpha = 0.32;
        ctx.lineWidth = 1;
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.globalAlpha = 1;
      }

      // goal + link
      if (r.goal) {
        const gg = mapStore.worldToGrid(r.goal.x, r.goal.y);
        if (gg) {
          const s = screenOf(gg.gx, gg.gy);
          ctx.setLineDash([5, 5]);
          ctx.beginPath();
          ctx.moveTo(sx, sy);
          ctx.lineTo(s.sx, s.sy);
          ctx.strokeStyle = color;
          ctx.globalAlpha = 0.5;
          ctx.lineWidth = 1.5;
          ctx.stroke();
          ctx.setLineDash([]);
          ctx.globalAlpha = 1;

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

      // selection halo
      if (fleet.isSelected(r.robot_id)) {
        ctx.beginPath();
        ctx.arc(sx, sy, 16, 0, Math.PI * 2);
        ctx.strokeStyle = color;
        ctx.globalAlpha = 0.45;
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.globalAlpha = 1;
      }

      // body: triangle pointing along yaw (screen y is inverted vs world)
      ctx.save();
      ctx.translate(sx, sy);
      ctx.rotate(-mapStore.worldYawToView(r.pose.yaw));
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

      // label
      if (showLabels) {
        ctx.font = '600 10px ui-sans-serif, system-ui';
        ctx.fillStyle = color;
        ctx.textAlign = 'center';
        ctx.fillText(r.robot_id.replace(/^robot_/, 'R'), sx, sy - 18);
      }
    }

    // scale bar
    if (info) {
      const metres = 5;
      const px = (metres / info.resolution) * view.scale;
      if (px > 24 && px < w * 0.6) {
        ctx.strokeStyle = '#98989d';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(16, h - 18);
        ctx.lineTo(16 + px, h - 18);
        ctx.moveTo(16, h - 22);
        ctx.lineTo(16, h - 14);
        ctx.moveTo(16 + px, h - 22);
        ctx.lineTo(16 + px, h - 14);
        ctx.stroke();
        ctx.font = '500 10px ui-sans-serif, system-ui';
        ctx.fillStyle = '#6e6e73';
        ctx.textAlign = 'left';
        ctx.fillText(`${metres} m`, 16, h - 26);
      }
    }
  }

  // Redraw whenever the map, fleet, settings, or view changes.
  $effect(() => {
    void mapStore.revision;
    void fleet.robots;
    void settings.value;
    void view.scale;
    void view.tx;
    void view.ty;
    void session.detections;
    void showGrid;
    void showTrails;
    void showLabels;
    void showSensors;
    void showPlans;
    draw();
  });

  $effect(() => {
    let raf = 0;
    const loop = () => {
      draw();
      raf = requestAnimationFrame(loop);
    };
    raf = requestAnimationFrame(loop);
    const ro = new ResizeObserver(() => draw());
    if (host) ro.observe(host);
    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
    };
  });

  $effect(() => {
    const cancelGoalMode = (event: KeyboardEvent) => {
      if (event.key === 'Escape') navigation.cancelGoalMode();
    };
    window.addEventListener('keydown', cancelGoalMode);
    return () => window.removeEventListener('keydown', cancelGoalMode);
  });

  $effect(() => {
    void mapStore.refreshStatus();
    const timer = window.setInterval(() => void mapStore.refreshStatus(), 3000);
    return () => window.clearInterval(timer);
  });

  $effect(() => {
    const selectedRobot = fleet.selected.length === 1 ? fleet.selected[0] : null;
    void mapStore.statusUpdatedAt;
    void mapStore.selectRobotView(selectedRobot);
  });

  $effect(() => {
    void mapStore.viewMode;
    void mapStore.viewRobot;
    trails.clear();
    view.initialised = false;
  });

  function onPointerDown(e: PointerEvent) {
    (e.target as HTMLElement).setPointerCapture(e.pointerId);
    pointers.set(e.pointerId, { x: e.clientX, y: e.clientY });
    dragged = false;
    if (pointers.size === 2) {
      const [a, b] = [...pointers.values()];
      pinchStart = Math.hypot(a.x - b.x, a.y - b.y);
      scaleStart = view.scale;
    }
  }

  function onPointerMove(e: PointerEvent) {
    const prev = pointers.get(e.pointerId);
    if (!prev) {
      if (canvas) {
        const rect = canvas.getBoundingClientRect();
        const g = gridOf(e.clientX - rect.left, e.clientY - rect.top);
        cursorWorld = mapStore.gridToWorld(g.gx, g.gy);
      }
      return;
    }
    const cur = { x: e.clientX, y: e.clientY };
    pointers.set(e.pointerId, cur);

    if (pointers.size === 2 && pinchStart > 0) {
      const [a, b] = [...pointers.values()];
      const d = Math.hypot(a.x - b.x, a.y - b.y);
      const target = Math.max(0.12, Math.min(6, (scaleStart * d) / pinchStart));
      zoomBy(target / view.scale);
      dragged = true;
      return;
    }

    const dx = cur.x - prev.x;
    const dy = cur.y - prev.y;
    if (Math.abs(dx) + Math.abs(dy) > 2) {
      dragged = true;
      follow = false;
    }
    view.tx += dx;
    view.ty += dy;
  }

  function onPointerUp(e: PointerEvent) {
    const wasDrag = dragged;
    pointers.delete(e.pointerId);
    if (pointers.size < 2) pinchStart = 0;
    if (wasDrag || !canvas) return;

    const rect = canvas.getBoundingClientRect();
    const clickX = e.clientX - rect.left;
    const clickY = e.clientY - rect.top;

    // In inspection mode, map markers are directly selectable. Shift-click
    // mirrors the fleet rail's additive selection behaviour.
    if (!navigation.goalMode) {
      let nearest: { id: string; distance: number } | null = null;
      for (const robot of robotsOnMap()) {
        const grid = mapStore.worldToGrid(robot.pose.x, robot.pose.y);
        if (!grid) continue;
        const screen = screenOf(grid.gx, grid.gy);
        const distance = Math.hypot(screen.sx - clickX, screen.sy - clickY);
        if (distance <= 18 && (!nearest || distance < nearest.distance)) {
          nearest = { id: robot.robot_id, distance };
        }
      }
      if (nearest) {
        fleet.select(nearest.id, e.shiftKey);
        actions.selectRobots(fleet.selected);
      }
      return;
    }

    // Point navigation is deliberately armed first to prevent accidental goals
    // while the operator pans or inspects the map.
    const g = gridOf(clickX, clickY);
    const world = mapStore.gridToWorld(g.gx, g.gy);
    if (!world) return;
    const targets = fleet.selected.filter((id) => fleet.can(id, 'navigate'));
    for (const id of targets) actions.setGoal(id, world);
    if (targets.length) navigation.finishGoal(world);
  }

  function onWheel(e: WheelEvent) {
    e.preventDefault();
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    zoomBy(e.deltaY < 0 ? 1.12 : 1 / 1.12, e.clientX - rect.left, e.clientY - rect.top);
  }

  const canGoal = $derived(fleet.selected.filter((id) => fleet.can(id, 'navigate')).length);
  const registrationEntries = $derived(Object.entries(mapStore.status?.registrations ?? {}));
  const protectedRegistrations = $derived(
    registrationEntries.filter(([, item]) => item.rejection?.startsWith('outside configured prior'))
      .length
  );
  const acceptedRegistrations = $derived(
    registrationEntries.filter(([, item]) => item.accepted).length
  );

  /** Why a match has not been accepted, in terms the operator can act on:
   *  drive the robots through the same rooms, versus this building cannot be
   *  told apart under rotation and needs a configured start pose. */
  function registrationBlocker(item: MapRegistration): string {
    if (item.support < 0.35) return 'Waiting for overlap';
    if (item.yaw_ratio > 0.8) return 'Rotation ambiguous';
    return 'Too little detail';
  }
</script>

<div class="panel-glow relative h-full w-full overflow-hidden rounded-[--radius-card] border border-border bg-bg">
  <div bind:this={host} class="absolute inset-0">
    <canvas
      bind:this={canvas}
      class="h-full w-full touch-none"
      style="cursor:{navigation.goalMode && canGoal ? 'crosshair' : 'grab'}"
      onpointerdown={onPointerDown}
      onpointermove={onPointerMove}
      onpointerup={onPointerUp}
      onpointercancel={onPointerUp}
      onwheel={onWheel}
    ></canvas>
  </div>

  {#if !mapStore.ready}
    <div class="pointer-events-none absolute inset-0 grid place-items-center">
      <div class="text-xs text-fg-dim">Waiting for map…</div>
    </div>
  {/if}

  <div class="pointer-events-none absolute left-3 top-3 flex items-center gap-1.5">
    <div
      class="flex items-center gap-2 rounded-[4px] border border-border bg-surface/88 px-2.5 py-1
             text-[10px] font-medium text-fg-muted shadow-sm backdrop-blur-xl"
    >
      <span class="h-1.5 w-1.5 rounded-full {mapStore.ready ? 'bg-ok' : 'bg-warn'}"></span>
      {mapStore.ready ? mapStore.viewLabel : 'Connecting'}
    </div>
    {#if mapStore.status}
      <div
        class="rounded-[4px] border border-border bg-surface/88 px-2.5 py-1 text-[10px]
               font-medium text-fg-muted shadow-sm backdrop-blur-xl"
        title="Inter-robot map alignment mode"
      >
        Alignment {mapStore.status.mode}
        {#if protectedRegistrations > 0}
          · {protectedRegistrations} prior-held
        {:else if acceptedRegistrations > 0}
          · {acceptedRegistrations} matched
        {:else if mapStore.status.mode === 'auto'}
          · local only
        {/if}
      </div>
    {/if}
  </div>

  <!-- goal hint -->
  {#if navigation.goalMode && canGoal > 0}
    <div
      class="pointer-events-none absolute left-1/2 top-3 -translate-x-1/2 rounded-[4px] border
             border-accent/20 bg-surface/90 px-3 py-1.5 text-[10px] font-medium text-accent
             shadow-sm backdrop-blur-xl"
    >
      <Crosshair class="mr-1 inline h-3 w-3" />
      Click destination for {canGoal} robot{canGoal > 1 ? 's' : ''} · Esc to cancel
    </div>
  {/if}

  <!-- Map layers and registration diagnostics. -->
  <div class="absolute bottom-3 right-14">
    <button
      title="Map layers"
      class="panel-glow flex h-8 items-center gap-1.5 rounded-[4px] border border-border
             bg-surface/95 px-2.5 text-[10px] font-medium text-fg-muted transition-colors
             hover:bg-surface-2 {layersOpen ? 'text-accent' : ''}"
      onclick={() => (layersOpen = !layersOpen)}
    >
      <Layers3 class="h-3.5 w-3.5" />
      Layers
    </button>

    {#if layersOpen}
      <div
        class="panel-glow absolute bottom-10 right-0 w-56 rounded-[5px] border border-border bg-surface/97 p-2.5
               text-[10px] shadow-lg backdrop-blur-xl"
      >
        <div class="mb-2 text-[9px] font-semibold uppercase tracking-[0.08em] text-fg-dim">
          Overlays
        </div>
        <button
          class="flex h-7 w-full items-center justify-between rounded-[3px] px-1.5 text-fg-muted hover:bg-surface-2"
          onclick={() => (showGrid = !showGrid)}
        >
          <span class="flex items-center gap-2"><Grid3X3 class="h-3.5 w-3.5" /> Metric grid</span>
          <span class="font-semibold {showGrid ? 'text-accent' : 'text-fg-dim'}">{showGrid ? 'ON' : 'OFF'}</span>
        </button>
        <button
          class="flex h-7 w-full items-center justify-between rounded-[3px] px-1.5 text-fg-muted hover:bg-surface-2"
          onclick={() => (showTrails = !showTrails)}
        >
          <span class="flex items-center gap-2"><Focus class="h-3.5 w-3.5" /> Robot trails</span>
          <span class="font-semibold {showTrails ? 'text-accent' : 'text-fg-dim'}">{showTrails ? 'ON' : 'OFF'}</span>
        </button>
        <button
          class="flex h-7 w-full items-center justify-between rounded-[3px] px-1.5 text-fg-muted hover:bg-surface-2"
          onclick={() => (showPlans = !showPlans)}
        >
          <span class="flex items-center gap-2"><Route class="h-3.5 w-3.5" /> Predicted routes</span>
          <span class="font-semibold {showPlans ? 'text-accent' : 'text-fg-dim'}">{showPlans ? 'ON' : 'OFF'}</span>
        </button>
        <button
          class="flex h-7 w-full items-center justify-between rounded-[3px] px-1.5 text-fg-muted hover:bg-surface-2"
          onclick={() => (showLabels = !showLabels)}
        >
          <span class="flex items-center gap-2"><Tags class="h-3.5 w-3.5" /> Robot labels</span>
          <span class="font-semibold {showLabels ? 'text-accent' : 'text-fg-dim'}">{showLabels ? 'ON' : 'OFF'}</span>
        </button>
        <button
          class="flex h-7 w-full items-center justify-between rounded-[3px] px-1.5 text-fg-muted hover:bg-surface-2"
          onclick={() => (showSensors = !showSensors)}
        >
          <span class="flex items-center gap-2"><ScanLine class="h-3.5 w-3.5" /> Sensors + footprint</span>
          <span class="font-semibold {showSensors ? 'text-accent' : 'text-fg-dim'}">{showSensors ? 'ON' : 'OFF'}</span>
        </button>
        <button
          class="mt-1 flex h-7 w-full items-center gap-2 rounded-[3px] border-t border-border px-1.5
                 text-fg-muted hover:bg-surface-2"
          onclick={clearTrails}
        >
          <Trash2 class="h-3.5 w-3.5" /> Clear trails
        </button>

        <div class="my-2 border-t border-border"></div>
        <div class="mb-1 text-[9px] font-semibold uppercase tracking-[0.08em] text-fg-dim">
          Occupancy
        </div>
        <div class="flex items-center gap-3 px-1 py-1 text-fg-dim">
          <span class="flex items-center gap-1"><i class="h-2.5 w-2.5 border border-border bg-white"></i> Free</span>
          <span class="flex items-center gap-1"><i class="h-2.5 w-2.5 bg-[#343a44]"></i> Wall</span>
          <span class="flex items-center gap-1"><i class="h-2.5 w-2.5 bg-[#e5e8ec]"></i> Unknown</span>
        </div>

        {#if mapStore.status}
          <div class="my-2 border-t border-border"></div>
          <div class="flex items-center justify-between text-fg-dim">
            <span>Reference frame</span>
            <span class="font-medium text-fg-muted">{mapStore.status.reference ?? '—'}</span>
          </div>
          {#each registrationEntries as [robotId, item]}
            <div class="mt-1 flex items-start justify-between gap-2 text-fg-dim">
              <span>{robotId.replace(/^robot_/, 'R')}</span>
              <span
                class="max-w-36 text-right font-medium {item.accepted
                  ? 'text-ok'
                  : item.rejection?.startsWith('outside')
                    ? 'text-warn'
                    : 'text-fg-muted'}"
                title={item.rejection ??
                  `score ${item.score} · shared area ${(item.support * 100).toFixed(0)}%` +
                    ` · rival rotation ${(item.yaw_ratio * 100).toFixed(0)}%`}
              >
                {item.accepted
                  ? `Matched · ${(item.score * 100).toFixed(0)}%`
                  : item.rejection?.startsWith('outside')
                    ? 'Configured prior held'
                    : registrationBlocker(item)}
              </span>
            </div>
          {/each}
        {/if}
      </div>
    {/if}
  </div>

  <!-- view controls -->
  <div class="panel-glow absolute bottom-3 right-3 flex flex-col overflow-hidden rounded-[4px] border border-border bg-surface/95">
    <button
      title="Follow fleet"
      class="grid h-9 w-9 touch-target place-items-center border-b border-border
             transition-colors hover:bg-surface-2
             {follow ? 'bg-accent/8 text-accent' : 'text-fg-muted'}"
      onclick={() => {
        follow = !follow;
        if (follow) centreOnFleet();
      }}
    >
      <Locate class="h-4 w-4" />
    </button>
    <button
      title="Centre selected robots"
      disabled={fleet.selected.length === 0}
      class="grid h-9 w-9 touch-target place-items-center border-b border-border text-fg-muted
             transition-colors hover:bg-surface-2 disabled:opacity-35"
      onclick={centreOnSelected}
    >
      <Focus class="h-4 w-4" />
    </button>
    <button
      title="Zoom in"
      class="grid h-9 w-9 touch-target place-items-center border-b border-border text-fg-muted
             transition-colors hover:bg-surface-2"
      onclick={() => zoomBy(1.25)}
    >
      <Plus class="h-4 w-4" />
    </button>
    <button
      title="Zoom out"
      class="grid h-9 w-9 touch-target place-items-center border-b border-border text-fg-muted
             transition-colors hover:bg-surface-2"
      onclick={() => zoomBy(1 / 1.25)}
    >
      <Minus class="h-4 w-4" />
    </button>
    <button
      title="Fit fleet"
      class="grid h-9 w-9 touch-target place-items-center text-fg-muted transition-colors
             hover:bg-surface-2"
      onclick={() => {
        fitMap();
      }}
    >
      <Maximize2 class="h-4 w-4" />
    </button>
  </div>

  <!-- cursor readout -->
  <div
    class="pointer-events-none absolute bottom-3 left-3 flex items-center gap-2 rounded-[4px]
           border border-border bg-surface/88 px-2.5 py-1 text-[10px] tabular text-fg-dim
           shadow-sm backdrop-blur-xl"
  >
    <span>{cursorWorld ? `${cursorWorld.x.toFixed(1)}, ${cursorWorld.y.toFixed(1)} m` : 'Move cursor to inspect'}</span>
    {#if mapStore.info}
      <span class="h-3 w-px bg-border"></span>
      <span>{Math.round(mapStore.info.resolution * 100)} cm/cell</span>
      <span class="h-3 w-px bg-border"></span>
      <span>rev {mapStore.seq}</span>
    {/if}
  </div>
</div>
