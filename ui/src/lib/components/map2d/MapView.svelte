<script lang="ts">
  import {
    Box,
    Crosshair,
    Focus,
    Grid3X3,
    Layers3,
    Locate,
    Maximize2,
    Minus,
    Plus,
    RotateCcw,
    Route,
    ScanLine,
    Tags,
    Trash2,
    Wifi
  } from 'lucide-svelte';
  import Map3D from '$lib/components/map3d/Map3D.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { mapStore } from '$lib/stores/mapstore.svelte';
  import { session } from '$lib/stores/session.svelte';
  import { navigation } from '$lib/stores/navigation.svelte';
  import { settings } from '$lib/stores/settings.svelte';
  import { review } from '$lib/stores/review.svelte';
  import { actions } from '$lib/api/connection';
  import { robotDisplayName } from '$lib/robotDisplayName';
  import type { MapRegistration } from '$lib/types/protocol';
  import {
    drawLoopClosures,
    drawMetricGrid,
    drawNetworkHeatmap,
    drawReviewedObjects,
    drawRobots,
    drawScaleBar,
    hitTestReviewedObject
  } from './mapLayers';

  let host = $state<HTMLDivElement | null>(null);
  let canvas = $state<HTMLCanvasElement | null>(null);
  let map3D = $state<{
    centreFleet: () => void;
    centreSelected: () => void;
    zoomBy: (factor: number) => void;
    fitCloud: () => void;
  } | null>(null);
  let view = $state({ scale: 0.55, tx: 0, ty: 0, initialised: false });
  let follow = $state(true);
  let cursorWorld = $state<{ x: number; y: number } | null>(null);
  let layersOpen = $state(false);
  // `?view=3d` opens straight into the point cloud. The frontend already takes
  // `?mock=1&robots=4`, so URL-driven view state is the existing idiom here —
  // and it is the only way to reach the 3D view from a headless browser, which
  // is how it gets verified.
  let show3D = $state(
    typeof location !== 'undefined' &&
      new URLSearchParams(location.search).get('view') === '3d'
  );
  let showGrid = $state(true);
  let showTrails = $state(true);
  let showLabels = $state(true);
  let showSensors = $state(false);
  let showPlans = $state(true);
  let showNetwork = $state(true);
  let resetPending = $state(false);
  let resetError = $state<string | null>(null);

  const trails = new Map<string, { x: number; y: number }[]>();
  const pointers = new Map<number, { x: number; y: number }>();
  let pinchStart = 0;
  let dragged = false;
  let lastRenderedInfo: MapInfo | null = null;

  /** Screen px per grid cell, then per metre. */
  function screenOf(gx: number, gy: number) {
    return { sx: gx * view.scale + view.tx, sy: gy * view.scale + view.ty };
  }

  function gridOf(sx: number, sy: number) {
    return { gx: (sx - view.tx) / view.scale, gy: (sy - view.ty) / view.scale };
  }

  function robotsOnMap() {
    if (mapStore.viewMode === 'local' && mapStore.viewRobot) {
      if (!fleet.isEnabled(mapStore.viewRobot)) return [];
      const robot = fleet.get(mapStore.viewRobot);
      return robot ? [robot] : [];
    }
    const members = mapStore.status?.global_members;
    if (members && members.length > 0) {
      return fleet.robots.filter((robot) => members.includes(robot.robot_id) && fleet.isEnabled(robot.robot_id));
    }
    return [];
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

  async function resetMaps(robotId?: string) {
    const label = robotId ? robotDisplayName(robotId) : 'every robot';
    const confirmed = window.confirm(
      `Reset the accumulated map for ${label}?\n\n` +
        'This clears SwarmDeck map data only. Robot-side SLAM and recordings keep running.'
    );
    if (!confirmed) return;

    resetPending = true;
    resetError = null;
    try {
      await actions.resetMap(robotId);
      clearTrails();
      await mapStore.reloadCurrentView();
      await mapStore.refreshStatus();
    } catch (error) {
      resetError = error instanceof Error ? error.message : 'Map reset failed';
    } finally {
      resetPending = false;
    }
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

  function toggleFollow() {
    follow = !follow;
    if (!follow) return;
    if (show3D) map3D?.centreFleet();
    else centreOnFleet();
  }

  function centreSelectedForView() {
    if (show3D) {
      map3D?.centreSelected();
      follow = false;
    } else {
      centreOnSelected();
    }
  }

  function zoomForView(factor: number) {
    if (show3D) map3D?.zoomBy(factor);
    else zoomBy(factor);
  }

  function fitView() {
    if (show3D) {
      map3D?.fitCloud();
      follow = false;
    } else {
      fitMap();
    }
  }

  // Canvas layers live in mapLayers.ts; this component owns only viewport state.
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
    ctx.fillStyle = '#d6dae0';
    ctx.fillRect(0, 0, w, h);

    const info = mapStore.info;
    const grid = mapStore.canvas;
    if (!view.initialised && info && fleet.count) {
      view.initialised = true;
      lastRenderedInfo = info;
      fitMap();
    } else if (view.initialised && lastRenderedInfo && info) {
      if (
        lastRenderedInfo.origin.x !== info.origin.x ||
        lastRenderedInfo.origin.y !== info.origin.y ||
        lastRenderedInfo.width !== info.width ||
        lastRenderedInfo.height !== info.height
      ) {
        const offGx = (lastRenderedInfo.origin.x - info.origin.x) / info.resolution;
        const offGy =
          (info.height - lastRenderedInfo.height) +
          (lastRenderedInfo.origin.y - info.origin.y) / info.resolution;
        view.tx -= offGx * view.scale;
        view.ty -= offGy * view.scale;
      }
      lastRenderedInfo = info;
    } else if (info) {
      lastRenderedInfo = info;
    }
    if (follow) centreOnFleet();

    if (grid && info) {
      const gw = info.width * view.scale;
      const gh = info.height * view.scale;
      ctx.imageSmoothingEnabled = view.scale < 1;
      ctx.drawImage(grid, view.tx, view.ty, gw, gh);
      drawNetworkHeatmap(ctx, screenOf, view, showNetwork);
      const mask = mapStore.occupancyMask(view.scale);
      if (mask) {
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(mask, view.tx, view.ty, gw, gh);
      }
    }

    drawMetricGrid(ctx, w, h, info, view, screenOf, showGrid);
    drawLoopClosures(ctx, screenOf, showPlans);
    drawReviewedObjects(ctx, screenOf);
    if (info) {
      drawRobots(ctx, robotsOnMap(), {
        info,
        view,
        screenOf,
        trails,
        showTrails,
        showPlans,
        showSensors,
        showLabels
      });
    }
    drawScaleBar(ctx, info, view, w, h);
  }

  // Redraw whenever the map, fleet, settings, or view changes.
  $effect(() => {
    if (mapStore.viewMode === 'local' && mapStore.viewRobot && !fleet.isEnabled(mapStore.viewRobot)) {
      void mapStore.setViewPreference('global', null);
    }
  });

  $effect(() => {
    void mapStore.revision;
    void fleet.robots;
    void settings.value;
    void view.scale;
    void view.tx;
    void view.ty;
    void session.detections;
    void review.entities;
    void review.proposals;
    void review.focused;
    void review.selected;
    void showGrid;
    void showTrails;
    void showLabels;
    void showSensors;
    void showPlans;
    void showNetwork;
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
    lastRenderedInfo = null;
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
      const detection = hitTestReviewedObject(clickX, clickY, screenOf);
      if (detection) {
        if (review.selected === detection.id) {
          review.select(null);
        } else {
          review.select(detection.id);
          const robotId = detection.robotId ?? detection.robotIds[0];
          if (robotId) actions.focusRobot(robotId);
        }
        return;
      }

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
  const resetRobotId = $derived(fleet.selected.length === 1 ? fleet.selected[0] : null);
  const resetRobot = $derived(resetRobotId ? fleet.get(resetRobotId) : undefined);
  const viewedNetwork = $derived(
    mapStore.viewRobot ? fleet.get(mapStore.viewRobot)?.network ?? null : null
  );
  const resetRobotBlocked = $derived(
    resetPending ||
      !resetRobot ||
      !resetRobot.capabilities.includes('map') ||
      resetRobot.nav_status === 'active' ||
      resetRobot.goal !== null
  );
  const resetAllBlocked = $derived(
    resetPending ||
      fleet.robots.some((robot) => robot.nav_status === 'active' || robot.goal !== null)
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

<div class="panel-glow relative h-full w-full overflow-hidden rounded-[--radius-panel] border border-transparent bg-bg">
  <!--
    The 3D view sits over the 2D one rather than replacing it. 2D stays the
    operator's working surface — it is where goals are set and where the fleet
    is supervised — and 3D is a way of inspecting what the robots have actually
    built. Mounted only while shown, so a fleet on 2D SLAM never pays for a
    WebGL context it has no cloud to fill.
  -->
  {#if show3D}
    <div class="absolute inset-0 z-10">
      <Map3D bind:this={map3D} active={show3D} {follow} />
    </div>
  {/if}
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

  <!-- goal hint -->
  {#if navigation.goalMode && canGoal > 0}
    <div
      class="pointer-events-none absolute left-1/2 top-3 -translate-x-1/2 rounded-[--radius-control] border
             border-accent/20 bg-surface/90 px-3 py-1.5 text-[10px] font-medium text-accent
             shadow-sm backdrop-blur-xl"
    >
      <Crosshair class="mr-1 inline h-3 w-3" />
      Click destination for {canGoal} robot{canGoal > 1 ? 's' : ''} · Esc to cancel
    </div>
  {/if}

  <!-- Map layers and registration diagnostics. -->
  <div class="absolute bottom-3 right-20 z-20">
    <button
      title="Map layers"
      class="panel-glow flex h-11 items-center gap-2 rounded-full border border-transparent
             bg-surface/95 px-4 text-[11px] font-semibold text-fg-muted transition-colors
             hover:bg-surface-2 {layersOpen ? 'bg-accent-container text-accent-container-fg' : ''}"
      onclick={() => (layersOpen = !layersOpen)}
    >
      <Layers3 class="h-3.5 w-3.5" />
      Layers
    </button>

    {#if layersOpen}
      <div
        class="panel-glow absolute bottom-14 right-0 w-64 rounded-[--radius-panel] border border-transparent bg-surface/97 p-4
               text-[10px] shadow-[0_16px_40px_-18px_rgb(25_32_42/0.42)] backdrop-blur-xl"
      >
        <div class="mb-2 text-[9px] font-semibold uppercase tracking-[0.08em] text-fg-dim">
          Overlays
        </div>
        <button
          class="flex h-9 w-full items-center justify-between rounded-[--radius-control] px-1.5 text-fg-muted hover:bg-surface-2"
          onclick={() => (showGrid = !showGrid)}
        >
          <span class="flex items-center gap-2"><Grid3X3 class="h-3.5 w-3.5" /> Metric grid</span>
          <span class="font-semibold {showGrid ? 'text-accent' : 'text-fg-dim'}">{showGrid ? 'ON' : 'OFF'}</span>
        </button>
        <button
          class="flex h-9 w-full items-center justify-between rounded-[--radius-control] px-1.5 text-fg-muted hover:bg-surface-2"
          onclick={() => (showTrails = !showTrails)}
        >
          <span class="flex items-center gap-2"><Focus class="h-3.5 w-3.5" /> Robot trails</span>
          <span class="font-semibold {showTrails ? 'text-accent' : 'text-fg-dim'}">{showTrails ? 'ON' : 'OFF'}</span>
        </button>
        <button
          class="flex h-9 w-full items-center justify-between rounded-[--radius-control] px-1.5 text-fg-muted hover:bg-surface-2"
          title="Dashed = global planner route; solid = local controller route"
          onclick={() => (showPlans = !showPlans)}
        >
          <span class="flex items-center gap-2"><Route class="h-3.5 w-3.5" /> Global + local paths</span>
          <span class="font-semibold {showPlans ? 'text-accent' : 'text-fg-dim'}">{showPlans ? 'ON' : 'OFF'}</span>
        </button>
        <button
          class="flex h-9 w-full items-center justify-between rounded-[--radius-control] px-1.5 text-fg-muted hover:bg-surface-2"
          onclick={() => (showLabels = !showLabels)}
        >
          <span class="flex items-center gap-2"><Tags class="h-3.5 w-3.5" /> Robot labels</span>
          <span class="font-semibold {showLabels ? 'text-accent' : 'text-fg-dim'}">{showLabels ? 'ON' : 'OFF'}</span>
        </button>
        <button
          class="flex h-9 w-full items-center justify-between rounded-[--radius-control] px-1.5 text-fg-muted hover:bg-surface-2"
          onclick={() => (showSensors = !showSensors)}
        >
          <span class="flex items-center gap-2"><ScanLine class="h-3.5 w-3.5" /> Sensors + actual footprint</span>
          <span class="font-semibold {showSensors ? 'text-accent' : 'text-fg-dim'}">{showSensors ? 'ON' : 'OFF'}</span>
        </button>
        <button
          class="flex h-9 w-full items-center justify-between rounded-[--radius-control] px-1.5 text-fg-muted hover:bg-surface-2"
          title="Robot-side Wi-Fi quality on the selected robot's local map"
          onclick={() => (showNetwork = !showNetwork)}
        >
          <span class="flex items-center gap-2"><Wifi class="h-3.5 w-3.5" /> Network heatmap</span>
          <span class="font-semibold {showNetwork ? 'text-accent' : 'text-fg-dim'}">{showNetwork ? 'ON' : 'OFF'}</span>
        </button>
        <button
          class="flex h-9 w-full items-center justify-between rounded-[--radius-control] px-1.5 text-fg-muted hover:bg-surface-2"
          onclick={() => (show3D = !show3D)}
        >
          <span class="flex items-center gap-2"><Box class="h-3.5 w-3.5" /> 3D cloud</span>
          <span class="font-semibold {show3D ? 'text-accent' : 'text-fg-dim'}">{show3D ? 'ON' : 'OFF'}</span>
        </button>
        <button
          class="mt-1 flex h-9 w-full items-center gap-2 rounded-[--radius-control] border-t border-border px-1.5
                 text-fg-muted hover:bg-surface-2"
          onclick={clearTrails}
        >
          <Trash2 class="h-3.5 w-3.5" /> Clear trails
        </button>

        <div class="my-2 border-t border-border"></div>
        <div class="mb-1 text-[9px] font-semibold uppercase tracking-[0.08em] text-fg-dim">
          Map estimate
        </div>
        <div class="mb-1 flex gap-1">
          <button
            class="flex h-9 flex-1 items-center justify-center rounded-[--radius-control] px-1.5
                   {mapStore.mapSource === 'slam'
                     ? 'bg-surface-2 text-fg'
                     : 'text-fg-muted hover:bg-surface-2'}"
            title="The grid each robot's own SLAM package built, in its own frame"
            onclick={() => void mapStore.setMapSource('slam')}
          >
            Robot SLAM
          </button>
          <button
            class="flex h-9 flex-1 items-center justify-center rounded-[--radius-control] px-1.5
                   {mapStore.mapSource === 'optimized'
                     ? 'bg-surface-2 text-fg'
                     : 'text-fg-muted hover:bg-surface-2'}"
            title="The same keyframes posed by the collaborative pose-graph solver"
            onclick={() => void mapStore.setMapSource('optimized')}
          >
            Optimised
          </button>
        </div>
        {#if mapStore.unmergedScopes.length}
          <div class="mb-1 rounded-[--radius-control] bg-surface-2 px-1.5 py-1 text-[9px] text-fg-dim">
            Not merged into the fleet map:
            <span class="font-medium text-fg-muted">
              {mapStore.unmergedScopes
                .flatMap((scope) => scope.robots)
                .map(robotDisplayName)
                .join(', ')}
            </span>
          </div>
        {/if}

        <div class="my-2 border-t border-border"></div>
        <div class="mb-1 text-[9px] font-semibold uppercase tracking-[0.08em] text-fg-dim">
          Map data
        </div>
        <button
          class="flex h-9 w-full items-center gap-2 rounded-[--radius-control] px-1.5 text-fg-muted
                 hover:bg-surface-2 disabled:cursor-not-allowed disabled:opacity-35"
          title={resetRobot
            ? resetRobotBlocked
              ? 'Stop this robot before resetting its map'
              : `Reset only ${robotDisplayName(resetRobot.robot_id)}`
            : 'Select exactly one mapping robot'}
          disabled={resetRobotBlocked}
          onclick={() => resetRobotId && void resetMaps(resetRobotId)}
        >
          <RotateCcw class="h-3.5 w-3.5" />
          {resetPending && resetRobotId ? 'Resetting…' : resetRobotId ? `Reset ${robotDisplayName(resetRobotId)}` : 'Reset selected map'}
        </button>
        <button
          class="flex h-9 w-full items-center gap-2 rounded-[--radius-control] px-1.5 text-danger
                 hover:bg-danger/8 disabled:cursor-not-allowed disabled:opacity-35"
          title={resetAllBlocked ? 'Stop all navigating robots before resetting maps' : 'Reset every accumulated map'}
          disabled={resetAllBlocked}
          onclick={() => void resetMaps()}
        >
          <Trash2 class="h-3.5 w-3.5" /> Reset all maps
        </button>
        {#if resetError}
          <div class="mt-1 rounded-[--radius-control] bg-danger/8 px-1.5 py-1 text-danger">{resetError}</div>
        {/if}

        {#if mapStore.status}
          <div class="my-2 border-t border-border"></div>
          <div class="flex items-center justify-between text-fg-dim">
            <span>Reference frame</span>
            <span class="font-medium text-fg-muted">
              {mapStore.status.reference ? robotDisplayName(mapStore.status.reference) : '—'}
            </span>
          </div>
          {#each registrationEntries as [robotId, item]}
            <div class="mt-1 flex items-start justify-between gap-2 text-fg-dim">
              <span>{robotDisplayName(robotId)}</span>
              <span
                class="max-w-36 text-right font-medium {item.accepted
                  ? 'text-ok'
                  : item.rejection?.startsWith('outside')
                    ? 'text-warn'
                    : 'text-fg-muted'}"
                title={item.accepted && item.misses > 0
                  ? `${item.misses} ambiguous frame(s) since the last accepted match; ` +
                    'still merging on the transform from that match'
                  : (item.rejection ??
                      `score ${item.score} · shared area ${(item.support * 100).toFixed(0)}%` +
                        ` · rival rotation ${(item.yaw_ratio * 100).toFixed(0)}%`)}
              >
                {item.accepted && item.misses > 0
                  ? 'Holding last match'
                  : item.accepted
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

  {#if showNetwork && mapStore.viewMode === 'local' && mapStore.networkLayer}
    <div
      class="pointer-events-none absolute bottom-11 left-3 z-20 rounded-[--radius-control] border border-border
             bg-surface/90 px-2.5 py-1.5 text-[9px] text-fg-dim shadow-sm backdrop-blur-xl"
    >
      <div class="mb-1 flex items-center justify-between gap-4">
        <span class="font-semibold uppercase tracking-[0.07em]">Wi-Fi quality</span>
        <span class="font-mono text-fg-muted">
          {viewedNetwork ? `${Math.round(viewedNetwork.quality_pct)}% · ${Math.round(viewedNetwork.rssi_dbm)} dBm` : 'history'}
        </span>
      </div>
      <div
        class="h-1.5 w-44 rounded-full"
        style="background:linear-gradient(90deg, rgb(210 48 115), rgb(245 190 60), rgb(31 158 137))"
      ></div>
      <div class="mt-0.5 flex justify-between"><span>Poor</span><span>Fair</span><span>Good</span></div>
    </div>
  {/if}

  <!-- View controls keep the same meaning in 2D and 3D. -->
  <div class="panel-glow absolute bottom-3 right-3 z-20 flex flex-col overflow-hidden rounded-[--radius-panel] border border-transparent bg-surface/95 p-1">
    <button
      title="Follow fleet"
      class="grid h-10 w-10 touch-target place-items-center rounded-[--radius-control]
             transition-colors hover:bg-surface-2
             {follow ? 'bg-accent-container text-accent-container-fg' : 'text-fg-muted'}"
      onclick={toggleFollow}
    >
      <Locate class="h-4 w-4" />
    </button>
    <button
      title="Centre selected robots"
      disabled={fleet.selected.length === 0}
      class="grid h-10 w-10 touch-target place-items-center rounded-[--radius-control] text-fg-muted
             transition-colors hover:bg-surface-2 disabled:opacity-35"
      onclick={centreSelectedForView}
    >
      <Focus class="h-4 w-4" />
    </button>
    <button
      title="Zoom in"
      class="grid h-10 w-10 touch-target place-items-center rounded-[--radius-control] text-fg-muted
             transition-colors hover:bg-surface-2"
      onclick={() => zoomForView(1.25)}
    >
      <Plus class="h-4 w-4" />
    </button>
    <button
      title="Zoom out"
      class="grid h-10 w-10 touch-target place-items-center rounded-[--radius-control] text-fg-muted
             transition-colors hover:bg-surface-2"
      onclick={() => zoomForView(1 / 1.25)}
    >
      <Minus class="h-4 w-4" />
    </button>
    <button
      title="Fit fleet"
      class="grid h-10 w-10 touch-target place-items-center rounded-[--radius-control] text-fg-muted transition-colors
             hover:bg-surface-2"
      onclick={fitView}
    >
      <Maximize2 class="h-4 w-4" />
    </button>
  </div>

  <!-- cursor readout -->
  <div
    class="pointer-events-none absolute bottom-3 left-3 z-20 flex items-center gap-2 rounded-[--radius-control]
           border border-transparent bg-surface/92 px-3 py-1.5 text-[10px] tabular text-fg-dim
           shadow-[0_2px_8px_-5px_rgb(25_32_42/0.35)] backdrop-blur-xl"
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
