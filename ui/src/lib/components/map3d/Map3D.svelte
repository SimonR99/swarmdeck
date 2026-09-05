<script lang="ts">
  /**
   * StarCraft-like 3D Tactical Map, powered by Three.js.
   *
   * Renders the true 3D map of the robots (accumulated voxel terrain with real heights,
   * walls, obstacles, and ceiling) using real point data from the robots, NOT an extrusion
   * of the 2D map.
   *
   * Features:
   * - Extruded 3D chevron robot symbols in team colors with bevels and headlights
   * - Interactive robot selection (click / shift-click)
   * - Interactive navigation goal targeting on the 3D ground plane
   * - Ceiling cutoff slider to remove roofs and inspect interior rooms
   * - 3D global & local routes, movement trails, sensor arcs, and footprints
   * - 3D holographic waypoint beacons and detection crystals
   * - 3D tactical StarCraft build grid, costmap, and network heatmaps
   */
  import { onMount, untrack } from 'svelte';
  import { inflate } from 'pako';
  import * as THREE from 'three';
  import {
    Box,
    Check,
    Compass,
    Crosshair,
    Eye,
    Layers,
    Sliders,
    Sparkles,
    X
  } from 'lucide-svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { mapStore } from '$lib/stores/mapstore.svelte';
  import { navigation } from '$lib/stores/navigation.svelte';
  import { review } from '$lib/stores/review.svelte';
  import { detectionCatalog } from '$lib/stores/detection.svelte';
  import { actions } from '$lib/api/connection';
  import { robotDisplayName } from '$lib/robotDisplayName';
  import { StarcraftScene } from './StarcraftScene';
  import type { MapRobot } from '../map2d/mapLayers';

  let {
    active = false,
    follow = false,
    showGrid = true,
    showTrails = true,
    showLabels = true,
    showSensors = false,
    showPlans = true,
    showNetwork = false,
    showCostmap = false,
    costmapKind = 'global',
    trails = new Map<string, { x: number; y: number }[]>(),
    onCursorChange
  }: {
    active?: boolean;
    follow?: boolean;
    showGrid?: boolean;
    showTrails?: boolean;
    showLabels?: boolean;
    showSensors?: boolean;
    showPlans?: boolean;
    showNetwork?: boolean;
    showCostmap?: boolean;
    costmapKind?: 'global' | 'local';
    trails?: Map<string, { x: number; y: number }[]>;
    onCursorChange?: (coords: { x: number; y: number } | null) => void;
  } = $props();

  let canvas = $state<HTMLCanvasElement | null>(null);
  let scene: StarcraftScene | null = null;

  // Cloud & Terrain State
  let points = $state(0);
  let voxels = $state(0);
  let robotsOnCloud = $state<string[]>([]);
  let error = $state<string | null>(null);
  let isMock = $state(false);

  // Ceiling Slider State
  let ceilingMax = $state(3.2);
  let ceilingMin = $state(0.0);
  let ceilingCutoff = $state(2.3); // Initial cutoff: open interior view
  let ceilingSliderOpen = $state(true);

  // Interaction State
  let dragging = $state(false);
  let isPanning = false;
  let pointerDownPos: { x: number; y: number } | null = null;
  let dragged = false;
  let cursor3D = $state<{ x: number; y: number; z: number } | null>(null);
  let detectionScreenPos = $state<{ sx: number; sy: number } | null>(null);

  function robotsOnMap(): MapRobot[] {
    if (mapStore.viewMode === 'local' && mapStore.viewRobot) {
      if (!fleet.isEnabled(mapStore.viewRobot)) return [];
      const robot = fleet.get(mapStore.viewRobot);
      return robot ? [robot] : [];
    }
    const members = mapStore.status?.global_members;
    if (members && members.length > 0) {
      return fleet.robots.filter(
        (robot) => members.includes(robot.robot_id) && fleet.isEnabled(robot.robot_id)
      );
    }
    return fleet.robots.filter((robot) => fleet.isEnabled(robot.robot_id));
  }

  // Public camera control methods for ViewControls toolbar
  export function centreFleet() {
    if (!scene) return;
    scene.centreRobots(robotsOnMap());
    scene.render();
  }

  export function centreSelected() {
    if (!scene) return;
    scene.centreRobots(robotsOnMap(), new Set(fleet.selected));
    scene.render();
  }

  export function zoomBy(factor: number) {
    if (!scene) return;
    scene.zoomBy(factor);
    scene.render();
  }

  export function rotateBy(angleDelta: number) {
    if (!scene) return;
    scene.rotateBy(angleDelta);
    scene.render();
  }

  export function resetRotation() {
    if (!scene) return;
    scene.resetRotation();
    scene.render();
  }

  export function fitCloud() {
    if (!scene) return;
    scene.fitCloud();
    scene.render();
  }

  export function setCeiling(height: number) {
    ceilingCutoff = height;
    if (scene) {
      scene.setCeiling(height);
      scene.render();
    }
  }

  export function setIsometricView() {
    if (!scene) return;
    scene.yaw = -0.785; // 45 deg
    scene.pitch = 0.96; // 55 deg
    scene.render();
  }

  export function cutCeilingQuick() {
    ceilingCutoff = Math.max(ceilingMin, Math.min(2.1, ceilingMax - 0.4));
    if (scene) {
      scene.setCeiling(ceilingCutoff);
      scene.render();
    }
  }

  export function resetCeiling() {
    ceilingCutoff = ceilingMax + 0.2;
    if (scene) {
      scene.setCeiling(ceilingCutoff);
      scene.render();
    }
  }

  async function fetchCloud() {
    if (!scene) return;
    try {
      const url =
        mapStore.viewMode === 'local' && mapStore.viewRobot
          ? `/api/map/cloud?robot_id=${encodeURIComponent(mapStore.viewRobot)}`
          : '/api/map/cloud';
      const response = await fetch(url, { cache: 'no-store' });
      if (!response.ok) throw new Error(`cloud ${response.status}`);

      const total = Number(response.headers.get('X-Cloud-Points') ?? 0);
      const scale = Number(response.headers.get('X-Cloud-Scale') ?? 0.01);
      const names = (response.headers.get('X-Cloud-Robots') ?? '')
        .split(',')
        .filter(Boolean);

      if (total > 0) {
        const raw = inflate(new Uint8Array(await response.arrayBuffer()));
        const xyz = new Int16Array(raw.buffer, raw.byteOffset, total * 3);
        const owners = new Uint8Array(raw.buffer, raw.byteOffset + total * 6, total);

        const keep = names.map((id) => fleet.isEnabled(id));
        let kept = 0;
        for (let i = 0; i < total; i++) if (keep[owners[i]]) kept++;

        const positions = new Float32Array(kept * 3);
        const keptOwners = new Uint8Array(kept);
        let o = 0;
        for (let i = 0; i < total; i++) {
          if (!keep[owners[i]]) continue;
          positions[o * 3] = xyz[i * 3] * scale;
          positions[o * 3 + 1] = xyz[i * 3 + 1] * scale;
          positions[o * 3 + 2] = xyz[i * 3 + 2] * scale;
          keptOwners[o] = owners[i];
          o++;
        }

        const colors = names.map((id) => fleet.colorOf(id));
        const bounds = scene.terrain.buildFromPoints(positions, kept, keptOwners, colors);

        points = kept;
        voxels = scene.terrain.voxelCount;
        robotsOnCloud = names.filter((_, i) => keep[i]);
        ceilingMin = bounds.minZ;
        ceilingMax = Math.max(bounds.maxZ, 2.5);
        if (ceilingCutoff > ceilingMax || ceilingCutoff < ceilingMin) {
          ceilingCutoff = Math.max(ceilingMin, ceilingMax - 0.4);
        }
        scene.setCeiling(ceilingCutoff);
        isMock = false;
        error = null;
      } else {
        // Mock 3D environment with real walls, rooms, obstacles, and ceiling
        const bounds = scene.terrain.buildMock3DEnvironment();
        points = 24000;
        voxels = scene.terrain.voxelCount;
        robotsOnCloud = fleet.robots.map((r) => r.robot_id);
        ceilingMin = bounds.minZ;
        ceilingMax = bounds.maxZ;
        ceilingCutoff = 2.2;
        scene.setCeiling(ceilingCutoff);
        isMock = true;
        error = null;
      }
      scene.render();
    } catch (e) {
      error = e instanceof Error ? e.message : String(e);
      // Even if fetch failed (e.g. mock mode without backend), present mock 3D base
      if (scene) {
        const bounds = scene.terrain.buildMock3DEnvironment();
        points = 24000;
        voxels = scene.terrain.voxelCount;
        robotsOnCloud = fleet.robots.map((r) => r.robot_id);
        ceilingMin = bounds.minZ;
        ceilingMax = bounds.maxZ;
        ceilingCutoff = 2.2;
        scene.setCeiling(ceilingCutoff);
        isMock = true;
        scene.render();
      }
    }
  }

  // Pointer & RTS Mouse Interaction
  function getNDC(e: PointerEvent): THREE.Vector2 {
    if (!canvas) return new THREE.Vector2(0, 0);
    const rect = canvas.getBoundingClientRect();
    const x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    const y = -(((e.clientY - rect.top) / rect.height) * 2 - 1);
    return new THREE.Vector2(x, y);
  }

  function onPointerDown(e: PointerEvent) {
    if (!canvas) return;
    canvas.setPointerCapture(e.pointerId);
    pointerDownPos = { x: e.clientX, y: e.clientY };
    dragged = false;
    dragging = true;
    isPanning = e.button === 2 || e.button === 1 || e.shiftKey;
  }

  function onPointerMove(e: PointerEvent) {
    if (!scene || !canvas) return;
    const ndc = getNDC(e);

    if (dragging && pointerDownPos) {
      const dx = e.clientX - pointerDownPos.x;
      const dy = e.clientY - pointerDownPos.y;
      if (Math.hypot(dx, dy) > 5) {
        dragged = true;
      }

      if (isPanning) {
        scene.panBy(-dx * 0.8, -dy * 0.8);
      } else {
        scene.yaw -= dx * 0.007;
        scene.pitch = Math.max(0.12, Math.min(1.48, scene.pitch + dy * 0.007));
      }
      pointerDownPos = { x: e.clientX, y: e.clientY };
      scene.render();
    } else {
      // Hover raycasting
      const groundHit = scene.raycastGround(ndc);
      if (groundHit) {
        cursor3D = { x: groundHit.x, y: groundHit.y, z: groundHit.z };
        onCursorChange?.({ x: groundHit.x, y: groundHit.y });

        // Update 3D target rally reticle when goal mode is armed
        if (navigation.goalMode) {
          scene.layers.cursorReticle.position.set(groundHit.x, groundHit.y, 0.02);
          scene.layers.cursorReticle.visible = true;
          scene.render();
        } else if (scene.layers.cursorReticle.visible) {
          scene.layers.cursorReticle.visible = false;
          scene.render();
        }
      } else {
        cursor3D = null;
        onCursorChange?.(null);
        if (scene.layers.cursorReticle.visible) {
          scene.layers.cursorReticle.visible = false;
          scene.render();
        }
      }
    }
  }

  function onPointerUp(e: PointerEvent) {
    const wasDrag = dragged;
    dragging = false;
    pointerDownPos = null;

    if (!scene || !canvas || wasDrag) return;

    const ndc = getNDC(e);

    // Goal Mode Navigation Dispatch
    if (navigation.goalMode) {
      const ground = scene.raycastGround(ndc);
      if (ground) {
        const world = { x: ground.x, y: ground.y };
        const targets = fleet.selected.filter((id) => fleet.can(id, 'navigate'));
        for (const id of targets) {
          actions.setGoal(id, world);
        }
        if (targets.length) {
          navigation.finishGoal(world);
        }
      }
      return;
    }

    // Interactive Object / Robot Picking
    const hit = scene.raycastInteractive(ndc);
    if (hit.robotId) {
      fleet.select(hit.robotId, e.shiftKey);
      actions.selectRobots(fleet.selected);
      scene.render();
      return;
    }

    if (hit.detectionId) {
      if (review.selected === hit.detectionId) {
        review.select(null);
      } else {
        review.select(hit.detectionId);
        const activeObj = review.proposalOf(hit.detectionId) ?? review.entityOf(hit.detectionId);
        const robotId = activeObj?.robot_ids?.[0];
        if (robotId) actions.focusRobot(robotId);
      }
      scene.render();
      return;
    }

    // Clicking empty space deselects detection
    if (review.selected) {
      review.select(null);
    }
  }

  function onWheel(e: WheelEvent) {
    e.preventDefault();
    if (!scene) return;
    scene.zoomBy(e.deltaY < 0 ? 1.15 : 1 / 1.15);
    scene.render();
  }

  function onContextMenu(e: MouseEvent) {
    e.preventDefault(); // Prevent browser right-click menu
  }

  // Animation & Rendering Loop
  let rafId = 0;
  function tick(timestamp: number) {
    if (!scene || !active) return;
    const time = timestamp * 0.001;

    // Center on fleet if follow mode is active
    if (follow) {
      scene.centreRobots(robotsOnMap());
    }

    const robots = robotsOnMap();
    scene.robotManager.update(robots, {
      showSensors,
      showLabels,
      time
    });

    scene.layers.update({
      robots,
      trails,
      showGrid,
      showTrails,
      showPlans,
      showSensors,
      showCostmap,
      showNetwork,
      costmapKind,
      time
    });

    // Update active detection screen projection for popover
    const activeDetId = review.selected ?? review.focused;
    if (activeDetId) {
      const activeObj = review.proposalOf(activeDetId) ?? review.entityOf(activeDetId);
      if (activeObj) {
        const v = new THREE.Vector3(activeObj.position.x, activeObj.position.y, 0.4);
        const screenProj = scene.worldToScreen(v);
        detectionScreenPos = screenProj.visible ? { sx: screenProj.sx, sy: screenProj.sy } : null;
      } else {
        detectionScreenPos = null;
      }
    } else {
      detectionScreenPos = null;
    }

    scene.render();
    rafId = requestAnimationFrame(tick);
  }

  $effect(() => {
    // React to ceiling cutoff slider
    if (scene) {
      scene.setCeiling(ceilingCutoff);
    }
  });

  onMount(() => {
    if (!canvas) return;

    try {
      scene = new StarcraftScene(canvas);
      scene.resize();
    } catch (err) {
      error = err instanceof Error ? err.message : String(err);
      return;
    }

    void fetchCloud();
    const poll = window.setInterval(() => active && void fetchCloud(), 5000);

    const ro = new ResizeObserver(() => {
      scene?.resize();
      scene?.render();
    });
    ro.observe(canvas);

    rafId = requestAnimationFrame(tick);

    return () => {
      window.clearInterval(poll);
      if (rafId) cancelAnimationFrame(rafId);
      ro.disconnect();
      scene?.dispose();
    };
  });
</script>

<div class="relative h-full w-full select-none overflow-hidden bg-[#0c0f14]">
  <canvas
    bind:this={canvas}
    class="h-full w-full touch-none {navigation.goalMode ? 'cursor-crosshair' : dragging ? 'cursor-grabbing' : 'cursor-grab'}"
    onpointerdown={onPointerDown}
    onpointermove={onPointerMove}
    onpointerup={onPointerUp}
    onpointercancel={onPointerUp}
    onwheel={onWheel}
    oncontextmenu={onContextMenu}
  ></canvas>

  <!-- StarCraft Tactical Status HUD (Top Left) -->
  <div
    class="panel-glow pointer-events-none absolute left-3 top-3 z-20 flex flex-col gap-1 rounded-[--radius-control]
           border border-border/80 bg-surface/92 px-3 py-2 text-[10px] text-fg-dim shadow-xl backdrop-blur-xl"
  >
    <div class="flex items-center gap-2">
      <span class="inline-flex h-2 w-2 rounded-full {error ? 'bg-warn' : isMock ? 'bg-accent' : 'bg-ok'} animate-pulse"></span>
      <span class="font-semibold uppercase tracking-wider text-fg">
        {isMock ? 'StarCraft 3D Simulation' : 'StarCraft 3D Tactical Map'}
      </span>
    </div>
    {#if error}
      <span class="text-warn">{error}</span>
    {:else}
      <div class="flex items-center gap-2 font-mono text-fg-muted">
        <span>{voxels.toLocaleString()} voxels</span>
        <span class="text-border">·</span>
        <span>{robotsOnCloud.length} robot{robotsOnCloud.length === 1 ? '' : 's'}</span>
        {#if cursor3D}
          <span class="text-border">·</span>
          <span class="text-accent font-medium">({cursor3D.x.toFixed(1)}, {cursor3D.y.toFixed(1)}) m</span>
        {/if}
      </div>
      <div class="text-[9px] text-fg-dim/80">
        Left-drag to orbit · Right-drag to pan · Scroll to zoom
      </div>
    {/if}
  </div>

  <!-- Ceiling Cutoff Tactical Slider (Top Right) -->
  <div class="absolute right-3 top-3 z-20 flex items-center gap-2">
    <div
      class="panel-glow flex items-center gap-2 rounded-[--radius-control] border border-border/90
             bg-surface/95 px-3 py-1.5 shadow-2xl backdrop-blur-xl"
    >
      <div class="flex items-center gap-1 text-[10px] font-semibold uppercase tracking-wider text-accent">
        <Sliders class="h-3 w-3" />
        <span>Ceiling</span>
      </div>

      <input
        type="range"
        min={ceilingMin}
        max={ceilingMax + 0.2}
        step="0.05"
        bind:value={ceilingCutoff}
        class="h-1.5 w-28 cursor-pointer appearance-none rounded-full bg-surface-2 accent-accent"
        title="Slide to remove ceiling / roof and view interior"
      />

      <span class="min-w-10 font-mono text-[10px] font-semibold text-fg">
        {ceilingCutoff.toFixed(2)}m
      </span>

      <button
        class="rounded-[--radius-control] bg-surface-2 px-2 py-0.5 text-[9px] font-medium text-fg-muted
               transition-colors hover:bg-accent-container hover:text-accent-container-fg"
        title="Slice off the roof to view room interior"
        onclick={cutCeilingQuick}
      >
        Cut Roof
      </button>

      <button
        class="rounded-[--radius-control] bg-surface-2 px-2 py-0.5 text-[9px] font-medium text-fg-muted
               transition-colors hover:bg-surface hover:text-fg"
        title="Restore full height ceiling"
        onclick={resetCeiling}
      >
        Full
      </button>

      <button
        class="rounded-[--radius-control] bg-surface-2 px-1.5 py-0.5 text-[9px] font-medium text-accent
               transition-colors hover:bg-accent-container hover:text-accent-container-fg"
        title="Set classic 45° RTS Isometric perspective"
        onclick={setIsometricView}
      >
        <Compass class="h-3 w-3" />
      </button>
    </div>
  </div>

  <!-- Goal Navigation Mode Banner -->
  {#if navigation.goalMode}
    {@const canGoal = fleet.selected.filter((id) => fleet.can(id, 'navigate')).length}
    <div
      class="pointer-events-none absolute left-1/2 top-3 -translate-x-1/2 rounded-[--radius-control] border
             border-accent/40 bg-surface/95 px-4 py-2 text-[11px] font-medium text-accent
             shadow-2xl backdrop-blur-xl"
    >
      <Crosshair class="mr-1.5 inline h-3.5 w-3.5 animate-spin" />
      Click 3D ground destination for {canGoal} robot{canGoal > 1 ? 's' : ''} · Esc to cancel
    </div>
  {/if}

  <!-- Active Reviewed Object Detection Preview Card in 3D -->
  {#if (review.selected || review.focused) && detectionScreenPos}
    {@const activeDetectionId = review.selected ?? review.focused}
    {@const activeObj = activeDetectionId
      ? (review.proposalOf(activeDetectionId) ?? review.entityOf(activeDetectionId))
      : null}
    {#if activeObj && activeObj.image}
      <div
        class="pointer-events-none absolute z-25 flex -translate-x-1/2 -translate-y-full flex-col items-center pb-3.5"
        style="left: {detectionScreenPos.sx}px; top: {detectionScreenPos.sy}px;"
      >
        <div
          class="flex flex-col items-center overflow-hidden rounded-xl border border-border/80
                 bg-surface/95 p-2 shadow-2xl backdrop-blur-xl"
        >
          <img
            src={activeObj.image}
            alt="{detectionCatalog.labelOf(activeObj.class)} detection crop"
            class="h-32 w-32 rounded-lg object-cover shadow-sm"
          />
          <div class="mt-1 flex w-full items-center justify-between px-1 text-[10px] font-semibold text-fg">
            <span>{detectionCatalog.labelOf(activeObj.class)}</span>
            <span class="font-normal text-fg-dim">
              {`${Math.round(activeObj.best_score * 100)}%`}
            </span>
          </div>
        </div>
        <div class="-mt-1 h-2 w-2 rotate-45 border-b border-r border-border/80 bg-surface shadow-sm"></div>
      </div>
    {/if}
  {/if}
</div>
