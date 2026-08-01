<script lang="ts">
  import { Crosshair, Maximize2, Minus, Plus, Locate } from 'lucide-svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { mapStore } from '$lib/stores/mapstore.svelte';
  import { session } from '$lib/stores/session.svelte';
  import { actions } from '$lib/api/connection';

  let host = $state<HTMLDivElement | null>(null);
  let canvas = $state<HTMLCanvasElement | null>(null);
  let view = $state({ scale: 0.55, tx: 0, ty: 0, initialised: false });
  let follow = $state(true);
  let cursorWorld = $state<{ x: number; y: number } | null>(null);

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

  function centreOnFleet() {
    const info = mapStore.info;
    if (!info || !canvas || fleet.count === 0) return;
    let sx = 0,
      sy = 0,
      n = 0;
    for (const r of fleet.robots) {
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
    ctx.fillStyle = 'var(--color-bg)';
    ctx.fillStyle = '#0a0e14';
    ctx.fillRect(0, 0, w, h);

    const info = mapStore.info;
    const grid = mapStore.canvas;

    if (!view.initialised && info && fleet.count) {
      view.initialised = true;
      centreOnFleet();
    }
    if (follow) centreOnFleet();

    // occupancy grid
    if (grid && info) {
      ctx.imageSmoothingEnabled = view.scale < 1;
      ctx.drawImage(grid, view.tx, view.ty, info.width * view.scale, info.height * view.scale);
    }

    // detections
    for (const d of session.detections) {
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
    for (const r of fleet.robots) {
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
      if (t.length > 1) {
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
      ctx.rotate(-r.pose.yaw);
      ctx.beginPath();
      ctx.moveTo(11, 0);
      ctx.lineTo(-7, 7);
      ctx.lineTo(-4, 0);
      ctx.lineTo(-7, -7);
      ctx.closePath();
      ctx.fillStyle = color;
      ctx.fill();
      ctx.strokeStyle = '#0a0e14';
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.restore();

      // label
      ctx.font = '600 10px ui-sans-serif, system-ui';
      ctx.fillStyle = color;
      ctx.textAlign = 'center';
      ctx.fillText(r.robot_id.replace(/^robot_/, 'R'), sx, sy - 18);
    }

    // scale bar
    if (info) {
      const metres = 5;
      const px = (metres / info.resolution) * view.scale;
      if (px > 24 && px < w * 0.6) {
        ctx.strokeStyle = '#5b7089';
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
        ctx.fillStyle = '#8ba0bb';
        ctx.textAlign = 'left';
        ctx.fillText(`${metres} m`, 16, h - 26);
      }
    }
  }

  // Redraw whenever the map, fleet, or view changes.
  $effect(() => {
    void mapStore.revision;
    void fleet.robots;
    void view.scale;
    void view.tx;
    void view.ty;
    void session.detections;
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

    // Tap to send a goal to every selected, navigation-capable robot.
    const rect = canvas.getBoundingClientRect();
    const g = gridOf(e.clientX - rect.left, e.clientY - rect.top);
    const world = mapStore.gridToWorld(g.gx, g.gy);
    if (!world) return;
    const targets = fleet.selected.filter((id) => fleet.can(id, 'navigate'));
    for (const id of targets) actions.setGoal(id, world);
  }

  function onWheel(e: WheelEvent) {
    e.preventDefault();
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    zoomBy(e.deltaY < 0 ? 1.12 : 1 / 1.12, e.clientX - rect.left, e.clientY - rect.top);
  }

  const canGoal = $derived(fleet.selected.filter((id) => fleet.can(id, 'navigate')).length);
</script>

<div class="relative h-full w-full overflow-hidden rounded-[--radius-card] border border-border bg-bg">
  <div bind:this={host} class="absolute inset-0">
    <canvas
      bind:this={canvas}
      class="h-full w-full touch-none"
      style="cursor:{canGoal ? 'crosshair' : 'grab'}"
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
  {#if canGoal > 0}
    <div
      class="pointer-events-none absolute left-1/2 top-3 -translate-x-1/2 rounded-lg border
             border-accent/30 bg-accent/10 px-3 py-1.5 text-[11px] font-medium text-accent
             backdrop-blur"
    >
      <Crosshair class="mr-1 inline h-3 w-3" />
      Tap to set goal for {canGoal} robot{canGoal > 1 ? 's' : ''}
    </div>
  {/if}

  <!-- view controls -->
  <div class="absolute bottom-3 right-3 flex flex-col gap-1.5">
    <button
      title="Follow fleet"
      class="grid h-11 w-11 touch-target place-items-center rounded-xl border bg-surface/90
             backdrop-blur transition-colors hover:bg-surface-2
             {follow ? 'border-accent text-accent' : 'border-border text-fg-muted'}"
      onclick={() => {
        follow = !follow;
        if (follow) centreOnFleet();
      }}
    >
      <Locate class="h-4 w-4" />
    </button>
    <button
      title="Zoom in"
      class="grid h-11 w-11 touch-target place-items-center rounded-xl border border-border
             bg-surface/90 text-fg-muted backdrop-blur transition-colors hover:bg-surface-2"
      onclick={() => zoomBy(1.25)}
    >
      <Plus class="h-4 w-4" />
    </button>
    <button
      title="Zoom out"
      class="grid h-11 w-11 touch-target place-items-center rounded-xl border border-border
             bg-surface/90 text-fg-muted backdrop-blur transition-colors hover:bg-surface-2"
      onclick={() => zoomBy(1 / 1.25)}
    >
      <Minus class="h-4 w-4" />
    </button>
    <button
      title="Fit fleet"
      class="grid h-11 w-11 touch-target place-items-center rounded-xl border border-border
             bg-surface/90 text-fg-muted backdrop-blur transition-colors hover:bg-surface-2"
      onclick={() => {
        view.scale = 0.55;
        centreOnFleet();
        follow = true;
      }}
    >
      <Maximize2 class="h-4 w-4" />
    </button>
  </div>

  <!-- cursor readout -->
  {#if cursorWorld}
    <div
      class="pointer-events-none absolute bottom-3 left-3 rounded-md border border-border
             bg-surface/80 px-2 py-1 text-[10px] tabular text-fg-dim backdrop-blur"
    >
      {cursorWorld.x.toFixed(1)}, {cursorWorld.y.toFixed(1)} m
    </div>
  {/if}
</div>
