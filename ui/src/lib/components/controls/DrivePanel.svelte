<script lang="ts">
  import {
    ArrowDown,
    ArrowLeft,
    ArrowRight,
    ArrowUp,
    CircleSlash,
    Crosshair,
    Octagon
  } from 'lucide-svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { navigation } from '$lib/stores/navigation.svelte';
  import { actions } from '$lib/api/connection';
  import Badge from '../ui/Badge.svelte';

  const activeId = $derived(fleet.selected[0] ?? null);
  const robot = $derived(activeId ? fleet.get(activeId) : undefined);
  const canDrive = $derived(Boolean(activeId && fleet.can(activeId, 'navigate') && robot?.online));

  let driveTimer: number | null = null;
  let driving = false;
  let driveLabel = $state('Ready');

  function stopTimer() {
    if (driveTimer !== null) window.clearInterval(driveTimer);
    driveTimer = null;
  }

  function stopDrive(force = false) {
    const wasDriving = driving;
    stopTimer();
    driving = false;
    if (activeId && (wasDriving || force)) actions.drive(activeId, 0, 0);
    driveLabel = 'Ready';
  }

  function startDrive(linear: number, angular: number, label: string, event?: PointerEvent) {
    if (!activeId || !canDrive) return;
    event?.preventDefault();
    stopTimer();
    navigation.cancelGoalMode();
    actions.drive(activeId, linear, angular);
    driving = true;
    driveLabel = label;
    driveTimer = window.setInterval(() => {
      if (activeId) actions.drive(activeId, linear, angular);
    }, 120);
  }

  function hardStop() {
    stopDrive(true);
    if (activeId) actions.cancelGoal(activeId);
  }

  function keyVector(key: string): [number, number, string] | null {
    if (key === 'ArrowUp' || key.toLowerCase() === 'w') return [0.28, 0, 'Forward'];
    if (key === 'ArrowDown' || key.toLowerCase() === 's') return [-0.22, 0, 'Reverse'];
    if (key === 'ArrowLeft' || key.toLowerCase() === 'a') return [0, 0.8, 'Turn left'];
    if (key === 'ArrowRight' || key.toLowerCase() === 'd') return [0, -0.8, 'Turn right'];
    return null;
  }

  $effect(() => {
    const id = activeId;
    const down = (event: KeyboardEvent) => {
      if (event.repeat || event.target instanceof HTMLInputElement) return;
      const vector = keyVector(event.key);
      if (!vector) return;
      event.preventDefault();
      startDrive(...vector);
    };
    const up = (event: KeyboardEvent) => {
      if (keyVector(event.key)) stopDrive();
    };
    const pointerUp = () => stopDrive();
    window.addEventListener('keydown', down);
    window.addEventListener('keyup', up);
    window.addEventListener('pointerup', pointerUp);
    window.addEventListener('pointercancel', pointerUp);
    window.addEventListener('blur', pointerUp);
    return () => {
      stopTimer();
      if (id && driving) actions.drive(id, 0, 0);
      driving = false;
      window.removeEventListener('keydown', down);
      window.removeEventListener('keyup', up);
      window.removeEventListener('pointerup', pointerUp);
      window.removeEventListener('pointercancel', pointerUp);
      window.removeEventListener('blur', pointerUp);
    };
  });
</script>

<section class="panel-glow flex shrink-0 flex-col rounded-[--radius-card] border border-border bg-surface">
  <header class="flex h-10 shrink-0 items-center justify-between border-b border-border px-3">
    <div>
      <!--
        Name the robot being driven. The controls act on `fleet.selected[0]`, so
        with several robots selected the arrows move whichever happens to be
        first — which presents as "this robot will not move" while a different
        one drives off. Saying which one costs nothing and removes the whole
        class of confusion.
      -->
      <div class="flex items-center gap-1.5 text-[11px] font-semibold text-fg">
        Robot control
        {#if activeId}
          <span class="font-medium" style="color:{fleet.colorOf(activeId)}">
            {activeId.replace(/^robot_/, 'R')}
          </span>
        {/if}
      </div>
      <div class="mt-0.5 text-[9px] text-fg-dim">
        {#if fleet.selected.length > 1}
          Drives {activeId?.replace(/^robot_/, 'R')} only · {fleet.selected.length} selected
        {:else}
          Hold to drive · release to stop
        {/if}
      </div>
    </div>
    <Badge tone={robot?.mode === 'teleop' ? 'accent' : robot?.nav_status === 'active' ? 'ok' : 'neutral'}>
      {robot?.mode === 'teleop' ? driveLabel : robot?.nav_status === 'active' ? 'NAV ACTIVE' : 'STANDBY'}
    </Badge>
  </header>

  {#if robot && activeId}
    <div class="grid grid-cols-[1fr_108px] gap-3 p-3">
      <div class="min-w-0">
        <div class="mb-2 flex items-center justify-between">
          <span class="text-[10px] font-semibold uppercase tracking-[0.06em] text-fg-muted">Manual</span>
          <span class="text-[9px] text-fg-dim">WASD / arrows</span>
        </div>

        <div class="grid w-[116px] grid-cols-3 gap-1">
          <span></span>
          <button
            aria-label="Drive forward"
            disabled={!canDrive}
            class="drive-key"
            onpointerdown={(event) => startDrive(0.28, 0, 'Forward', event)}
          ><ArrowUp class="h-4 w-4" /></button>
          <span></span>
          <button
            aria-label="Turn left"
            disabled={!canDrive}
            class="drive-key"
            onpointerdown={(event) => startDrive(0, 0.8, 'Turn left', event)}
          ><ArrowLeft class="h-4 w-4" /></button>
          <button aria-label="Stop robot" class="drive-key text-danger" onclick={hardStop}>
            <Octagon class="h-4 w-4" />
          </button>
          <button
            aria-label="Turn right"
            disabled={!canDrive}
            class="drive-key"
            onpointerdown={(event) => startDrive(0, -0.8, 'Turn right', event)}
          ><ArrowRight class="h-4 w-4" /></button>
          <span></span>
          <button
            aria-label="Drive backward"
            disabled={!canDrive}
            class="drive-key"
            onpointerdown={(event) => startDrive(-0.22, 0, 'Reverse', event)}
          ><ArrowDown class="h-4 w-4" /></button>
        </div>
      </div>

      <div class="border-l border-border pl-3">
        <div class="mb-2 text-[10px] font-semibold uppercase tracking-[0.06em] text-fg-muted">
          Point nav
        </div>
        <button
          disabled={!canDrive}
          class="flex h-9 w-full items-center justify-center gap-1.5 rounded-[4px] border text-[10px]
                 font-semibold transition-colors disabled:opacity-40
                 {navigation.goalMode
            ? 'border-accent bg-accent text-white'
            : 'border-border bg-surface text-fg hover:bg-surface-2'}"
          onclick={() => navigation.toggleGoalMode()}
        >
          <Crosshair class="h-3.5 w-3.5" />
          {navigation.goalMode ? 'Armed' : 'Set goal'}
        </button>
        <button
          disabled={robot.nav_status !== 'active'}
          class="mt-1.5 flex h-8 w-full items-center justify-center gap-1 rounded-[4px] border
                 border-border text-[9px] font-medium text-fg-muted hover:bg-surface-2
                 disabled:opacity-40"
          onclick={() => actions.cancelGoal(activeId)}
        >
          <CircleSlash class="h-3 w-3" /> Cancel
        </button>
        <div class="mt-2 text-[9px] leading-4 text-fg-dim">
          {navigation.goalMode ? 'Select a free point on the map.' : 'Plan with Nav2 on the live map.'}
        </div>
      </div>
    </div>
  {:else}
    <div class="grid flex-1 place-items-center px-4 text-center text-[10px] text-fg-dim">
      Select an online robot to enable controls
    </div>
  {/if}
</section>

<style>
  .drive-key {
    display: grid;
    width: 36px;
    height: 36px;
    place-items: center;
    border: 1px solid var(--color-border);
    border-radius: 4px;
    background: var(--color-surface);
    color: var(--color-fg-muted);
    transition: background 120ms, border-color 120ms, color 120ms;
    touch-action: none;
  }
  .drive-key:hover:not(:disabled) {
    background: var(--color-surface-2);
    border-color: var(--color-border-strong);
    color: var(--color-fg);
  }
  .drive-key:active:not(:disabled) {
    background: var(--color-surface-3);
  }
  .drive-key:disabled {
    opacity: 0.4;
  }
</style>
