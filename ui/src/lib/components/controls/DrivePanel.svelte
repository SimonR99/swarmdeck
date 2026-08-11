<script lang="ts">
  import {
    CircleSlash,
    Crosshair,
    Gauge,
    Octagon
  } from 'lucide-svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { navigation } from '$lib/stores/navigation.svelte';
  import { actions } from '$lib/api/connection';
  import { robotDisplayName } from '$lib/robotDisplayName';
  import Badge from '../ui/Badge.svelte';

  const activeId = $derived(fleet.selected[0] ?? null);
  const robot = $derived(activeId ? fleet.get(activeId) : undefined);
  const canDrive = $derived(Boolean(activeId && fleet.can(activeId, 'navigate') && robot?.online));

  type Direction = 'up' | 'down' | 'left' | 'right';

  const MAX_ANGULAR_SPEED = 0.8;
  const JOYSTICK_RADIUS = 45;
  const JOYSTICK_DEAD_ZONE = 0.06;

  let driveTimer: number | null = null;
  let driving = false;
  let driveLabel = $state('Ready');
  let maxSpeed = $state(0.3);
  let joystickX = $state(0);
  let joystickY = $state(0);
  let joystickPointer = $state<number | null>(null);
  let joystickElement = $state<HTMLButtonElement>();

  // Multiple keyboard inputs can be active at once, so diagonal WASD driving
  // remains available alongside the touch-first joystick.
  const keyDirs = new Set<Direction>();

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

  function vectorFor(dirs: Set<Direction>): [number, number] {
    let linear = 0;
    if (dirs.has('up') && !dirs.has('down')) linear = maxSpeed;
    else if (dirs.has('down') && !dirs.has('up')) linear = -maxSpeed;

    let angular = 0;
    if (dirs.has('left') && !dirs.has('right')) angular = MAX_ANGULAR_SPEED;
    else if (dirs.has('right') && !dirs.has('left')) angular = -MAX_ANGULAR_SPEED;

    return [linear, angular];
  }

  function joystickVector(): [number, number] {
    const stickDistance = Math.min(1, Math.hypot(joystickX, joystickY));
    if (stickDistance <= JOYSTICK_DEAD_ZONE) return [0, 0];

    // Remove one circular dead zone without changing the stick direction.
    // Applying a dead zone to X and Y independently makes diagonal motion
    // bend as either axis crosses its threshold. Past the centre zone, this
    // maps displacement linearly from 0 at its edge to 1 at full travel.
    const outputDistance =
      (stickDistance - JOYSTICK_DEAD_ZONE) / (1 - JOYSTICK_DEAD_ZONE);
    const scale = outputDistance / stickDistance;
    return [
      -joystickY * scale * maxSpeed,
      -joystickX * scale * MAX_ANGULAR_SPEED
    ];
  }

  function currentVector(): [number, number] {
    if (joystickPointer !== null) return joystickVector();
    return vectorFor(keyDirs);
  }

  function labelFor(linear: number, angular: number): string {
    const parts: string[] = [];
    if (linear > 0) parts.push('Forward');
    else if (linear < 0) parts.push('Reverse');
    if (angular > 0) parts.push('left');
    else if (angular < 0) parts.push('right');
    return parts.length ? parts.join(' ') : 'Ready';
  }

  function updateDrive() {
    if (!activeId || !canDrive) return;
    const [linear, angular] = currentVector();
    if (linear === 0 && angular === 0) {
      stopDrive();
      return;
    }
    navigation.cancelGoalMode();
    actions.drive(activeId, linear, angular);
    driving = true;
    driveLabel = labelFor(linear, angular);
    stopTimer();
    driveTimer = window.setInterval(() => {
      if (activeId) actions.drive(activeId, linear, angular);
    }, 120);
  }

  function hardStop() {
    keyDirs.clear();
    resetJoystick();
    stopDrive(true);
    if (activeId) actions.cancelGoal(activeId);
  }

  function keyDirection(key: string): Direction | null {
    if (key === 'ArrowUp' || key.toLowerCase() === 'w') return 'up';
    if (key === 'ArrowDown' || key.toLowerCase() === 's') return 'down';
    if (key === 'ArrowLeft' || key.toLowerCase() === 'a') return 'left';
    if (key === 'ArrowRight' || key.toLowerCase() === 'd') return 'right';
    return null;
  }

  function updateJoystick(event: PointerEvent) {
    if (event.pointerId !== joystickPointer || !joystickElement) return;
    event.preventDefault();
    const rect = joystickElement.getBoundingClientRect();
    let x = event.clientX - (rect.left + rect.width / 2);
    let y = event.clientY - (rect.top + rect.height / 2);
    const distance = Math.hypot(x, y);
    if (distance > JOYSTICK_RADIUS) {
      x = (x / distance) * JOYSTICK_RADIUS;
      y = (y / distance) * JOYSTICK_RADIUS;
    }
    joystickX = x / JOYSTICK_RADIUS;
    joystickY = y / JOYSTICK_RADIUS;
    updateDrive();
  }

  function startJoystick(event: PointerEvent) {
    if (!activeId || !canDrive) return;
    event.preventDefault();
    joystickPointer = event.pointerId;
    joystickElement?.setPointerCapture(event.pointerId);
    updateJoystick(event);
  }

  function resetJoystick(pointerId?: number) {
    if (pointerId !== undefined && pointerId !== joystickPointer) return;
    joystickPointer = null;
    joystickX = 0;
    joystickY = 0;
  }

  function changeMaxSpeed(event: Event) {
    maxSpeed = Number((event.currentTarget as HTMLInputElement).value);
    if (driving) updateDrive();
  }

  $effect(() => {
    const id = activeId;
    const down = (event: KeyboardEvent) => {
      if (event.repeat || event.target instanceof HTMLInputElement) return;
      const direction = keyDirection(event.key);
      if (!direction) return;
      event.preventDefault();
      keyDirs.add(direction);
      updateDrive();
    };
    const up = (event: KeyboardEvent) => {
      const direction = keyDirection(event.key);
      if (!direction) return;
      keyDirs.delete(direction);
      updateDrive();
    };
    const pointerUp = (event: PointerEvent) => {
      if (event.pointerId !== joystickPointer) return;
      resetJoystick(event.pointerId);
      updateDrive();
    };
    const clearAll = () => {
      keyDirs.clear();
      resetJoystick();
      stopDrive();
    };
    window.addEventListener('keydown', down);
    window.addEventListener('keyup', up);
    window.addEventListener('pointerup', pointerUp);
    window.addEventListener('pointercancel', pointerUp);
    window.addEventListener('blur', clearAll);
    return () => {
      stopTimer();
      if (id && driving) actions.drive(id, 0, 0);
      driving = false;
      keyDirs.clear();
      resetJoystick();
      window.removeEventListener('keydown', down);
      window.removeEventListener('keyup', up);
      window.removeEventListener('pointerup', pointerUp);
      window.removeEventListener('pointercancel', pointerUp);
      window.removeEventListener('blur', clearAll);
    };
  });
</script>

<section class="panel-glow flex shrink-0 flex-col rounded-[--radius-card] border border-border bg-surface">
  <header class="flex h-10 shrink-0 items-center justify-between border-b border-border px-3">
    <div>
      <!--
        Name the robot being driven. The controls act on `fleet.selected[0]`, so
        with several robots selected manual input moves whichever happens to be
        first — which presents as "this robot will not move" while a different
        one drives off. Saying which one costs nothing and removes the whole
        class of confusion.
      -->
      <div class="flex items-center gap-1.5 text-[11px] font-semibold text-fg">
        Robot control
        {#if activeId}
          <span class="font-medium" style="color:{fleet.colorOf(activeId)}">
            {robotDisplayName(activeId)}
          </span>
        {/if}
      </div>
      <div class="mt-0.5 text-[9px] text-fg-dim">
        {#if fleet.selected.length > 1}
          Drives {activeId ? robotDisplayName(activeId) : ''} only · {fleet.selected.length} selected
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
    <div class="grid grid-cols-[132px_1fr] gap-3 p-3">
      <div>
        <div class="mb-2 flex items-center justify-between">
          <span class="text-[10px] font-semibold uppercase tracking-[0.06em] text-fg-muted">Manual</span>
          <span class="text-[9px] text-fg-dim">WASD / arrows</span>
        </div>

        <div class="relative mx-auto h-[124px] w-[124px]">
          <button
            bind:this={joystickElement}
            type="button"
            aria-label="Drive joystick. Push up or down to drive and left or right to turn."
            disabled={!canDrive}
            class:joystick-active={joystickPointer !== null}
            class="joystick-base"
            onpointerdown={startJoystick}
            onpointermove={updateJoystick}
          >
            <span class="joystick-forward" aria-hidden="true">FWD</span>
            <span class="joystick-reverse" aria-hidden="true">REV</span>
            <span
              class="joystick-thumb"
              style="transform: translate({joystickX * JOYSTICK_RADIUS}px, {joystickY * JOYSTICK_RADIUS}px)"
              aria-hidden="true"
            ></span>
          </button>
        </div>

        <button
          type="button"
          aria-label="Stop robot"
          disabled={!canDrive}
          class="mt-2 flex h-9 w-full items-center justify-center gap-1.5 rounded-[4px] border
                 border-danger/40 bg-danger/5 text-[10px] font-semibold text-danger transition-colors
                 hover:bg-danger/10 disabled:opacity-40"
          onclick={hardStop}
        >
          <Octagon class="h-3.5 w-3.5" /> Stop
        </button>
      </div>

      <div class="border-l border-border pl-3">
        <div class="mb-2 flex items-center justify-between">
          <span class="flex items-center gap-1 text-[10px] font-semibold uppercase tracking-[0.06em] text-fg-muted">
            <Gauge class="h-3 w-3" /> Max speed
          </span>
          <output class="tabular text-[10px] font-semibold text-fg">{maxSpeed.toFixed(2)}</output>
        </div>
        <!--
          Max matches the backend's own clamp on `drive` (app.py), deliberately.
          The slider used to go to 0.60 while the server clamped to 0.45, so the
          top quarter of its travel did nothing and the number under the operator's
          thumb was not the speed the robot was given.
        -->
        <input
          type="range"
          min="0.1"
          max="0.45"
          step="0.05"
          value={maxSpeed}
          disabled={!canDrive}
          aria-label="Maximum manual drive speed in metres per second"
          class="speed-slider"
          oninput={changeMaxSpeed}
        />
        <div class="mt-0.5 flex justify-between text-[8px] text-fg-dim">
          <span>0.10</span><span>m/s</span><span>0.45</span>
        </div>

        <div class="mb-2 mt-3 border-t border-border pt-3 text-[10px] font-semibold uppercase tracking-[0.06em] text-fg-muted">
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
      </div>
    </div>
  {:else}
    <div class="grid flex-1 place-items-center px-4 text-center text-[10px] text-fg-dim">
      Select an online robot to enable controls
    </div>
  {/if}
</section>

<style>
  .joystick-base {
    position: relative;
    display: block;
    width: 124px;
    height: 124px;
    border: 1px solid var(--color-border);
    border-radius: 9999px;
    overflow: hidden;
    background:
      linear-gradient(var(--color-border), var(--color-border)) center / 1px 78% no-repeat,
      linear-gradient(90deg, var(--color-border), var(--color-border)) center / 78% 1px no-repeat,
      radial-gradient(circle, var(--color-surface) 0 47%, var(--color-surface-2) 100%);
    box-shadow: inset 0 2px 7px rgb(0 0 0 / 0.08);
    touch-action: none;
    transition: border-color 120ms, box-shadow 120ms;
  }
  .joystick-base:hover:not(:disabled),
  .joystick-active {
    border-color: var(--color-border-strong);
    box-shadow: inset 0 2px 7px rgb(0 0 0 / 0.1), 0 0 0 2px rgb(11 92 173 / 0.08);
  }
  .joystick-base:disabled {
    opacity: 0.4;
  }
  .joystick-forward,
  .joystick-reverse {
    position: absolute;
    left: 50%;
    transform: translateX(-50%);
    color: var(--color-fg-dim);
    font-size: 7px;
    font-weight: 700;
    letter-spacing: 0.08em;
  }
  .joystick-forward {
    top: 8px;
  }
  .joystick-reverse {
    bottom: 8px;
  }
  .joystick-thumb {
    position: absolute;
    top: 44px;
    left: 44px;
    width: 34px;
    height: 34px;
    border: 1px solid color-mix(in srgb, var(--color-accent), black 10%);
    border-radius: 9999px;
    background: var(--color-accent);
    box-shadow: 0 3px 8px rgb(0 0 0 / 0.22), inset 0 1px 0 rgb(255 255 255 / 0.28);
    pointer-events: none;
    transition: transform 45ms linear;
  }
  .joystick-active .joystick-thumb {
    transition: none;
  }
  .speed-slider {
    width: 100%;
    height: 28px;
    accent-color: var(--color-accent);
    touch-action: pan-x;
  }
  .speed-slider:disabled {
    opacity: 0.4;
  }
</style>
