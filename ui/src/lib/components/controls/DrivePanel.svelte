<script lang="ts">
  import {
    ArrowDown,
    ArrowLeft,
    ArrowRight,
    ArrowUp,
    CircleSlash,
    Crosshair,
    Gauge,
    Octagon
  } from 'lucide-svelte';
  import { untrack } from 'svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { navigation } from '$lib/stores/navigation.svelte';
  import { settings } from '$lib/stores/settings.svelte';
  import { actions } from '$lib/api/connection';
  import { robotDisplayName } from '$lib/robotDisplayName';
  import Badge from '../ui/Badge.svelte';

  const activeId = $derived(fleet.selected[0] ?? null);
  const robot = $derived(activeId ? fleet.get(activeId) : undefined);
  const canDrive = $derived(
    Boolean(
      activeId &&
      (fleet.can(activeId, 'navigate') || fleet.can(activeId, 'estop')) &&
      robot?.online
    )
  );
  // Operator setting, defaulting to the arrow pad: one button per direction is
  // what a finger on a tablet can hit, where a thumbstick needs a sustained
  // drag held at the right angle.
  const driveMode = $derived(settings.value.drive_control_mode ?? 'arrows');

  type Direction = 'up' | 'down' | 'left' | 'right';

  const DIRECTIONS: Direction[] = ['up', 'down', 'left', 'right'];
  const MAX_ANGULAR_SPEED = 0.8;
  const JOYSTICK_RADIUS = 45;
  const JOYSTICK_DEAD_ZONE = 0.06;

  let driveTimer: number | null = null;
  let driving = false;
  let driveLabel = $state('Ready');
  let maxSpeed = $state(0.5);
  let bodyHeight = $state(0.0);
  let joystickX = $state(0);
  let joystickY = $state(0);
  let joystickPointer = $state<number | null>(null);
  let joystickElement = $state<HTMLButtonElement>();

  // Which pointer is holding each arrow, so a release only clears the button
  // the finger that lifted was on -- two-finger diagonals behave like two keys.
  let padPointers = $state<Record<Direction, number | null>>({
    up: null,
    down: null,
    left: null,
    right: null
  });

  // Track active keyboard inputs reactively so keys (WASD / arrows) provide
  // the exact same visual feedback as on-screen pointer holds.
  let keyDirs = $state<Record<Direction, boolean>>({
    up: false,
    down: false,
    left: false,
    right: false
  });

  function isHeld(direction: Direction): boolean {
    return padPointers[direction] !== null || keyDirs[direction];
  }

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

  /** Keyboard and arrow pad share one set, so a key and a button add up. */
  function heldDirs(): Set<Direction> {
    const dirs = new Set<Direction>();
    for (const direction of DIRECTIONS) {
      if (isHeld(direction)) dirs.add(direction);
    }
    return dirs;
  }

  const displayedJoystick = $derived.by(() => {
    if (joystickPointer !== null) {
      return { x: joystickX, y: joystickY, active: true };
    }
    const dirs = heldDirs();
    if (dirs.size === 0) {
      return { x: 0, y: 0, active: false };
    }
    let x = 0;
    let y = 0;
    if (dirs.has('up') && !dirs.has('down')) y -= 1;
    else if (dirs.has('down') && !dirs.has('up')) y += 1;
    if (dirs.has('left') && !dirs.has('right')) x -= 1;
    else if (dirs.has('right') && !dirs.has('left')) x += 1;

    const dist = Math.hypot(x, y);
    if (dist > 0) {
      x = x / dist;
      y = y / dist;
    }
    return { x, y, active: true };
  });

  function currentVector(): [number, number] {
    if (joystickPointer !== null) return joystickVector();
    return vectorFor(heldDirs());
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

  function resetKeys() {
    for (const direction of DIRECTIONS) keyDirs[direction] = false;
  }

  function hardStop() {
    resetKeys();
    resetJoystick();
    resetPad();
    stopDrive(true);
    if (activeId) actions.cancelGoal(activeId);
  }

  function pressArrow(event: PointerEvent, direction: Direction) {
    if (!activeId || !canDrive) return;
    event.preventDefault();
    // Record the hold before capturing, as the joystick does: capture is the
    // nicety (it keeps the release on this button when a finger slides off its
    // edge), and it must not be able to cost the drive itself.
    padPointers[direction] = event.pointerId;
    (event.currentTarget as HTMLButtonElement).setPointerCapture(event.pointerId);
    updateDrive();
  }

  /** Release only what this pointer was holding; other fingers keep driving. */
  function releasePointer(pointerId: number): boolean {
    let released = false;
    for (const direction of DIRECTIONS) {
      if (padPointers[direction] === pointerId) {
        padPointers[direction] = null;
        released = true;
      }
    }
    return released;
  }

  function resetPad() {
    for (const direction of DIRECTIONS) padPointers[direction] = null;
  }

  function isTypingTarget(target: EventTarget | null): boolean {
    if (!target || !(target instanceof HTMLElement)) return false;
    return (
      target instanceof HTMLInputElement ||
      target instanceof HTMLTextAreaElement ||
      target instanceof HTMLSelectElement ||
      target.isContentEditable
    );
  }

  function keyDirection(event: KeyboardEvent): Direction | null {
    const key = event.key;
    const code = event.code;
    if (key === 'ArrowUp' || key.toLowerCase() === 'w' || code === 'KeyW' || code === 'ArrowUp') return 'up';
    if (key === 'ArrowDown' || key.toLowerCase() === 's' || code === 'KeyS' || code === 'ArrowDown') return 'down';
    if (key === 'ArrowLeft' || key.toLowerCase() === 'a' || code === 'KeyA' || code === 'ArrowLeft') return 'left';
    if (key === 'ArrowRight' || key.toLowerCase() === 'd' || code === 'KeyD' || code === 'ArrowRight') return 'right';
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

  // A control removed mid-hold never gets its release, so swapping the pad for
  // the stick -- or losing the robot -- has to stop the robot itself.
  $effect(() => {
    driveMode;
    canDrive;
    untrack(() => {
      resetJoystick();
      resetPad();
      resetKeys();
      stopDrive();
    });
  });

  $effect(() => {
    const id = activeId;
    const down = (event: KeyboardEvent) => {
      if (event.repeat || isTypingTarget(event.target)) return;
      const direction = keyDirection(event);
      if (!direction) return;
      event.preventDefault();
      keyDirs[direction] = true;
      updateDrive();
    };
    const up = (event: KeyboardEvent) => {
      if (isTypingTarget(event.target)) return;
      const direction = keyDirection(event);
      if (!direction) return;
      keyDirs[direction] = false;
      updateDrive();
    };
    // Listening on the window, not the controls: a pointer released outside the
    // page -- over a browser chrome element, or after a capture is broken -- is
    // still a release, and missing it would leave the robot driving.
    const pointerUp = (event: PointerEvent) => {
      const wasJoystick = event.pointerId === joystickPointer;
      if (wasJoystick) resetJoystick(event.pointerId);
      if (!releasePointer(event.pointerId) && !wasJoystick) return;
      updateDrive();
    };
    const clearAll = () => {
      resetKeys();
      resetJoystick();
      resetPad();
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
      resetKeys();
      resetJoystick();
      resetPad();
      window.removeEventListener('keydown', down);
      window.removeEventListener('keyup', up);
      window.removeEventListener('pointerup', pointerUp);
      window.removeEventListener('pointercancel', pointerUp);
      window.removeEventListener('blur', clearAll);
    };
  });
</script>

<section class="panel-glow flex shrink-0 flex-col rounded-[--radius-panel] border border-transparent bg-surface">
  <header class="flex h-14 shrink-0 items-center justify-between border-b border-border/70 px-4">
    <div>
      <!--
        Name the robot being driven. The controls act on `fleet.selected[0]`, so
        with several robots selected manual input moves whichever happens to be
        first — which presents as "this robot will not move" while a different
        one drives off. Saying which one costs nothing and removes the whole
        class of confusion.
      -->
      <div class="flex items-center gap-1.5 text-xs font-semibold text-fg">
        Robot control
        {#if activeId}
          <span class="font-medium" style="color:{fleet.colorOf(activeId)}">
            {robotDisplayName(activeId)}
          </span>
        {/if}
      </div>
      <div class="mt-0.5 text-[10px] text-fg-dim">
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
    <div class="grid grid-cols-[132px_1fr] gap-4 p-4">
      <div>
        <div class="mb-2 flex items-center justify-between">
          <span class="text-[10px] font-semibold uppercase tracking-[0.06em] text-fg-muted">Manual</span>
          <span class="text-[9px] text-fg-dim">WASD / arrows</span>
        </div>

        {#if driveMode === 'joystick'}
          <div class="relative mx-auto h-[124px] w-[124px]">
            <button
              bind:this={joystickElement}
              type="button"
              aria-label="Drive joystick. Push up or down to drive and left or right to turn."
              disabled={!canDrive}
              class:joystick-active={displayedJoystick.active}
              class="joystick-base"
              onpointerdown={startJoystick}
              onpointermove={updateJoystick}
            >
              <span class="joystick-forward" aria-hidden="true">FWD</span>
              <span class="joystick-reverse" aria-hidden="true">REV</span>
              <span
                class="joystick-thumb"
                style="transform: translate({displayedJoystick.x * JOYSTICK_RADIUS}px, {displayedJoystick.y * JOYSTICK_RADIUS}px)"
                aria-hidden="true"
              ></span>
            </button>
          </div>
        {:else}
          <!--
            One button per direction, sized for a finger rather than a mouse.
            Forward and reverse span the pad because they are the two an
            operator holds longest; turns split the middle row the way the keys
            they mirror sit on a keyboard.
          -->
          <div class="grid grid-cols-2 gap-1.5">
            <button
              type="button"
              aria-label="Drive forward"
              aria-pressed={isHeld('up')}
              disabled={!canDrive}
              class="arrow-key col-span-2"
              class:arrow-held={isHeld('up')}
              onpointerdown={(event) => pressArrow(event, 'up')}
            >
              <ArrowUp class="h-4 w-4" />
            </button>
            <button
              type="button"
              aria-label="Turn left"
              aria-pressed={isHeld('left')}
              disabled={!canDrive}
              class="arrow-key"
              class:arrow-held={isHeld('left')}
              onpointerdown={(event) => pressArrow(event, 'left')}
            >
              <ArrowLeft class="h-4 w-4" />
            </button>
            <button
              type="button"
              aria-label="Turn right"
              aria-pressed={isHeld('right')}
              disabled={!canDrive}
              class="arrow-key"
              class:arrow-held={isHeld('right')}
              onpointerdown={(event) => pressArrow(event, 'right')}
            >
              <ArrowRight class="h-4 w-4" />
            </button>
            <button
              type="button"
              aria-label="Drive in reverse"
              aria-pressed={isHeld('down')}
              disabled={!canDrive}
              class="arrow-key col-span-2"
              class:arrow-held={isHeld('down')}
              onpointerdown={(event) => pressArrow(event, 'down')}
            >
              <ArrowDown class="h-4 w-4" />
            </button>
          </div>
        {/if}

        <button
          type="button"
          aria-label="Stop robot"
          disabled={!canDrive}
          class="mt-2 flex h-10 w-full items-center justify-center gap-1.5 rounded-full border
                 border-transparent bg-danger/10 text-[11px] font-semibold text-danger transition-colors
                 hover:bg-danger/15 active:scale-[0.98] disabled:opacity-40"
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
          max="1.0"
          step="0.05"
          value={maxSpeed}
          disabled={!canDrive}
          aria-label="Maximum manual drive speed in metres per second"
          class="speed-slider"
          oninput={changeMaxSpeed}
        />
        <div class="mt-0.5 flex justify-between text-[8px] text-fg-dim">
          <span>0.10</span><span>m/s</span><span>1.00</span>
        </div>

        <div class="mb-2 mt-3 border-t border-border pt-3 text-[10px] font-semibold uppercase tracking-[0.06em] text-fg-muted">
          Point nav
        </div>
        <button
          disabled={!canDrive}
          class="flex h-10 w-full items-center justify-center gap-1.5 rounded-full border text-[11px]
                 font-semibold transition-colors disabled:opacity-40
                 {navigation.goalMode
            ? 'border-transparent bg-accent text-white shadow-[0_2px_6px_-4px_rgb(47_99_199/0.8)]'
            : 'border-transparent bg-accent-container text-accent-container-fg hover:brightness-95'}"
          onclick={() => navigation.toggleGoalMode()}
        >
          <Crosshair class="h-3.5 w-3.5" />
          {navigation.goalMode ? 'Armed' : 'Set goal'}
        </button>
        <button
          disabled={robot.nav_status !== 'active'}
          class="mt-2 flex h-10 w-full items-center justify-center gap-1 rounded-full border
                 border-border text-[10px] font-medium text-fg-muted hover:bg-surface-2
                 disabled:opacity-40"
          onclick={() => actions.cancelGoal(activeId)}
        >
          <CircleSlash class="h-3 w-3" /> Cancel
        </button>

        {#if robot.capabilities.includes('body')}
          <div class="mb-2 mt-3 border-t border-border pt-3">
            <div class="mb-1.5 flex items-center justify-between">
              <span class="text-[10px] font-semibold uppercase tracking-[0.06em] text-fg-muted">
                Stand Height
              </span>
              <output class="tabular text-[10px] font-semibold text-fg">
                {bodyHeight > 0 ? '+' : ''}{bodyHeight.toFixed(2)} m
              </output>
            </div>
            <input
              type="range"
              min="-0.15"
              max="0.15"
              step="0.01"
              bind:value={bodyHeight}
              disabled={!canDrive}
              aria-label="Spot stand height relative to default in metres"
              class="speed-slider"
              oninput={() => {
                if (activeId) actions.bodyCommand(activeId, 'set_height', bodyHeight);
              }}
            />
            <div class="mt-0.5 flex justify-between text-[8px] text-fg-dim">
              <span>-0.15m</span><span>0.00m</span><span>+0.15m</span>
            </div>
            <div class="mt-1.5 grid grid-cols-3 gap-1">
              <button
                type="button"
                disabled={!canDrive}
                class="flex h-7 items-center justify-center rounded-md bg-surface-3 text-[9px] font-medium text-fg-muted hover:bg-border active:scale-[0.98] disabled:opacity-40"
                onclick={() => {
                  bodyHeight = -0.15;
                  if (activeId) actions.bodyCommand(activeId, 'set_height', -0.15);
                }}
              >
                Crouch
              </button>
              <button
                type="button"
                disabled={!canDrive}
                class="flex h-7 items-center justify-center rounded-md bg-surface-3 text-[9px] font-medium text-fg-muted hover:bg-border active:scale-[0.98] disabled:opacity-40"
                onclick={() => {
                  bodyHeight = 0.0;
                  if (activeId) actions.bodyCommand(activeId, 'set_height', 0.0);
                }}
              >
                Default
              </button>
              <button
                type="button"
                disabled={!canDrive}
                class="flex h-7 items-center justify-center rounded-md bg-surface-3 text-[9px] font-medium text-fg-muted hover:bg-border active:scale-[0.98] disabled:opacity-40"
                onclick={() => {
                  bodyHeight = 0.15;
                  if (activeId) actions.bodyCommand(activeId, 'set_height', 0.15);
                }}
              >
                Tiptoes
              </button>
            </div>
          </div>
        {/if}
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
    border: 1px solid transparent;
    border-radius: 9999px;
    overflow: hidden;
    background:
      linear-gradient(var(--color-border), var(--color-border)) center / 1px 78% no-repeat,
      linear-gradient(90deg, var(--color-border), var(--color-border)) center / 78% 1px no-repeat,
      radial-gradient(circle, var(--color-surface) 0 47%, var(--color-surface-3) 100%);
    box-shadow: inset 0 2px 7px rgb(25 32 42 / 0.08), 0 1px 3px rgb(25 32 42 / 0.08);
    touch-action: none;
    transition: border-color 120ms, box-shadow 120ms;
  }
  .joystick-base:hover:not(:disabled),
  .joystick-active {
    border-color: color-mix(in srgb, var(--color-accent), transparent 65%);
    box-shadow: inset 0 2px 7px rgb(25 32 42 / 0.1), 0 0 0 4px rgb(47 99 199 / 0.1);
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
  .arrow-key {
    display: grid;
    place-items: center;
    height: 38px;
    border: 1px solid transparent;
    border-radius: var(--radius-control);
    background: var(--color-surface-3);
    color: var(--color-fg);
    box-shadow: 0 1px 3px rgb(25 32 42 / 0.06);
    /* A held arrow must not also pan, zoom, or pop a text selection: on a
       tablet those gestures cancel the pointer, which reads as the robot
       stopping for no reason mid-drive. */
    touch-action: none;
    user-select: none;
    -webkit-touch-callout: none;
    transition: background-color 90ms, border-color 90ms, color 90ms, transform 90ms;
  }
  .arrow-key:hover:not(:disabled) {
    background: var(--color-border);
  }
  .arrow-key:disabled {
    opacity: 0.4;
  }
  .arrow-held:not(:disabled) {
    border-color: var(--color-accent);
    background: var(--color-accent);
    color: white;
    box-shadow: inset 0 2px 5px rgb(0 0 0 / 0.18);
    transform: scale(0.97);
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
