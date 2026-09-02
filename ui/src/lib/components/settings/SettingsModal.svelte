<script lang="ts">
  import { Settings, X } from 'lucide-svelte';
  import Button from '$lib/components/ui/Button.svelte';
  import { settings, colorForRobot } from '$lib/stores/settings.svelte';
  import { detectionCatalog } from '$lib/stores/detection.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import type { AppSettings } from '$lib/types/protocol';

  let { open, onclose }: { open: boolean; onclose: () => void } = $props();
  let draft = $state<AppSettings>(structuredClone($state.snapshot(settings.value)));
  let error = $state('');
  let wasOpen = false;

  function catalogFloors(): Record<string, number> {
    return Object.fromEntries(
      detectionCatalog.classes.map((target) => [target.name, target.min_score ?? 0.25])
    );
  }

  function withCatalogFloors(value: AppSettings): AppSettings {
    return {
      ...value,
      detection_single_mode: value.detection_single_mode ?? false,
      detection_single_name: value.detection_single_name ?? 'Target',
      detection_single_color: value.detection_single_color ?? '#fbbf24',
      detection_cross_class_merge: value.detection_cross_class_merge ?? true,
      detection_class_floors: {
        ...catalogFloors(),
        ...(value.detection_class_floors ?? {})
      },
      detection_robot_floors: { ...(value.detection_robot_floors ?? {}) }
    };
  }

  // Which floors the sliders edit: '' is the fleet default, anything else is
  // one robot's override of it. Overrides stay sparse on purpose — a robot
  // that currently agrees with the fleet keeps following later fleet changes
  // instead of being frozen at today's value.
  let floorScope = $state('');

  function toggleClass(name: string) {
    const next = draft.detection_classes.includes(name)
      ? draft.detection_classes.filter((item) => item !== name)
      : [...draft.detection_classes, name];
    // The backend reads an empty list as "everything", so clearing the last
    // class would silently turn every class back on. Refuse instead.
    if (!next.length) return;
    draft.detection_classes = next;
  }

  function setFloor(name: string, value: number) {
    const bounded = Math.max(0.05, Math.min(0.95, value));
    if (!floorScope) {
      draft.detection_class_floors = {
        ...draft.detection_class_floors,
        [name]: bounded
      };
      return;
    }
    draft.detection_robot_floors = {
      ...draft.detection_robot_floors,
      [floorScope]: { ...(draft.detection_robot_floors[floorScope] ?? {}), [name]: bounded }
    };
  }

  /**
   * Fleet scope returns a class to its catalog floor; robot scope drops the
   * override entirely so the robot rejoins the fleet value, rather than
   * pinning it to whatever the fleet happens to read right now.
   */
  function resetFloor(name: string) {
    if (!floorScope) {
      const catalog = detectionCatalog.classes.find((target) => target.name === name);
      setFloor(name, catalog?.min_score ?? 0.25);
      return;
    }
    const { [name]: _dropped, ...rest } = draft.detection_robot_floors[floorScope] ?? {};
    const next = { ...draft.detection_robot_floors };
    if (Object.keys(rest).length) next[floorScope] = rest;
    else delete next[floorScope];
    draft.detection_robot_floors = next;
  }

  function fleetFloorOf(name: string): number {
    return draft.detection_class_floors[name] ?? 0.25;
  }

  function overrideOf(name: string): number | undefined {
    return floorScope ? draft.detection_robot_floors[floorScope]?.[name] : undefined;
  }

  function floorOf(name: string): number {
    return overrideOf(name) ?? fleetFloorOf(name);
  }

  function resizeRobots(count: number) {
    const bounded = Math.max(1, Math.min(16, Math.round(count)));
    draft.robot_count = bounded;
    while (draft.robots.length < bounded) {
      const index = draft.robots.length;
      draft.robots.push({
        id: `robot_${index}`,
        enabled: true,
        color: colorForRobot(`robot_${index}`, index)
      });
    }
  }

  async function save() {
    error = '';
    try {
      await settings.save(draft);
      onclose();
    } catch (reason) {
      error = reason instanceof Error ? reason.message : 'Could not save settings';
    }
  }

  $effect(() => {
    if (open && !wasOpen) {
      draft = withCatalogFloors(structuredClone($state.snapshot(settings.value)));
      floorScope = '';
      error = '';
    }
    wasOpen = open;
  });

  $effect(() => {
    const closeOnEscape = (event: KeyboardEvent) => {
      if (open && event.key === 'Escape') onclose();
    };
    window.addEventListener('keydown', closeOnEscape);
    return () => window.removeEventListener('keydown', closeOnEscape);
  });
</script>

{#if open}
  <div
    class="fixed inset-0 z-[100] grid place-items-center bg-fg/35 p-4 backdrop-blur-[3px]"
    role="presentation"
    onclick={(event) => event.target === event.currentTarget && onclose()}
  >
    <div
      class="panel-glow flex max-h-[88vh] w-full max-w-2xl flex-col overflow-hidden rounded-[--radius-dialog]
             border border-transparent bg-surface shadow-[0_24px_64px_-20px_rgb(25_32_42/0.48)]"
      role="dialog"
      aria-modal="true"
      aria-labelledby="settings-title"
    >
      <header class="flex h-[72px] shrink-0 items-center justify-between border-b border-border/70 px-5">
        <div class="flex items-center gap-3">
          <span class="grid h-10 w-10 place-items-center rounded-[--radius-control] bg-accent-container text-accent-container-fg">
            <Settings class="h-[18px] w-[18px]" />
          </span>
          <h2 id="settings-title" class="text-sm font-semibold">System settings</h2>
        </div>
        <button class="grid h-11 w-11 place-items-center rounded-full text-fg-muted hover:bg-surface-2" onclick={onclose}>
          <X class="h-5 w-5" />
        </button>
      </header>

      <div class="overflow-y-auto p-5">
        <div class="grid gap-3 sm:grid-cols-2">
          <label class="space-y-1.5 text-[11px] font-medium text-fg-muted">
            Usage notification delay
            <div class="flex items-center rounded-[--radius-control] border border-transparent bg-surface-2 px-2">
              <input
                type="number"
                min="10"
                max="3600"
                bind:value={draft.unattended_threshold_s}
                class="h-11 min-w-0 flex-1 bg-transparent text-xs tabular text-fg outline-none"
              />
              <span class="text-fg-dim">seconds</span>
            </div>
          </label>

          <label class="space-y-1.5 text-[11px] font-medium text-fg-muted">
            Alert clear suppress
            <div class="flex items-center rounded-[--radius-control] border border-transparent bg-surface-2 px-2">
              <input
                type="number"
                min="0"
                max="3600"
                bind:value={draft.alert_suppress_s}
                class="h-11 min-w-0 flex-1 bg-transparent text-xs tabular text-fg outline-none"
              />
              <span class="text-fg-dim">seconds</span>
            </div>
          </label>

          <label class="space-y-1.5 text-[11px] font-medium text-fg-muted">
            Expected robots
            <select
              value={draft.robot_count}
              onchange={(event) => resizeRobots(Number(event.currentTarget.value))}
              class="h-11 w-full rounded-[--radius-control] border border-transparent bg-surface-2 px-2 text-xs text-fg outline-none"
            >
              {#each Array.from({ length: 16 }, (_, i) => i + 1) as count}<option value={count}>{count}</option>{/each}
            </select>
          </label>

          <label class="space-y-1.5 text-[11px] font-medium text-fg-muted">
            Manual drive control
            <select
              bind:value={draft.drive_control_mode}
              class="h-11 w-full rounded-[--radius-control] border border-transparent bg-surface-2 px-2 text-xs text-fg outline-none"
            >
              <option value="arrows">Arrows — one button per direction</option>
              <option value="joystick">Joystick — analogue thumbstick</option>
            </select>
          </label>
        </div>
        <p class="mt-2 text-[10px] text-fg-dim">
          Arrows suit a tablet, where one button at a time is easier to hit than a thumbstick.
          WASD and the arrow keys drive in either mode.
        </p>
        <p class="mt-2 text-[10px] text-fg-dim">
          After you clear an alert, the same condition will not reappear until the suppress window ends.
        </p>

        <div class="my-4 border-t border-border"></div>
        <div class="flex items-center justify-between gap-4">
          <div>
            <div class="text-[11px] font-semibold text-fg">Object detection</div>
            <div class="mt-0.5 text-[10px] text-fg-dim">Runs on RGB frames and is independent of Gazebo.</div>
          </div>
          <button
            class="h-7 min-w-12 rounded-[--radius-control] border px-2 text-[9px] font-semibold
                   {draft.detection_enabled ? 'border-accent bg-accent/8 text-accent' : 'border-border text-fg-dim'}"
            onclick={() => (draft.detection_enabled = !draft.detection_enabled)}
          >
            {draft.detection_enabled ? 'ON' : 'OFF'}
          </button>
        </div>
        <p class="mt-2 text-[10px] text-fg-dim">
          Raise a class until false positives disappear. The % on a camera box
          is the model's confidence for that detection. Changes apply the moment
          you save, to markers already on the map as well as new ones.
        </p>

        <div class="mt-3 grid gap-3 sm:grid-cols-2">
          <label class="space-y-1.5 text-[11px] font-medium text-fg-muted">
            Same object within
            <div class="flex items-center rounded-[--radius-control] border border-transparent bg-surface-2 px-2">
              <input
                type="number" min="0.05" max="5" step="0.05"
                bind:value={draft.detection_same_radius_m}
                disabled={!draft.detection_enabled}
                class="h-11 min-w-0 flex-1 bg-transparent text-xs tabular text-fg outline-none disabled:opacity-40"
              />
              <span class="text-fg-dim">m</span>
            </div>
          </label>
          <label class="space-y-1.5 text-[11px] font-medium text-fg-muted">
            Ask about merges within
            <div class="flex items-center rounded-[--radius-control] border border-transparent bg-surface-2 px-2">
              <input
                type="number" min="0.05" max="10" step="0.05"
                bind:value={draft.detection_ask_radius_m}
                disabled={!draft.detection_enabled}
                class="h-11 min-w-0 flex-1 bg-transparent text-xs tabular text-fg outline-none disabled:opacity-40"
              />
              <span class="text-fg-dim">m</span>
            </div>
          </label>
        </div>
        <p class="mt-2 text-[10px] text-fg-dim">
          A sighting inside the first radius is the object already on the map and is added to it
          silently. Between the two it is ambiguous, and the review queue offers the merge.
          Beyond, it is proposed as a new object.
        </p>

        <div class="mt-3 rounded-[--radius-control] border border-border bg-surface-2 p-2.5">
          <div class="mb-2 flex items-center justify-between gap-2">
            <div>
              <div class="text-[10px] font-semibold text-fg">Single detection</div>
              <div class="text-[9px] text-fg-dim">
                Display all detections under the same name and color with cross-class map merging.
              </div>
            </div>
            <label class="flex cursor-pointer items-center gap-1.5">
              <input
                type="checkbox"
                checked={draft.detection_single_mode ?? false}
                onchange={(e) => (draft.detection_single_mode = e.currentTarget.checked)}
                disabled={!draft.detection_enabled}
                class="h-4 w-4 rounded border-border accent-[var(--color-accent)] disabled:opacity-40"
              />
              <span class="text-[10px] font-medium {draft.detection_single_mode ? 'text-accent' : 'text-fg-dim'}">
                {draft.detection_single_mode ? 'ON' : 'OFF'}
              </span>
            </label>
          </div>
          <div class="grid grid-cols-[32px_1fr] gap-2">
            <input
              type="color"
              value={draft.detection_single_color || '#fbbf24'}
              oninput={(e) => (draft.detection_single_color = e.currentTarget.value)}
              disabled={!draft.detection_enabled || !draft.detection_single_mode}
              class="h-10 w-8 cursor-pointer rounded-[--radius-control] border border-transparent bg-surface-2 p-0.5 disabled:opacity-40"
              title="Single detection display colour"
              aria-label="Single detection colour"
            />
            <input
              type="text"
              bind:value={draft.detection_single_name}
              placeholder="Single detection name (e.g. Target)"
              aria-label="Single detection name"
              disabled={!draft.detection_enabled || !draft.detection_single_mode}
              class="h-10 min-w-0 rounded-[--radius-control] border border-transparent bg-surface-2 px-2 text-[10px] text-fg outline-none disabled:opacity-40"
            />
          </div>
        </div>

        {#if detectionCatalog.classes.length}
          <div class="mt-3 flex flex-wrap items-center gap-1.5">
            <span class="mr-0.5 text-[9px] font-medium text-fg-muted">Applies to</span>
            <button
              class="h-6 rounded-full border px-2 text-[9px] font-semibold
                     {floorScope === ''
                       ? 'border-accent bg-accent/8 text-accent'
                       : 'border-border text-fg-dim hover:text-fg'}"
              disabled={!draft.detection_enabled}
              onclick={() => (floorScope = '')}
            >
              All robots
            </button>
            {#each draft.robots.slice(0, draft.robot_count) as robot (robot.id)}
              {@const overrides = Object.keys(draft.detection_robot_floors[robot.id] ?? {}).length}
              <button
                class="flex h-6 items-center gap-1 rounded-full border px-2 text-[9px] font-semibold
                       {floorScope === robot.id
                         ? 'border-accent bg-accent/8 text-accent'
                         : 'border-border text-fg-dim hover:text-fg'}"
                disabled={!draft.detection_enabled}
                onclick={() => (floorScope = robot.id)}
              >
                <span
                  class="h-1.5 w-1.5 rounded-full"
                  style="background:{colorForRobot(robot.id, 0, robot.color)}"
                ></span>
                {robot.id}
                {#if overrides}<span class="text-accent">{overrides}</span>{/if}
              </button>
            {/each}
          </div>
          <p class="mt-1.5 text-[10px] text-fg-dim">
            {#if floorScope}
              Sliders below override <span class="text-fg-muted">{floorScope}</span> only. Classes
              you leave alone keep following the fleet value, including later changes to it.
            {:else}
              Sliders below set the fleet value. Robots with their own override are unaffected.
            {/if}
          </p>
        {/if}

        {#if detectionCatalog.classes.length}
          <div class="mt-3 space-y-2">
            {#each detectionCatalog.classes as target (target.name)}
              {@const on = draft.detection_classes.includes(target.name)}
              {@const floor = floorOf(target.name)}
              {@const catalogFloor = target.min_score ?? 0.25}
              <div
                class="rounded-[--radius-control] border px-2.5 py-2
                       {on ? 'border-border bg-surface-2' : 'border-border/70 opacity-60'}"
              >
                <div class="flex items-center justify-between gap-2">
                  <button
                    class="flex min-h-7 items-center gap-1.5 rounded-[--radius-control] px-0.5 text-[10px] font-medium
                           disabled:opacity-40
                           {on ? 'text-fg' : 'text-fg-dim'}"
                    disabled={!draft.detection_enabled}
                    aria-pressed={on}
                    onclick={() => toggleClass(target.name)}
                  >
                    <span
                      class="h-2 w-2 rounded-full"
                      style="background:{on ? detectionCatalog.rawColorOf(target.name) : 'currentColor'}"
                    ></span>
                    {target.label}
                  </button>
                  <div class="flex items-center gap-2">
                    {#if floorScope ? overrideOf(target.name) !== undefined : Math.abs(floor - catalogFloor) > 0.005}
                      <button
                        class="text-[9px] text-fg-dim hover:text-fg"
                        disabled={!draft.detection_enabled}
                        onclick={() => resetFloor(target.name)}
                      >
                        {#if floorScope}
                          match fleet {Math.round(fleetFloorOf(target.name) * 100)}%
                        {:else}
                          reset {Math.round(catalogFloor * 100)}%
                        {/if}
                      </button>
                    {/if}
                    <span
                      class="w-8 text-right text-[10px] tabular
                             {overrideOf(target.name) !== undefined ? 'text-accent' : 'text-fg-muted'}"
                    >
                      {Math.round(floor * 100)}%
                    </span>
                  </div>
                </div>
                <input
                  type="range"
                  min="0.05"
                  max="0.95"
                  step="0.01"
                  value={floor}
                  disabled={!draft.detection_enabled || !on}
                  aria-label="{target.label} minimum score"
                  oninput={(event) => setFloor(target.name, Number(event.currentTarget.value))}
                  class="mt-1.5 w-full accent-[var(--color-accent)] disabled:opacity-40"
                />
              </div>
            {/each}
          </div>
          <p class="mt-2 text-[10px] text-fg-dim">
            Turning every class off would hide detections with the switch above still reading ON,
            so the last one stays selected.
          </p>
        {/if}

        <div class="my-4 border-t border-border"></div>
        <div class="mb-2 flex items-end justify-between">
          <div>
            <div class="text-[11px] font-semibold text-fg">Robot connections & map colors</div>
            <div class="mt-0.5 text-[10px] text-fg-dim">OFF hides that robot from the map and controls after Save. Type and address are reported by the adapter, not set here.</div>
          </div>
          <span class="text-[9px] text-fg-dim">Save to apply</span>
        </div>

        <div class="space-y-2">
          {#each draft.robots.slice(0, draft.robot_count) as robot, index (index)}
            {@const live = fleet.get(robot.id)}
            <div class="rounded-[--radius-control] border border-border bg-surface-2 p-2">
              <div class="grid grid-cols-[32px_32px_1fr] gap-2">
                <button
                  title={robot.enabled ? 'Disable robot' : 'Enable robot'}
                  class="grid h-10 w-8 place-items-center rounded-[--radius-control] border text-[9px] font-bold
                         {robot.enabled ? 'border-ok/30 bg-ok/8 text-ok' : 'border-border text-fg-dim'}"
                  onclick={() => (robot.enabled = !robot.enabled)}
                >{robot.enabled ? 'ON' : 'OFF'}</button>
                <input
                  type="color"
                  value={colorForRobot(robot.id, index, robot.color)}
                  oninput={(e) => (robot.color = e.currentTarget.value)}
                  class="h-10 w-8 cursor-pointer rounded-[--radius-control] border border-transparent bg-surface-2 p-0.5"
                  title="Robot map colour"
                  aria-label="Robot map colour"
                />
                <input bind:value={robot.id} aria-label="Robot ID" class="h-10 min-w-0 rounded-[--radius-control] border border-transparent bg-surface-2 px-2 text-[10px] text-fg outline-none" />
              </div>
              <!--
                Reported, not configured. The adapter declares what it is at
                `hello` and the socket knows where it dialled in from, so asking
                an operator to type either would only create a second source of
                truth that can disagree with the robot.
              -->
              <div class="mt-1.5 flex flex-wrap items-center gap-x-3 gap-y-0.5 pl-[72px] font-mono text-[9px] text-fg-dim">
                {#if live?.online}
                  <span class="text-ok">● online</span>
                  <span>{live.robot_type}</span>
                  {#if live.peer}<span>from {live.peer}</span>{/if}
                  {#if live.adapter}<span>{live.adapter}</span>{/if}
                  {#if live.ros}<span>{live.ros}</span>{/if}
                {:else}
                  <span>not connected — the adapter dials in and declares itself</span>
                {/if}
              </div>
            </div>
          {/each}
        </div>

        {#if error}<div class="mt-3 text-[10px] text-danger">{error}</div>{/if}
      </div>

      <footer class="flex shrink-0 items-center justify-between border-t border-border/70 px-5 py-4">
        <span class="text-[9px] text-fg-dim">Stored in sessions/settings.json</span>
        <div class="flex gap-2">
          <Button variant="ghost" size="sm" onclick={onclose}>Cancel</Button>
          <Button variant="primary" size="sm" onclick={save} disabled={settings.saving}>
            {settings.saving ? 'Saving…' : 'Save settings'}
          </Button>
        </div>
      </footer>
    </div>
  </div>
{/if}
