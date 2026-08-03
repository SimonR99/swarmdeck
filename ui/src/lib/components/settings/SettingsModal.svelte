<script lang="ts">
  import { Settings, X } from 'lucide-svelte';
  import Button from '$lib/components/ui/Button.svelte';
  import { settings, DEFAULT_ROBOT_COLORS } from '$lib/stores/settings.svelte';
  import type { AppSettings } from '$lib/types/protocol';

  let { open, onclose }: { open: boolean; onclose: () => void } = $props();
  let draft = $state<AppSettings>(structuredClone($state.snapshot(settings.value)));
  let error = $state('');
  let wasOpen = false;

  function resizeRobots(count: number) {
    const bounded = Math.max(1, Math.min(5, Math.round(count)));
    draft.robot_count = bounded;
    while (draft.robots.length < bounded) {
      const index = draft.robots.length;
      draft.robots.push({
        id: `robot_${index}`,
        enabled: true,
        type: 'ros2',
        endpoint: 'ws://localhost:8080/adapter',
        color: DEFAULT_ROBOT_COLORS[index % DEFAULT_ROBOT_COLORS.length]
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
      draft = structuredClone($state.snapshot(settings.value));
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
    class="fixed inset-0 z-[100] grid place-items-center bg-black/20 p-4 backdrop-blur-[2px]"
    role="presentation"
    onclick={(event) => event.target === event.currentTarget && onclose()}
  >
    <div
      class="panel-glow flex max-h-[88vh] w-full max-w-2xl flex-col overflow-hidden rounded-[7px]
             border border-border bg-surface shadow-2xl"
      role="dialog"
      aria-modal="true"
      aria-labelledby="settings-title"
    >
      <header class="flex h-12 shrink-0 items-center justify-between border-b border-border px-4">
        <div class="flex items-center gap-2">
          <Settings class="h-4 w-4 text-fg-muted" />
          <h2 id="settings-title" class="text-[13px] font-semibold">System settings</h2>
        </div>
        <button class="grid h-8 w-8 place-items-center rounded-[4px] text-fg-muted hover:bg-surface-2" onclick={onclose}>
          <X class="h-4 w-4" />
        </button>
      </header>

      <div class="overflow-y-auto p-4">
        <div class="grid gap-3 sm:grid-cols-2">
          <label class="space-y-1.5 text-[10px] font-medium text-fg-muted">
            Usage notification delay
            <div class="flex items-center rounded-[4px] border border-border bg-surface px-2">
              <input
                type="number"
                min="10"
                max="3600"
                bind:value={draft.unattended_threshold_s}
                class="h-9 min-w-0 flex-1 bg-transparent text-xs tabular text-fg outline-none"
              />
              <span class="text-fg-dim">seconds</span>
            </div>
          </label>

          <label class="space-y-1.5 text-[10px] font-medium text-fg-muted">
            Alert clear suppress
            <div class="flex items-center rounded-[4px] border border-border bg-surface px-2">
              <input
                type="number"
                min="0"
                max="3600"
                bind:value={draft.alert_suppress_s}
                class="h-9 min-w-0 flex-1 bg-transparent text-xs tabular text-fg outline-none"
              />
              <span class="text-fg-dim">seconds</span>
            </div>
          </label>

          <label class="space-y-1.5 text-[10px] font-medium text-fg-muted">
            Expected robots
            <select
              value={draft.robot_count}
              onchange={(event) => resizeRobots(Number(event.currentTarget.value))}
              class="h-9 w-full rounded-[4px] border border-border bg-surface px-2 text-xs text-fg outline-none"
            >
              {#each [1, 2, 3, 4, 5] as count}<option value={count}>{count}</option>{/each}
            </select>
          </label>
        </div>
        <p class="mt-2 text-[10px] text-fg-dim">
          After you clear an alert, the same condition will not reappear until the suppress window ends.
        </p>

        <div class="my-4 border-t border-border"></div>
        <div class="flex items-center justify-between gap-4">
          <div>
            <div class="text-[11px] font-semibold text-fg">Rubber-duck detection</div>
            <div class="mt-0.5 text-[10px] text-fg-dim">Runs on RGB frames and is independent of Gazebo.</div>
          </div>
          <button
            class="h-7 min-w-12 rounded-[4px] border px-2 text-[9px] font-semibold
                   {draft.detection_enabled ? 'border-accent bg-accent/8 text-accent' : 'border-border text-fg-dim'}"
            onclick={() => (draft.detection_enabled = !draft.detection_enabled)}
          >
            {draft.detection_enabled ? 'ON' : 'OFF'}
          </button>
        </div>
        <label class="mt-3 block text-[10px] font-medium text-fg-muted">
          Sensitivity · {Math.round(draft.detection_sensitivity * 100)}%
          <input
            type="range"
            min="0.1"
            max="1"
            step="0.05"
            bind:value={draft.detection_sensitivity}
            disabled={!draft.detection_enabled}
            class="mt-2 w-full accent-[var(--color-accent)] disabled:opacity-40"
          />
        </label>

        <div class="my-4 border-t border-border"></div>
        <div class="mb-2 flex items-end justify-between">
          <div>
            <div class="text-[11px] font-semibold text-fg">Robot connections & map colors</div>
            <div class="mt-0.5 text-[10px] text-fg-dim">Configure endpoints and robot visualization colors on the map.</div>
          </div>
          <span class="text-[9px] text-fg-dim">Applied on next simulation/adapter start</span>
        </div>

        <div class="space-y-2">
          {#each draft.robots.slice(0, draft.robot_count) as robot, index (index)}
            <div class="grid grid-cols-[28px_32px_1fr_90px] gap-2 rounded-[5px] border border-border bg-bg/60 p-2 sm:grid-cols-[28px_32px_1fr_90px_2fr]">
              <button
                title="Enable robot"
                class="mt-0.5 grid h-8 w-7 place-items-center rounded-[3px] border text-[9px] font-bold
                       {robot.enabled ? 'border-ok/30 bg-ok/8 text-ok' : 'border-border text-fg-dim'}"
                onclick={() => (robot.enabled = !robot.enabled)}
              >{robot.enabled ? 'ON' : '—'}</button>
              <input
                type="color"
                value={robot.color || DEFAULT_ROBOT_COLORS[index % DEFAULT_ROBOT_COLORS.length]}
                oninput={(e) => (robot.color = e.currentTarget.value)}
                class="h-8 w-8 cursor-pointer rounded-[3px] border border-border bg-surface p-0.5"
                title="Robot Map Color"
                aria-label="Robot map color"
              />
              <input bind:value={robot.id} aria-label="Robot ID" class="h-8 min-w-0 rounded-[3px] border border-border bg-surface px-2 text-[10px] text-fg outline-none" />
              <select bind:value={robot.type} aria-label="Robot adapter type" class="h-8 rounded-[3px] border border-border bg-surface px-1.5 text-[10px] text-fg outline-none">
                <option value="ros2">ROS 2</option><option value="ros1">ROS 1</option><option value="spot">Spot</option><option value="sim">Simulation</option>
              </select>
              <input bind:value={robot.endpoint} aria-label="Adapter endpoint" class="col-span-4 h-8 min-w-0 rounded-[3px] border border-border bg-surface px-2 font-mono text-[9px] text-fg outline-none sm:col-span-1" />
            </div>
          {/each}
        </div>

        {#if error}<div class="mt-3 text-[10px] text-danger">{error}</div>{/if}
      </div>

      <footer class="flex shrink-0 items-center justify-between border-t border-border px-4 py-3">
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
