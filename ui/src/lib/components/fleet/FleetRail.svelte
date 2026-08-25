<script lang="ts">
  import { PanelLeftClose, UsersRound } from 'lucide-svelte';
  import RobotCard from './RobotCard.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { settings } from '$lib/stores/settings.svelte';
  import { actions } from '$lib/api/connection';
  import { robotDisplayName } from '$lib/robotDisplayName';

  let { oncollapse = () => {} }: { oncollapse?: () => void } = $props();

  const bodyRobot = $derived.by(() => {
    if (fleet.selected.length !== 1) return undefined;
    const robot = fleet.get(fleet.selected[0]);
    return robot && fleet.can(robot.robot_id, 'body') ? robot : undefined;
  });
</script>

<aside
  class="panel-glow flex h-full w-full flex-col overflow-hidden rounded-[--radius-panel]
         border border-transparent bg-surface-2"
>
  <header class="flex h-14 shrink-0 items-center justify-between border-b border-border/70 px-4">
    <div class="flex min-w-0 items-center gap-2.5">
      <div class="grid h-9 w-9 shrink-0 place-items-center rounded-[--radius-control] bg-accent-container text-accent-container-fg">
        <UsersRound class="h-4 w-4" />
      </div>
      <div>
        <div class="text-[13px] font-semibold text-fg">Fleet</div>
        <div class="text-[10px] text-fg-dim">{fleet.online} of {fleet.count} online</div>
      </div>
    </div>
    <button
      class="grid h-10 w-10 touch-target place-items-center rounded-full
             text-fg-muted transition-colors hover:bg-surface-3 hover:text-fg"
      title="Collapse Fleet panel"
      aria-label="Collapse Fleet panel"
      onclick={oncollapse}
    >
      <PanelLeftClose class="h-4 w-4" />
    </button>
  </header>

  <div class="flex min-h-0 flex-1 flex-col gap-2 overflow-hidden p-2">
    {#each fleet.robots as robot (robot.robot_id)}
      <RobotCard {robot} unattendedThreshold={settings.value.unattended_threshold_s} />
    {/each}

    {#if fleet.count === 0}
      <div
        class="grid place-items-center rounded-[--radius-card] border border-dashed border-border
               px-4 py-10 text-center text-xs text-fg-dim"
      >
        No robots connected
      </div>
    {/if}
  </div>

  {#if bodyRobot}
    <section class="shrink-0 border-t border-border/70 bg-surface px-3 pb-3 pt-2.5" aria-label="Robot body actions">
      <div class="mb-2 flex items-center justify-between gap-2 px-1">
        <div>
          <div class="text-[11px] font-semibold text-fg">{robotDisplayName(bodyRobot.robot_id)} actions</div>
          <div class="mt-0.5 text-[9px] text-fg-dim">Body control</div>
        </div>
        <span class="h-2 w-2 rounded-full {bodyRobot.online ? 'bg-ok' : 'bg-danger'}"></span>
      </div>
      <div class="grid grid-cols-2 gap-2">
        <button
          class="inline-flex h-11 touch-target items-center justify-center rounded-full
                 bg-accent-container px-3 text-[11px] font-semibold text-accent-container-fg
                 transition-[background,transform] hover:brightness-95 active:scale-[0.98]
                 disabled:pointer-events-none disabled:opacity-40"
          disabled={!bodyRobot.online}
          onclick={() => actions.bodyCommand(bodyRobot.robot_id, 'claim')}
        >
          Claim
        </button>
        <button
          class="inline-flex h-11 touch-target items-center justify-center rounded-full
                 bg-surface-3 px-3 text-[11px] font-semibold text-fg-muted
                 transition-[background,transform] hover:bg-border active:scale-[0.98]
                 disabled:pointer-events-none disabled:opacity-40"
          disabled={!bodyRobot.online}
          onclick={() => actions.bodyCommand(bodyRobot.robot_id, 'release')}
        >
          Release
        </button>
        <button
          class="inline-flex h-11 touch-target items-center justify-center rounded-full
                 bg-surface-3 px-3 text-[11px] font-semibold text-fg-muted
                 transition-[background,transform] hover:bg-border active:scale-[0.98]
                 disabled:pointer-events-none disabled:opacity-40"
          disabled={!bodyRobot.online}
          onclick={() => actions.bodyCommand(bodyRobot.robot_id, 'sit')}
        >
          Sit
        </button>
        <button
          class="inline-flex h-11 touch-target items-center justify-center rounded-full
                 bg-surface-3 px-3 text-[11px] font-semibold text-fg-muted
                 transition-[background,transform] hover:bg-border active:scale-[0.98]
                 disabled:pointer-events-none disabled:opacity-40"
          disabled={!bodyRobot.online}
          onclick={() => actions.bodyCommand(bodyRobot.robot_id, 'stand')}
        >
          Stand
        </button>
      </div>
    </section>
  {/if}
</aside>
