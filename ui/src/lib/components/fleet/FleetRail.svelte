<script lang="ts">
  import { PanelLeftClose, UsersRound } from 'lucide-svelte';
  import RobotCard from './RobotCard.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { settings } from '$lib/stores/settings.svelte';

  let { oncollapse = () => {} }: { oncollapse?: () => void } = $props();
</script>

<aside
  class="panel-glow flex h-full w-full flex-col overflow-hidden rounded-[--radius-panel]
         border border-transparent bg-surface-2"
>
  <header class="flex h-11 shrink-0 items-center justify-between border-b border-border/70 px-3">
    <div class="flex min-w-0 items-center gap-2">
      <div class="grid h-7 w-7 shrink-0 place-items-center rounded-[--radius-control] bg-accent-container text-accent-container-fg">
        <UsersRound class="h-3.5 w-3.5" />
      </div>
      <div>
        <div class="text-[12px] font-semibold text-fg leading-none">Fleet</div>
        <div class="mt-0.5 text-[9px] text-fg-dim leading-none">{fleet.online} of {fleet.count} online</div>
      </div>
    </div>
    <button
      class="grid h-8 w-8 touch-target place-items-center rounded-full
             text-fg-muted transition-colors hover:bg-surface-3 hover:text-fg"
      title="Collapse Fleet panel"
      aria-label="Collapse Fleet panel"
      onclick={oncollapse}
    >
      <PanelLeftClose class="h-3.5 w-3.5" />
    </button>
  </header>

  <div class="flex min-h-0 flex-1 flex-col gap-1.5 overflow-y-auto p-2">
    {#each fleet.robots as robot (robot.robot_id)}
      <RobotCard {robot} unattendedThreshold={settings.value.unattended_threshold_s} />
    {/each}

    {#if fleet.count === 0}
      <div
        class="grid place-items-center rounded-[--radius-card] border border-dashed border-border
               px-4 py-8 text-center text-xs text-fg-dim"
      >
        No robots connected
      </div>
    {/if}
  </div>

</aside>
