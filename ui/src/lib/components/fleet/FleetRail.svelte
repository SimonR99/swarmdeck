<script lang="ts">
  import { Compass, PanelLeftClose, Square, UsersRound } from 'lucide-svelte';
  import RobotCard from './RobotCard.svelte';
  import { actions } from '$lib/api/connection';
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

  <!--
    Exploration: the reactive bootstrap that drives the fleet before anyone
    sends a goal. Shown only when a robot advertises `explore`, which only
    simulation adapters do, exactly as the reset control is gated.

    The label follows what the robots REPORT, not what was last clicked. A
    start that never reached the adapter leaves this reading "Explore", which
    is the truth; a button that latched optimistically would claim the fleet
    was exploring while it sat still.
  -->
  {#if fleet.canExplore}
    <footer class="shrink-0 border-t border-border/70 p-2">
      {#if fleet.exploring}
        <button
          class="flex h-9 w-full touch-target items-center justify-center gap-2
                 rounded-[--radius-control] border border-border bg-surface-3
                 text-[11px] font-semibold text-fg transition-colors
                 hover:bg-surface-4"
          title="Stop the reactive exploration bootstrap"
          onclick={() => actions.stopExplore()}
        >
          <Square class="h-3.5 w-3.5" />
          Stop exploring
        </button>
      {:else}
        <button
          class="flex h-9 w-full touch-target items-center justify-center gap-2
                 rounded-[--radius-control] bg-accent-container
                 text-[11px] font-semibold text-accent-container-fg
                 transition-opacity hover:opacity-90
                 disabled:cursor-not-allowed disabled:opacity-40"
          title="Drive the fleet reactively to bootstrap the maps"
          disabled={fleet.online === 0}
          onclick={() => actions.startExplore()}
        >
          <Compass class="h-3.5 w-3.5" />
          Explore
        </button>
      {/if}
    </footer>
  {/if}
</aside>
