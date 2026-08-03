<script lang="ts">
  import RobotCard from './RobotCard.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { settings } from '$lib/stores/settings.svelte';
</script>

<aside class="flex h-full w-full flex-col gap-2 overflow-y-auto pr-0.5">
  <div class="flex h-7 items-center justify-between px-1.5">
    <span class="text-[11px] font-semibold tracking-tight text-fg-muted">Fleet</span>
    <span class="rounded-[3px] bg-surface-3 px-1.5 py-0.5 text-[9px] tabular text-fg-muted">
      {fleet.count}
    </span>
  </div>

  {#each fleet.robots as robot (robot.robot_id)}
    <RobotCard {robot} unattendedThreshold={settings.value.unattended_threshold_s} />
  {/each}

  {#if fleet.count === 0}
    <div
      class="grid place-items-center rounded-[--radius-card] border border-dashed border-border
             px-3 py-8 text-center text-[11px] text-fg-dim"
    >
      No robots connected
    </div>
  {/if}
</aside>
