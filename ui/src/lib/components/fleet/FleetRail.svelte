<script lang="ts">
  import RobotCard from './RobotCard.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
</script>

<aside class="flex h-full w-full flex-col gap-2 overflow-y-auto pr-0.5">
  <div class="flex items-center justify-between px-1 pb-0.5">
    <span class="text-[10px] font-semibold uppercase tracking-widest text-fg-dim">Fleet</span>
    <span class="text-[10px] tabular text-fg-dim">{fleet.count}</span>
  </div>

  {#each fleet.robots as robot (robot.robot_id)}
    <RobotCard {robot} />
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
