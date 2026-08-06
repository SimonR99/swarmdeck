<script lang="ts">
  import { TriangleAlert, X, Info, OctagonAlert } from 'lucide-svelte';
  import { session } from '$lib/stores/session.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { actions } from '$lib/api/connection';
  import { robotDisplayName } from '$lib/robotDisplayName';

  const tones = {
    info: 'border-accent/35 bg-accent/10 text-accent',
    warn: 'border-warn/35 bg-warn/10 text-warn',
    critical: 'border-critical/40 bg-critical/12 text-danger'
  } as const;
</script>

<div class="pointer-events-none absolute right-3 top-3 z-30 flex w-72 flex-col gap-2">
  {#each session.alerts.slice(0, 4) as a (a.id)}
    <div
      class="pointer-events-auto flex items-start gap-2 rounded-[4px] border bg-surface/95 px-3 py-2.5
             shadow-[0_8px_20px_-14px_rgb(0_0_0/0.4)] backdrop-blur-xl {tones[a.level]}"
    >
      {#if a.level === 'critical'}
        <OctagonAlert class="mt-0.5 h-4 w-4 shrink-0" />
      {:else if a.level === 'warn'}
        <TriangleAlert class="mt-0.5 h-4 w-4 shrink-0" />
      {:else}
        <Info class="mt-0.5 h-4 w-4 shrink-0" />
      {/if}

      <div class="min-w-0 flex-1">
        <div class="text-[11px] font-semibold leading-tight">{a.message}</div>
        {#if a.robot_id}
          <div class="mt-0.5 text-[10px] opacity-70" style="color:{fleet.colorOf(a.robot_id)}">
            {robotDisplayName(a.robot_id)}
          </div>
        {/if}
      </div>

      <button
        class="grid h-6 w-6 shrink-0 place-items-center rounded-[4px] opacity-60 hover:opacity-100"
        onclick={() => actions.acknowledgeAlert(a.id)}
        title="Acknowledge"
      >
        <X class="h-3.5 w-3.5" />
      </button>
    </div>
  {/each}
</div>
