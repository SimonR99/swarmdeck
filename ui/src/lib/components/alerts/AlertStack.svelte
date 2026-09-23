<script lang="ts">
  import { TriangleAlert, X, Info, OctagonAlert } from 'lucide-svelte';
  import { session } from '$lib/stores/session.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { actions } from '$lib/api/connection';
  import { robotDisplayName } from '$lib/robotDisplayName';
  import { ALERT_TOAST_TONES, legibleOn, toastBackground } from './alertToast';

  // The cards float over the map and are not blurred, so they are opaque: the
  // map's own slate with a faint tint of the level's hue, which is what they
  // looked like when a translucent tint was blurred over the map. Their text
  // colours are in alertToast.ts, where a test holds them to 4.5:1.
  const borders = {
    info: 'border-accent/35',
    warn: 'border-warn/35',
    critical: 'border-critical/40'
  } as const;
</script>

<div class="pointer-events-none absolute right-3 top-3 z-30 flex w-72 flex-col gap-2">
  {#each session.alerts.slice(0, 4) as a (a.id)}
    {@const tone = ALERT_TOAST_TONES[a.level]}
    {@const background = toastBackground(tone)}
    <div
      class="pointer-events-auto flex items-start gap-3 rounded-[--radius-card] border px-4 py-3
             shadow-[0_8px_24px_-14px_rgb(25_32_42/0.42)] {borders[a.level]}"
      style="background-color:{background};color:{tone.text}"
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
        {#if a.detail}
          <div class="mt-1 break-words text-[10px] leading-snug" style="color:{tone.secondary}">{a.detail}</div>
        {/if}
        {#if a.robot_id}
          <div
            class="mt-0.5 text-[10px]"
            style="color:{legibleOn(fleet.colorOf(a.robot_id), background, tone.secondary)}"
          >
            {robotDisplayName(a.robot_id)}
          </div>
        {/if}
      </div>

      <button
        class="grid h-9 w-9 touch-target shrink-0 place-items-center rounded-full opacity-60 hover:bg-black/5 hover:opacity-100"
        onclick={() => actions.acknowledgeAlert(a.id)}
        title="Acknowledge"
      >
        <X class="h-3.5 w-3.5" />
      </button>
    </div>
  {/each}
</div>
