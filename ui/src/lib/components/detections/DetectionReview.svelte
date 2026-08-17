<script lang="ts">
  import { Check, GitMerge, X, EyeOff, RotateCcw } from 'lucide-svelte';
  import { review } from '$lib/stores/review.svelte';
  import { detectionCatalog } from '$lib/stores/detection.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { actions } from '$lib/api/connection';
  import { robotDisplayName } from '$lib/robotDisplayName';
  import type { DetectionProposal } from '$lib/types/protocol';

  // Anchored bottom-left, mirroring AlertStack top-right. It sits over the map
  // rather than in the right rail because the rail collapses with the camera,
  // and a review queue that disappears when you hide the camera is a queue the
  // operator stops trusting.
  const MAX_SHOWN = 3;

  let mergeOpen = $state<string | null>(null);

  const shown = $derived(review.proposals.slice(0, MAX_SHOWN));
  const overflow = $derived(Math.max(0, review.proposals.length - MAX_SHOWN));

  function label(name: string): string {
    return detectionCatalog.classes.find((c) => c.name === name)?.label ?? name;
  }

  function seenBy(p: DetectionProposal): string {
    const names = p.robot_ids.map((id) => robotDisplayName(id));
    if (names.length === 1) return names[0];
    return `${names.length} robots`;
  }

  function accept(p: DetectionProposal) {
    mergeOpen = null;
    actions.acceptDetection(p.id);
  }

  function ignore(p: DetectionProposal) {
    mergeOpen = null;
    actions.ignoreDetection(p.id);
  }

  function merge(p: DetectionProposal, entityId: string) {
    mergeOpen = null;
    actions.mergeDetection(p.id, entityId);
  }
</script>

{#if review.proposals.length || review.ignored}
  <div class="pointer-events-none absolute bottom-3 left-3 z-30 flex w-[268px] flex-col gap-2">
    {#if overflow}
      <div class="pointer-events-auto self-start rounded-full border border-border bg-surface/95
                  px-2.5 py-1 text-[9px] font-semibold text-fg-muted backdrop-blur-xl">
        +{overflow} more awaiting review
      </div>
    {/if}

    {#each shown as p (p.id)}
      {@const suggested = review.entityOf(p.suggested_entity_id)}
      {@const candidates = review.mergeCandidates(p)}
      <!-- svelte-ignore a11y_no_static_element_interactions -->
      <div
        class="pointer-events-auto rounded-[5px] border border-border bg-surface/95 shadow-[0_8px_20px_-14px_rgb(0_0_0/0.4)]
               backdrop-blur-xl transition-colors
               {review.focused === p.id ? 'border-accent' : ''}"
        onmouseenter={() => review.focus(p.id)}
        onmouseleave={() => review.focus(null)}
      >
        <div class="flex items-start gap-2 px-2.5 pt-2.5">
          <span
            class="mt-[3px] h-2.5 w-2.5 shrink-0 rounded-full"
            style="background:{detectionCatalog.colorOf(p.class)}"
          ></span>
          <div class="min-w-0 flex-1">
            <div class="flex items-baseline gap-1.5 text-[11px] font-semibold text-fg">
              {label(p.class)}
              <span class="text-[9px] font-medium text-fg-dim">
                {Math.round(p.best_score * 100)}%
              </span>
            </div>
            <div class="mt-0.5 text-[9px] text-fg-dim">
              {seenBy(p)} · {p.observations} sighting{p.observations === 1 ? '' : 's'} ·
              {p.position.x.toFixed(1)}, {p.position.y.toFixed(1)} m
            </div>
          </div>
        </div>

        {#if suggested && p.suggested_distance !== null}
          <!--
            The ambiguous case, which is the only one worth an operator's
            attention. Naming the distance is what makes the question
            answerable: "same duck seen from a worse angle" and "a second duck"
            look identical without it.
          -->
          <div class="mx-2.5 mt-2 rounded-[4px] border border-warn/30 bg-warn/8 px-2 py-1.5 text-[9px] text-fg-muted">
            {p.suggested_distance.toFixed(2)} m from a {label(p.class)} already on the map —
            same object?
          </div>
        {/if}

        <div class="flex gap-1 p-2.5 pt-2">
          <button
            class="flex h-7 flex-1 items-center justify-center gap-1 rounded-[4px] border border-ok/40
                   bg-ok/8 text-[9px] font-semibold text-ok hover:bg-ok/15"
            onclick={() => accept(p)}
          >
            <Check class="h-3 w-3" /> {suggested ? 'Separate' : 'Accept'}
          </button>

          {#if candidates.length}
            <button
              class="flex h-7 flex-1 items-center justify-center gap-1 rounded-[4px] border text-[9px]
                     font-semibold transition-colors
                     {suggested
                       ? 'border-accent bg-accent/10 text-accent hover:bg-accent/20'
                       : 'border-border text-fg-muted hover:bg-surface-2'}"
              onclick={() =>
                suggested && candidates.length === 1
                  ? merge(p, suggested.id)
                  : (mergeOpen = mergeOpen === p.id ? null : p.id)}
            >
              <GitMerge class="h-3 w-3" /> Merge
            </button>
          {/if}

          <button
            class="flex h-7 w-8 items-center justify-center rounded-[4px] border border-border
                   text-fg-dim hover:bg-surface-2 hover:text-fg"
            title="Ignore this object here, and stop asking about it"
            onclick={() => ignore(p)}
          >
            <EyeOff class="h-3 w-3" />
          </button>
        </div>

        {#if mergeOpen === p.id}
          <div class="border-t border-border px-2.5 py-2">
            <div class="mb-1 text-[9px] font-semibold uppercase tracking-[0.06em] text-fg-muted">
              Merge into
            </div>
            <div class="flex max-h-32 flex-col gap-1 overflow-y-auto">
              {#each candidates as candidate (candidate.id)}
                {@const d = Math.hypot(
                  candidate.position.x - p.position.x,
                  candidate.position.y - p.position.y
                )}
                <button
                  class="flex items-center justify-between rounded-[3px] border border-border px-2 py-1
                         text-[9px] text-fg hover:bg-surface-2"
                  onclick={() => merge(p, candidate.id)}
                >
                  <span>{label(candidate.class)} · {candidate.observations} obs</span>
                  <span class="tabular text-fg-dim">{d.toFixed(2)} m</span>
                </button>
              {/each}
            </div>
          </div>
        {/if}
      </div>
    {/each}

    {#if review.ignored}
      <button
        class="pointer-events-auto flex items-center gap-1 self-start rounded-full border border-border
               bg-surface/95 px-2.5 py-1 text-[9px] font-medium text-fg-dim backdrop-blur-xl
               hover:text-fg"
        title="Stop suppressing ignored objects, so they can be proposed again"
        onclick={() => actions.clearIgnoredDetections()}
      >
        <RotateCcw class="h-2.5 w-2.5" />
        {review.ignored} ignored
      </button>
    {/if}
  </div>
{/if}
