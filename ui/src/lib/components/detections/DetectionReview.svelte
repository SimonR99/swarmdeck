<script lang="ts">
  import { flip } from 'svelte/animate';
  import { Check, GitMerge, EyeOff, RotateCcw, MapPin, Trash2 } from 'lucide-svelte';
  import { review } from '$lib/stores/review.svelte';
  import { detectionCatalog } from '$lib/stores/detection.svelte';
  import { actions } from '$lib/api/connection';
  import { robotDisplayName } from '$lib/robotDisplayName';
  import type { DetectionProposal } from '$lib/types/protocol';

  // Anchored bottom-left, mirroring AlertStack top-right. It sits over the map
  // rather than in the right rail because the rail collapses with the camera,
  // and a review queue that disappears when you hide the camera is a queue the
  // operator stops trusting.
  const MAX_SHOWN = 3;
  /**
   * Dead time after a decision, because the queue moves under the pointer.
   *
   * Answering the top card promotes the next one into the exact position the
   * cursor is already in, so a second click lands on a proposal the operator
   * never read. Six ducks were accepted in one session that way. This is short
   * enough not to feel laggy and long enough that a double-click cannot decide
   * two different objects.
   */
  const SETTLE_MS = 450;

  let mergeOpen = $state<string | null>(null);
  let listOpen = $state(false);
  let busyUntil = $state(0);
  let now = $state(Date.now());
  const settling = $derived(now < busyUntil);

  $effect(() => {
    if (!settling) return;
    const timer = setTimeout(() => (now = Date.now()), busyUntil - now + 20);
    return () => clearTimeout(timer);
  });

  function decided() {
    now = Date.now();
    busyUntil = now + SETTLE_MS;
  }

  const shown = $derived(
    [
      ...review.proposals.filter((p) => p.id === review.selected),
      ...review.proposals.filter((p) => p.id !== review.selected)
    ].slice(0, MAX_SHOWN)
  );
  const sortedEntities = $derived([
    ...review.entities.filter((e) => e.id === review.selected),
    ...review.entities.filter((e) => e.id !== review.selected)
  ]);
  const overflow = $derived(Math.max(0, review.proposals.length - MAX_SHOWN));

  function label(name: string): string {
    return detectionCatalog.labelOf(name);
  }

  function seenBy(robotIds: string[]): string {
    const names = robotIds.map((id) => robotDisplayName(id));
    return names.length ? names.join(', ') : 'Unknown robot';
  }

  function accept(p: DetectionProposal) {
    if (settling) return;
    mergeOpen = null;
    decided();
    actions.acceptDetection(p.id);
  }

  function ignore(p: DetectionProposal) {
    if (settling) return;
    mergeOpen = null;
    decided();
    actions.ignoreDetection(p.id);
  }

  function merge(p: DetectionProposal, entityId: string) {
    if (settling) return;
    mergeOpen = null;
    decided();
    actions.mergeDetection(p.id, entityId);
  }

  function forgetProposal(p: DetectionProposal) {
    if (settling) return;
    mergeOpen = null;
    decided();
    actions.forgetProposal(p.id);
  }

  function selectProposal(p: DetectionProposal) {
    if (review.selected === p.id) {
      review.select(null);
      review.focus(null);
    } else {
      review.select(p.id);
      const robotId = p.robot_ids[0];
      if (robotId) actions.focusRobot(robotId);
    }
  }

  function toggleEntity(entityId: string) {
    if (review.selected === entityId) {
      review.select(null);
      review.focus(null);
    } else {
      review.select(entityId);
    }
  }

  $effect(() => {
    // A confirmed map marker has no pending card. Open the retained-object
    // list when the operator selects that marker so its detecting robot(s) are
    // still visible in the notification panel.
    if (review.selected && review.entityOf(review.selected)) listOpen = true;
  });
</script>

{#if review.proposals.length || review.ignored || review.entities.length}
  <!--
    bottom-12, not bottom-3: MapView's cursor/scale readout already owns
    `bottom-3 left-3` and is ~26 px tall, so anchoring here at bottom-3 covered
    the "x, y m · cm/cell · rev N" bar completely — and at z-30 over its z-20,
    invisibly. The stack grows upward, so this only moves its baseline.
  -->
  <div class="pointer-events-none absolute bottom-12 left-3 z-30 flex flex-col items-start gap-1.5">
    {#if review.proposals.length}
      <div
        class="pointer-events-auto flex items-center justify-between gap-2 rounded-full border
               border-border bg-surface/95 px-2.5 py-0.5 text-[8px] font-semibold text-fg-muted
               shadow-[0_8px_24px_-14px_rgb(25_32_42/0.4)] backdrop-blur-xl"
      >
        <span>
          {review.proposals.length} awaiting review
          {#if overflow > 0}
            <span class="font-normal text-fg-dim">(+{overflow} in queue)</span>
          {/if}
        </span>
        <button
          class="font-medium text-danger hover:underline"
          title="Delete all proposals awaiting review"
          onclick={() => actions.clearProposals()}
        >
          Delete all
        </button>
      </div>
    {/if}

    {#each shown as p (p.id)}
      {@const suggested = review.entityOf(p.suggested_entity_id)}
      {@const candidates = review.mergeCandidates(p)}
      <div animate:flip={{ duration: 250 }} class="relative flex flex-col items-start gap-1">
        <div
          role="button"
          tabindex="0"
          aria-label="Detection proposal"
          onclick={() => selectProposal(p)}
          onkeydown={(ev) => ev.key === 'Enter' && selectProposal(p)}
          class="pointer-events-auto flex flex-col items-stretch border shadow-xl backdrop-blur-xl cursor-pointer
                 transition-all duration-200
                 {review.selected === p.id
                   ? 'border-accent bg-surface ring-4 ring-accent/35 rounded-2xl p-2.5 gap-2 shadow-[0_12px_32px_-8px_rgb(47_99_199/0.5)] z-20'
                   : review.focused === p.id
                   ? 'border-accent/80 bg-surface/98 ring-2 ring-accent/20 rounded-full p-1.5 gap-1.5'
                   : 'border-border bg-surface/95 rounded-full p-1 gap-1'}"
          onmouseenter={() => review.focus(p.id)}
          onmouseleave={() => review.focus(null)}
        >
          <div class="flex items-center gap-1.5">
            <button
              class="flex items-center gap-1.5 px-2 py-1 rounded-full hover:bg-surface-2 transition-colors shrink-0"
              title="{label(p.class)} · {Math.round(p.best_score * 100)}% · {seenBy(p.robot_ids)} · {p.observations} viewpoint{p.observations === 1 ? '' : 's'} · {p.position.x.toFixed(1)}, {p.position.y.toFixed(1)} m"
              onclick={(e) => {
                e.stopPropagation();
                selectProposal(p);
              }}
            >
              <span
                class="rounded-full transition-all shrink-0 {review.selected === p.id ? 'h-3.5 w-3.5 ring-2 ring-accent/50' : 'h-2.5 w-2.5'}"
                style="background:{detectionCatalog.colorOf(p.class)}"
              ></span>
              <span class="text-[12px] font-semibold text-fg whitespace-nowrap">
                {label(p.class)}
                <span class="ml-1 text-[10px] font-normal text-fg-dim">({Math.round(p.best_score * 100)}%)</span>
              </span>
            </button>

            <button
              class="grid touch-target shrink-0 place-items-center rounded-full border border-ok/40
                     bg-ok/10 text-ok hover:bg-ok/20 disabled:opacity-40 transition-transform
                     {review.selected === p.id ? 'h-8 w-8 text-base' : 'h-7 w-7'}"
              disabled={settling}
              title={suggested ? 'Separate' : 'Accept'}
              onclick={(e) => {
                e.stopPropagation();
                accept(p);
              }}
            >
              <Check class="{review.selected === p.id ? 'h-4 w-4' : 'h-3.5 w-3.5'}" />
            </button>

            {#if candidates.length}
              <button
                class="grid touch-target shrink-0 place-items-center rounded-full border transition-all
                       {review.selected === p.id ? 'h-8 w-8' : 'h-7 w-7'}
                       {suggested
                         ? 'border-accent bg-accent/10 text-accent hover:bg-accent/20'
                         : 'border-border text-fg-muted hover:bg-surface-2'}
                       disabled:opacity-40"
                disabled={settling}
                title="Merge"
                onclick={(e) => {
                  e.stopPropagation();
                  suggested && candidates.length === 1
                    ? merge(p, suggested.id)
                    : (mergeOpen = mergeOpen === p.id ? null : p.id);
                }}
              >
                <GitMerge class="{review.selected === p.id ? 'h-4 w-4' : 'h-3.5 w-3.5'}" />
              </button>
            {/if}

            <button
              class="grid touch-target shrink-0 place-items-center rounded-full border border-border
                     text-fg-dim hover:bg-surface-2 hover:text-fg disabled:opacity-40 transition-all
                     {review.selected === p.id ? 'h-8 w-8' : 'h-7 w-7'}"
              disabled={settling}
              title="Ignore this object here, and stop asking about it"
              onclick={(e) => {
                e.stopPropagation();
                ignore(p);
              }}
            >
              <EyeOff class="{review.selected === p.id ? 'h-4 w-4' : 'h-3.5 w-3.5'}" />
            </button>

            <button
              class="grid touch-target shrink-0 place-items-center rounded-full border border-border/80
                     text-fg-dim hover:bg-danger/10 hover:text-danger disabled:opacity-40 transition-all
                     {review.selected === p.id ? 'h-8 w-8' : 'h-7 w-7'}"
              disabled={settling}
              title="Delete this proposal"
              onclick={(e) => {
                e.stopPropagation();
                forgetProposal(p);
              }}
            >
              <Trash2 class="{review.selected === p.id ? 'h-4 w-4' : 'h-3.5 w-3.5'}" />
            </button>
          </div>

          <!-- Expanded Image View directly within selected proposal card -->
          {#if review.selected === p.id}
            <div class="mt-1 flex flex-col items-center overflow-hidden rounded-xl bg-surface-2 p-1.5 border border-border/70">
              {#if p.image}
                <img
                  src={p.image}
                  alt="{label(p.class)} detection crop"
                  class="h-44 w-full max-w-[240px] rounded-lg object-contain bg-black/20 shadow-sm"
                />
              {:else}
                <div class="flex h-28 w-full flex-col items-center justify-center p-2 text-center text-[11px] text-fg-dim">
                  <span class="font-medium text-fg">No crop image</span>
                  <span class="text-[9px] opacity-70 mt-1">Awaiting fresh sighting</span>
                </div>
              {/if}
              <div class="mt-1.5 flex w-full items-center justify-between px-1 text-[11px]">
                <span class="font-medium text-fg">{seenBy(p.robot_ids)}</span>
                <span class="text-fg-muted font-mono">{p.position.x.toFixed(2)}, {p.position.y.toFixed(2)} m</span>
              </div>
            </div>
          {/if}
        </div>

        {#if (review.focused === p.id && review.selected !== p.id)}
          <div
            class="pointer-events-none absolute left-full top-0 ml-3 z-50 flex flex-col items-center
                   overflow-hidden rounded-xl border border-border/80 bg-surface/95 p-1.5 shadow-2xl backdrop-blur-xl"
          >
            {#if p.image}
              <img
                src={p.image}
                alt="{label(p.class)} detection crop"
                class="h-36 w-36 rounded-lg object-contain bg-black/20 shadow-sm"
              />
            {:else}
              <div class="h-32 w-32 rounded-lg bg-surface-2 flex flex-col items-center justify-center p-2 text-center text-[10px] text-fg-dim">
                <span class="font-medium text-fg">No crop image</span>
                <span class="text-[8px] opacity-70 mt-1">Awaiting fresh sighting</span>
              </div>
            {/if}
            <div class="mt-1.5 flex w-full items-center justify-between px-1 text-[10px] font-semibold text-fg">
              <span>{label(p.class)}</span>
              <span class="font-normal text-fg-dim">{Math.round(p.best_score * 100)}%</span>
            </div>
          </div>
        {/if}

        {#if mergeOpen === p.id}
          <div class="pointer-events-auto w-48 rounded-[--radius-control] border border-border bg-surface/95
                      shadow-[0_8px_24px_-14px_rgb(25_32_42/0.4)] backdrop-blur-xl">
            <div class="px-2 pt-1.5 text-[8px] font-semibold uppercase tracking-[0.06em] text-fg-muted">
              Merge into
            </div>
            <div class="flex max-h-32 flex-col gap-1 overflow-y-auto p-1.5">
              {#each candidates as candidate (candidate.id)}
                {@const d = Math.hypot(
                  candidate.position.x - p.position.x,
                  candidate.position.y - p.position.y
                )}
                <button
                  class="flex items-center justify-between rounded-[--radius-control] border border-border px-1.5 py-0.5
                         text-[8px] text-fg hover:bg-surface-2"
                  onclick={() => merge(p, candidate.id)}
                >
                  <span>{label(candidate.class)} · {candidate.observations} viewpoints</span>
                  <span class="tabular text-fg-dim">{d.toFixed(2)} m</span>
                </button>
              {/each}
            </div>
          </div>
        {/if}
      </div>
    {/each}

    <div class="pointer-events-auto flex flex-wrap items-center gap-1">
      {#if review.entities.length}
        <button
          class="flex items-center gap-1 rounded-full border bg-surface/95 px-2 py-0.5 text-[8px]
                 font-semibold backdrop-blur-xl transition-colors
                 {listOpen ? 'border-accent text-accent' : 'border-border text-fg-muted hover:text-fg'}"
          onclick={() => (listOpen = !listOpen)}
        >
          <MapPin class="h-2 w-2" />
          {review.entities.length} on map
        </button>
      {/if}
      {#if review.ignored}
        <button
          class="flex items-center gap-1 rounded-full border border-border bg-surface/95 px-2 py-0.5
                 text-[8px] font-medium text-fg-dim backdrop-blur-xl hover:text-fg"
          title="Stop suppressing ignored objects, so they can be proposed again"
          onclick={() => actions.clearIgnoredDetections()}
        >
          <RotateCcw class="h-2 w-2" />
          {review.ignored} ignored
        </button>
      {/if}
    </div>

    {#if listOpen && review.entities.length}
      <div class="pointer-events-auto w-56 rounded-[--radius-control] border border-border bg-surface/95 shadow-[0_8px_24px_-14px_rgb(25_32_42/0.4)] backdrop-blur-xl">
        <div class="flex items-center justify-between border-b border-border px-2 py-1">
          <span class="text-[8px] font-semibold uppercase tracking-[0.06em] text-fg-muted">
            Confirmed objects
          </span>
          <button
            class="text-[8px] font-medium text-danger hover:underline"
            title="Delete all detections (confirmed objects and proposals awaiting review)"
            onclick={() => {
              actions.deleteAllDetections();
              listOpen = false;
            }}
          >
            Delete all
          </button>
        </div>
        <div class="flex max-h-56 flex-col overflow-y-auto">
          {#each sortedEntities as e (e.id)}
            <div
              animate:flip={{ duration: 250 }}
              role="button"
              tabindex="0"
              onclick={() => toggleEntity(e.id)}
              onkeydown={(ev) => ev.key === 'Enter' && toggleEntity(e.id)}
              onmouseenter={() => review.focus(e.id)}
              onmouseleave={() => review.focus(null)}
              class="relative cursor-pointer flex items-center gap-2 border-b border-border/60 px-2 py-1.5 last:border-b-0 transition-all
                     {review.selected === e.id ? 'bg-accent/15 ring-1 ring-accent/40 rounded-[--radius-control] my-0.5' : 'hover:bg-surface-2'}"
            >
              <span
                class="shrink-0 rounded-full transition-all {review.selected === e.id ? 'h-2.5 w-2.5 ring-2 ring-accent/30' : 'h-1.5 w-1.5'}"
                style="background:{detectionCatalog.colorOf(e.class)}"
              ></span>
              <div class="min-w-0 flex-1">
                <div class="truncate {review.selected === e.id ? 'text-[10px] font-semibold text-fg' : 'text-[9px] font-medium text-fg'}">
                  {label(e.class)}
                  {#if review.selected === e.id}
                    <span class="ml-1 text-[8px] font-medium text-accent">(Selected)</span>
                  {/if}
                </div>
                <div class="text-[8px] {review.selected === e.id ? 'text-fg-muted font-medium' : 'text-fg-dim'}">
                  {e.position.x.toFixed(1)}, {e.position.y.toFixed(1)} m ·
                  {e.observations} viewpoint{e.observations === 1 ? '' : 's'} ·
                  {seenBy(e.robot_ids)}
                </div>
              </div>

              {#if (review.focused === e.id || review.selected === e.id) && e.image}
                <div
                  class="pointer-events-none absolute left-full top-0 ml-2 z-50 flex flex-col items-center
                         overflow-hidden rounded-xl border border-border/80 bg-surface/95 p-1.5 shadow-2xl backdrop-blur-xl"
                >
                  <img
                    src={e.image}
                    alt="{label(e.class)} detection crop"
                    class="h-28 w-28 rounded-lg object-cover shadow-sm"
                  />
                  <div class="mt-1 flex w-full items-center justify-between px-1 text-[9px] font-semibold text-fg">
                    <span>{label(e.class)}</span>
                    <span class="font-normal text-fg-dim">{e.observations} views</span>
                  </div>
                </div>
              {/if}

              <button
                class="grid h-7 w-7 shrink-0 place-items-center rounded-full text-fg-dim hover:bg-danger/10 hover:text-danger"
                title="Remove from the map. Still visible to a robot, it will be proposed again — use Ignore to silence it."
                onclick={(event) => {
                  event.stopPropagation();
                  actions.forgetDetection(e.id);
                }}
              >
                <Trash2 class="h-2.5 w-2.5" />
              </button>
            </div>
          {/each}
        </div>
        <div class="border-t border-border px-2 py-1 text-[8px] text-fg-dim">
          Deleting only unplaces it. A robot still looking at the object will propose it
          again — Ignore is what stops the asking.
        </div>
      </div>
    {/if}
  </div>
{/if}
