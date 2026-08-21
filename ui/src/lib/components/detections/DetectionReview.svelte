<script lang="ts">
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
  const overflow = $derived(Math.max(0, review.proposals.length - MAX_SHOWN));

  function label(name: string): string {
    return detectionCatalog.classes.find((c) => c.name === name)?.label ?? name;
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

  function selectProposal(p: DetectionProposal) {
    review.select(p.id);
    const robotId = p.robot_ids[0];
    if (robotId) actions.focusRobot(robotId);
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
  <div class="pointer-events-none absolute bottom-12 left-3 z-30 flex w-[268px] flex-col gap-2">
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
               {review.selected === p.id || review.focused === p.id ? 'border-accent' : ''}"
        onmouseenter={() => review.focus(p.id)}
        onmouseleave={() => review.focus(null)}
      >
        <button
          class="flex w-full items-start gap-2 px-2.5 pt-2.5 text-left"
          title="Show the detecting robot and its local map"
          onclick={() => selectProposal(p)}
        >
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
            <!--
              Viewpoints, not frames. A parked robot produces thousands of
              frames of one measurement, so showing that number would read as
              overwhelming evidence for a single biased estimate.
            -->
            <div class="mt-0.5 text-[9px] text-fg-dim">
              {seenBy(p.robot_ids)} · {p.observations} viewpoint{p.observations === 1 ? '' : 's'}
              {#if p.sightings > p.observations}<span class="opacity-70">of {p.sightings} frames</span>{/if}
              · {p.position.x.toFixed(1)}, {p.position.y.toFixed(1)} m
            </div>
          </div>
        </button>

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
                   bg-ok/8 text-[9px] font-semibold text-ok hover:bg-ok/15
                   disabled:opacity-40"
            disabled={settling}
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
                       : 'border-border text-fg-muted hover:bg-surface-2'}
                     disabled:opacity-40"
              disabled={settling}
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
                   text-fg-dim hover:bg-surface-2 hover:text-fg disabled:opacity-40"
            disabled={settling}
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
                  <span>{label(candidate.class)} · {candidate.observations} viewpoints</span>
                  <span class="tabular text-fg-dim">{d.toFixed(2)} m</span>
                </button>
              {/each}
            </div>
          </div>
        {/if}
      </div>
    {/each}

    <div class="pointer-events-auto flex flex-wrap items-center gap-1.5">
      {#if review.entities.length}
        <button
          class="flex items-center gap-1 rounded-full border bg-surface/95 px-2.5 py-1 text-[9px]
                 font-semibold backdrop-blur-xl transition-colors
                 {listOpen ? 'border-accent text-accent' : 'border-border text-fg-muted hover:text-fg'}"
          onclick={() => (listOpen = !listOpen)}
        >
          <MapPin class="h-2.5 w-2.5" />
          {review.entities.length} on map
        </button>
      {/if}
      {#if review.ignored}
        <button
          class="flex items-center gap-1 rounded-full border border-border bg-surface/95 px-2.5 py-1
                 text-[9px] font-medium text-fg-dim backdrop-blur-xl hover:text-fg"
          title="Stop suppressing ignored objects, so they can be proposed again"
          onclick={() => actions.clearIgnoredDetections()}
        >
          <RotateCcw class="h-2.5 w-2.5" />
          {review.ignored} ignored
        </button>
      {/if}
    </div>

    {#if listOpen && review.entities.length}
      <div class="pointer-events-auto rounded-[5px] border border-border bg-surface/95 shadow-[0_8px_20px_-14px_rgb(0_0_0/0.4)] backdrop-blur-xl">
        <div class="flex items-center justify-between border-b border-border px-2.5 py-1.5">
          <span class="text-[9px] font-semibold uppercase tracking-[0.06em] text-fg-muted">
            Confirmed objects
          </span>
          <button
            class="text-[9px] font-medium text-danger hover:underline"
            onclick={() => {
              actions.forgetAllDetections();
              listOpen = false;
            }}
          >
            Delete all
          </button>
        </div>
        <div class="flex max-h-56 flex-col overflow-y-auto">
          {#each review.entities as e (e.id)}
            <div
              class="flex items-center gap-2 border-b border-border/60 px-2.5 py-1.5 last:border-b-0
                     {review.selected === e.id ? 'bg-accent/8' : ''}"
            >
              <span
                class="h-2 w-2 shrink-0 rounded-full"
                style="background:{detectionCatalog.colorOf(e.class)}"
              ></span>
              <div class="min-w-0 flex-1">
                <div class="truncate text-[10px] font-medium text-fg">{label(e.class)}</div>
                <div class="text-[9px] text-fg-dim">
                  {e.position.x.toFixed(1)}, {e.position.y.toFixed(1)} m ·
                  {e.observations} viewpoint{e.observations === 1 ? '' : 's'} ·
                  {seenBy(e.robot_ids)}
                </div>
              </div>
              <button
                class="grid h-6 w-6 shrink-0 place-items-center rounded-[3px] text-fg-dim
                       hover:bg-danger/10 hover:text-danger"
                title="Remove from the map. Still visible to a robot, it will be proposed again — use Ignore to silence it."
                onclick={() => actions.forgetDetection(e.id)}
              >
                <Trash2 class="h-3 w-3" />
              </button>
            </div>
          {/each}
        </div>
        <div class="border-t border-border px-2.5 py-1.5 text-[9px] text-fg-dim">
          Deleting only unplaces it. A robot still looking at the object will propose it
          again — Ignore is what stops the asking.
        </div>
      </div>
    {/if}
  </div>
{/if}
