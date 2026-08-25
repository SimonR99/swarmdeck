<script lang="ts">
  /**
   * Collaborative pose-graph health.
   *
   * Under grid merging the interesting question was "did the correlation
   * accept?", and the layers popover already answers it. Under a collaborative
   * back end the question is different and this panel answers that one: which
   * robots have actually met, how often, and does the fleet agree on one frame.
   * An inter-robot loop closure is the event that makes this swarm SLAM rather
   * than map stitching, so it is the thing given the most room.
   *
   * Hidden entirely unless something reports a graph, so the 2D SLAM Toolbox
   * fleet is not shown an empty box.
   */
  import { ChevronRight, Link2, Share2, TriangleAlert, X } from 'lucide-svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { mapStore } from '$lib/stores/mapstore.svelte';
  import { robotDisplayName } from '$lib/robotDisplayName';

  const rows = $derived(
    Object.entries(mapStore.slamGraphs)
      .filter(([id]) => fleet.isEnabled(id))
      .sort(([a], [b]) => a.localeCompare(b))
  );
  const disagreement = $derived(mapStore.status?.cslam_disagreement ?? {});
  const closures = $derived(
    rows.reduce((sum, [, g]) => sum + g.inter_robot.reduce((n, l) => n + l.count, 0), 0)
  );
  const joined = $derived(rows.filter(([, g]) => g.in_common_frame).length);
  const keyframes = $derived(rows.reduce((total, [, graph]) => total + graph.keyframes, 0));
  let modalOpen = $state(false);

  function closeModal() {
    modalOpen = false;
  }

  function onKeyDown(event: KeyboardEvent) {
    if (modalOpen && event.key === 'Escape') closeModal();
  }

  function onBackdropClick(event: MouseEvent) {
    if (event.target === event.currentTarget) closeModal();
  }

  $effect(() => {
    if (rows.length === 0) modalOpen = false;
  });
</script>

<svelte:window onkeydown={onKeyDown} />

{#if rows.length}
  <button
    class="panel-glow flex min-h-16 w-full shrink-0 items-center gap-3 rounded-[--radius-panel]
           border border-transparent bg-surface px-4 text-left transition-[background,box-shadow,transform]
           hover:bg-surface-2 hover:shadow-md active:scale-[0.99]"
    aria-haspopup="dialog"
    onclick={() => (modalOpen = true)}
  >
    <span
      class="grid h-10 w-10 shrink-0 place-items-center rounded-[--radius-control]
             bg-accent-container text-accent-container-fg"
    >
      <Share2 class="h-4 w-4" />
    </span>
    <span class="min-w-0 flex-1">
      <span class="block text-xs font-semibold text-fg">Swarm SLAM</span>
      <span class="mt-0.5 block text-[10px] text-fg-dim">
        {closures} closures · {keyframes} keyframes
      </span>
    </span>
    <span
      class="rounded-full px-2 py-0.5 text-[10px] font-semibold
             {joined === rows.length ? 'bg-ok/10 text-ok' : 'bg-warn/10 text-warn'}"
    >
      {joined}/{rows.length} in frame
    </span>
    <ChevronRight class="h-4 w-4 shrink-0 text-fg-dim" />
  </button>

  {#if modalOpen}
    <div
      class="fixed inset-0 z-[100] grid place-items-center bg-fg/35 p-4 backdrop-blur-[3px]"
      role="presentation"
      onclick={onBackdropClick}
    >
      <div
        class="panel-glow flex max-h-[min(720px,calc(100vh-32px))] w-full max-w-xl
               flex-col overflow-hidden rounded-[--radius-dialog] border border-transparent bg-surface
               shadow-[0_24px_64px_-20px_rgb(16_24_40/0.45)]"
        role="dialog"
        aria-modal="true"
        aria-labelledby="swarm-slam-title"
        tabindex="-1"
      >
        <header class="flex min-h-[72px] shrink-0 items-center justify-between border-b border-border/70 px-5">
          <div class="flex min-w-0 items-center gap-3">
            <span
              class="grid h-10 w-10 shrink-0 place-items-center rounded-[--radius-control]
                     bg-accent-container text-accent-container-fg"
            >
              <Share2 class="h-[18px] w-[18px]" />
            </span>
            <div>
              <h2 id="swarm-slam-title" class="text-sm font-semibold text-fg">Swarm SLAM</h2>
              <p class="mt-0.5 text-[11px] text-fg-dim">
                Inter-robot loop closures · joint pose graph
              </p>
            </div>
          </div>
          <button
            class="grid h-11 w-11 touch-target place-items-center rounded-full
                   text-fg-muted transition-colors hover:bg-surface-2 hover:text-fg"
            aria-label="Close Swarm SLAM details"
            title="Close"
            onclick={closeModal}
          >
            <X class="h-5 w-5" />
          </button>
        </header>

        <div class="grid shrink-0 grid-cols-3 border-b border-border/70 bg-surface-2">
          <div class="px-4 py-3">
            <div class="text-[10px] font-medium text-fg-dim">Robots in frame</div>
            <div class="mt-1 text-lg font-semibold tabular text-fg">{joined}/{rows.length}</div>
          </div>
          <div class="border-x border-border px-4 py-3">
            <div class="flex items-center gap-1.5 text-[10px] font-medium text-fg-dim">
              <Link2 class="h-3.5 w-3.5" /> Loop closures
            </div>
            <div class="mt-1 text-lg font-semibold tabular text-fg">{closures}</div>
          </div>
          <div class="px-4 py-3">
            <div class="flex items-center gap-1.5 text-[10px] font-medium text-fg-dim">
              <Share2 class="h-3.5 w-3.5" /> Keyframes
            </div>
            <div class="mt-1 text-lg font-semibold tabular text-fg">{keyframes}</div>
          </div>
        </div>

        <div class="min-h-0 flex-1 overflow-y-auto p-4">
          <div class="mb-3 text-[10px] font-semibold uppercase tracking-[0.08em] text-fg-dim">
            Robot graph status
          </div>
          <div class="flex flex-col gap-2">
            {#each rows as [robotId, graph]}
              {@const check = disagreement[robotId]}
              <article class="rounded-[--radius-card] border border-transparent bg-surface-2 p-4">
                <div class="flex items-start justify-between gap-4">
                  <span class="flex min-w-24 items-center gap-2 text-xs font-semibold text-fg">
                    <i class="h-2.5 w-2.5 rounded-full" style="background:{fleet.colorOf(robotId)}"></i>
                    {robotDisplayName(robotId)}
                  </span>
                  <div class="min-w-0 flex-1 text-right text-[11px]">
                    {#if graph.inter_robot.length}
                      <div class="text-fg-muted">
                        {#each graph.inter_robot as link, i}<!--
                          -->{i ? ' · ' : ''}{robotDisplayName(link.other)}<span class="text-fg-dim"
                            >&times;{link.count}</span
                          >{/each}
                      </div>
                    {:else}
                      <div class="text-fg-dim">No inter-robot closures yet</div>
                    {/if}
                    <div class="mt-1 text-fg-dim">
                      {graph.keyframes} keyframes{#if graph.residual != null}
                        · residual {graph.residual.toFixed(3)}{/if}
                    </div>
                    {#if check}
                      <div
                        class="mt-1 flex items-center justify-end gap-1
                               {!check.confident ? 'text-fg-dim' : check.metres > 0.5 ? 'text-warn' : 'text-ok'}"
                        title="Independent grid-correlation check against the pose graph's alignment{check.confident
                          ? ''
                          : ' — inconclusive, rival alignments could not be separated'}"
                      >
                        {#if check.confident && check.metres > 0.5}
                          <TriangleAlert class="h-3.5 w-3.5" />
                        {/if}
                        Check {check.metres.toFixed(2)} m / {check.degrees.toFixed(1)}&deg;{check.confident
                          ? ''
                          : ' (inconclusive)'}
                      </div>
                    {/if}
                  </div>
                </div>
              </article>
            {/each}
          </div>
        </div>
      </div>
    </div>
  {/if}
{/if}
