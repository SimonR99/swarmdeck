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
  import { Link2, Share2, TriangleAlert } from 'lucide-svelte';
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
</script>

{#if rows.length}
  <section class="panel-glow flex shrink-0 flex-col rounded-[--radius-panel] border border-border bg-surface">
    <header class="flex h-12 shrink-0 items-center justify-between border-b border-border px-3">
      <div>
        <div class="text-xs font-semibold text-fg">Swarm SLAM</div>
        <div class="mt-0.5 text-[10px] text-fg-dim">
          Inter-robot loop closures · joint pose graph
        </div>
      </div>
      <span
        class="rounded-full px-2 py-0.5 text-[10px] font-semibold
               {joined === rows.length ? 'bg-ok/10 text-ok' : 'bg-warn/10 text-warn'}"
        title="Robots the collaborative back end has placed in the common frame"
      >
        {joined}/{rows.length} in frame
      </span>
    </header>

    <div class="flex items-center gap-3 border-b border-border px-3 py-2 text-[10px] text-fg-dim">
      <span class="flex items-center gap-1">
        <Link2 class="h-3 w-3" />
        <span class="font-semibold text-fg-muted">{closures}</span> closures
      </span>
      <span class="flex items-center gap-1">
        <Share2 class="h-3 w-3" />
        <span class="font-semibold text-fg-muted">
          {rows.reduce((n, [, g]) => n + g.keyframes, 0)}
        </span> keyframes
      </span>
    </div>

    <div class="flex flex-col gap-2 p-3">
      {#each rows as [robotId, graph]}
        {@const check = disagreement[robotId]}
        <div class="flex items-start justify-between gap-2 text-[10px]">
          <span class="flex items-center gap-1.5 font-medium text-fg-muted">
            <i class="h-2 w-2 rounded-full" style="background:{fleet.colorOf(robotId)}"></i>
            {robotDisplayName(robotId)}
          </span>
          <div class="min-w-0 flex-1 text-right">
            {#if graph.inter_robot.length}
              <div class="text-fg-muted">
                {#each graph.inter_robot as link, i}<!--
                  -->{i ? ' · ' : ''}{robotDisplayName(link.other)}<span class="text-fg-dim"
                    >&times;{link.count}</span
                  >{/each}
              </div>
            {:else}
              <div class="text-fg-dim">no inter-robot closures yet</div>
            {/if}
            <div class="mt-0.5 text-fg-dim">
              {graph.keyframes} kf{#if graph.residual != null}
                · residual {graph.residual.toFixed(3)}{/if}
            </div>
            {#if check}
              <!--
                Grid correlation is no longer the estimator here; it is a second
                opinion drawn from evidence the loop closures never used, so a
                large number is a reason to look rather than a correction to
                apply. Ambiguous checks say so instead of being hidden.
              -->
              <div
                class="mt-0.5 flex items-center justify-end gap-1
                       {!check.confident ? 'text-fg-dim' : check.metres > 0.5 ? 'text-warn' : 'text-ok'}"
                title="Independent grid-correlation check against the pose graph's alignment{check.confident
                  ? ''
                  : ' — inconclusive, rival alignments could not be separated'}"
              >
                {#if check.confident && check.metres > 0.5}
                  <TriangleAlert class="h-3 w-3" />
                {/if}
                check {check.metres.toFixed(2)} m / {check.degrees.toFixed(1)}&deg;{check.confident
                  ? ''
                  : ' (inconclusive)'}
              </div>
            {/if}
          </div>
        </div>
      {/each}
    </div>
  </section>
{/if}
