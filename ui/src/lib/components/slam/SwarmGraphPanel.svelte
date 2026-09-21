<script lang="ts">
  import { Link2, Share2, TriangleAlert, X } from 'lucide-svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { mapStore } from '$lib/stores/mapstore.svelte';
  import { robotDisplayName } from '$lib/robotDisplayName';
  import { summarizePeerSlam } from './peerStatus';
  import { fetchReplicaCatalogue, type ReplicaCatalogueEntry } from '$lib/components/replicas/replicaCatalogue';

  let { open = false, onclose = () => {} }: { open?: boolean; onclose?: () => void } = $props();
  let peerComponent = $state<ReplicaCatalogueEntry | null>(null);
  const peerStatus = $derived(summarizePeerSlam(fleet.robots.filter((robot) => fleet.isEnabled(robot.robot_id))));
  const rows = $derived(Object.entries(mapStore.slamGraphs)
    .filter(([id]) => fleet.isEnabled(id))
    .sort(([a], [b]) => a.localeCompare(b)));
  const disagreement = $derived(mapStore.status?.cslam_disagreement ?? {});
  const closures = $derived(rows.reduce((sum, [, graph]) => sum + graph.inter_robot.reduce(
    (count, link) => count + (typeof link === 'string' ? 1 : link.count), 0), 0));
  const joined = $derived(Math.max(rows.filter(([, graph]) => graph.in_common_frame).length, peerComponent?.robot_ids.length ?? 0));
  const keyframes = $derived(peerStatus.reporters
    ? peerStatus.keyframes
    : rows.reduce((total, [, graph]) => total + graph.keyframes, 0));

  function closeModal() { onclose(); }
  function onKeyDown(event: KeyboardEvent) { if (open && event.key === 'Escape') closeModal(); }
  function onBackdropClick(event: MouseEvent) { if (event.target === event.currentTarget) closeModal(); }
  function linkLabel(link: { other: string; count: number } | string) {
    return typeof link === 'string' ? { other: link, count: 1 } : link;
  }

  async function loadPeerComponent() {
    try {
      const catalogue = await fetchReplicaCatalogue();
      peerComponent = catalogue.components
        .filter((entry) => entry.available && entry.status === 'ready' &&
          entry.session_id === catalogue.active_session_id && entry.robot_ids.length >= 2)
        .sort((a, b) => b.robot_ids.length - a.robot_ids.length)[0] ?? null;
    } catch {
      peerComponent = null;
    }
  }

  $effect(() => {
    if (!open) return;
    void loadPeerComponent();
    const timer = window.setInterval(() => void loadPeerComponent(), 2000);
    return () => window.clearInterval(timer);
  });
</script>

<svelte:window onkeydown={onKeyDown} />

{#if open}
  <div class="fixed inset-0 z-[100] grid place-items-center bg-fg/35 p-4 backdrop-blur-[3px]" role="presentation" onclick={onBackdropClick}>
    <div class="panel-glow flex max-h-[min(820px,calc(100vh-32px))] w-full max-w-xl flex-col overflow-hidden rounded-[--radius-dialog] border border-transparent bg-surface shadow-[0_24px_64px_-20px_rgb(16_24_40/0.45)]" role="dialog" aria-modal="true" aria-labelledby="swarm-slam-title" tabindex="-1">
      <header class="flex min-h-[72px] shrink-0 items-center justify-between border-b border-border/70 px-5">
        <div class="flex min-w-0 items-center gap-3">
          <span class="grid h-10 w-10 shrink-0 place-items-center rounded-[--radius-control] bg-accent-container text-accent-container-fg"><Share2 class="h-[18px] w-[18px]" /></span>
          <div>
            <h2 id="swarm-slam-title" class="text-sm font-semibold text-fg">Peer SLAM status</h2>
            <p class="mt-0.5 text-[11px] text-fg-dim">Swarm-SLAM graph and replica publication health</p>
          </div>
        </div>
        <button class="grid h-11 w-11 touch-target place-items-center rounded-full text-fg-muted transition-colors hover:bg-surface-2 hover:text-fg" aria-label="Close peer SLAM details" title="Close" onclick={closeModal}><X class="h-5 w-5" /></button>
      </header>

      <div class="grid shrink-0 grid-cols-3 border-b border-border/70 bg-surface-2">
        <div class="px-4 py-3"><div class="text-[10px] font-medium text-fg-dim">Merged robots</div><div class="mt-1 text-lg font-semibold tabular text-fg">{joined}/{Math.max(fleet.robots.filter((robot) => robot.online && fleet.isEnabled(robot.robot_id)).length, rows.length, joined, 1)}</div></div>
        <div class="border-x border-border px-4 py-3"><div class="flex items-center gap-1.5 text-[10px] font-medium text-fg-dim"><Link2 class="h-3.5 w-3.5" /> Closures</div><div class="mt-1 text-lg font-semibold tabular text-fg">{peerStatus.reporters ? peerStatus.closures : closures || '—'}</div></div>
        <div class="px-4 py-3"><div class="flex items-center gap-1.5 text-[10px] font-medium text-fg-dim"><Share2 class="h-3.5 w-3.5" /> Keyframes</div><div class="mt-1 text-lg font-semibold tabular text-fg">{keyframes}</div></div>
      </div>

      <div class="min-h-0 flex-1 overflow-y-auto p-4">
        {#if peerComponent}
          <p class="mb-3 rounded-[--radius-control] bg-ok/10 px-3 py-2 text-[11px] text-ok">Verified shared component: {peerComponent.robot_ids.map(robotDisplayName).join(', ')} · {peerComponent.submap_count} submaps</p>
        {/if}
        {#if rows.length}
          <div class="mb-3 text-[10px] font-semibold uppercase tracking-[0.08em] text-fg-dim">Robot graph status</div>
          <div class="mb-4 flex flex-col gap-2">
            {#each rows as [robotId, graph]}
              {@const check = disagreement[robotId]}
              <article class="rounded-[--radius-card] border border-transparent bg-surface-2 p-4">
                <div class="flex items-start justify-between gap-4">
                  <span class="flex min-w-24 items-center gap-2 text-xs font-semibold text-fg"><i class="h-2.5 w-2.5 rounded-full" style="background:{fleet.colorOf(robotId)}"></i>{robotDisplayName(robotId)}</span>
                  <div class="min-w-0 flex-1 text-right text-[11px]">
                    {#if graph.inter_robot.length}<div class="text-fg-muted">{#each graph.inter_robot as link, i}{i ? ' · ' : ''}{robotDisplayName(linkLabel(link).other)}<span class="text-fg-dim">×{linkLabel(link).count}</span>{/each}</div>{:else}<div class="text-fg-dim">No inter-robot closures yet</div>{/if}
                    <div class="mt-1 text-fg-dim">{graph.keyframes} keyframes{#if graph.residual != null} · residual {graph.residual.toFixed(3)}{/if}</div>
                    {#if check}<div class="mt-1 flex items-center justify-end gap-1 {!check.confident ? 'text-fg-dim' : check.metres > 0.5 ? 'text-warn' : 'text-ok'}">{#if check.confident && check.metres > 0.5}<TriangleAlert class="h-3.5 w-3.5" />{/if}Check {check.metres.toFixed(2)} m / {check.degrees.toFixed(1)}°{check.confident ? '' : ' (inconclusive)'}</div>{/if}
                  </div>
                </div>
              </article>
            {/each}
          </div>
        {:else}
          <p class="rounded-[--radius-control] bg-surface-2 px-3 py-2 text-[11px] text-fg-dim">Waiting for peer SLAM graph reports.</p>
        {/if}
        <p class="mt-3 text-[10px] text-fg-dim">Peer corrections are data carried by replica products; the navigation frame remains continuous odometry.</p>
      </div>
    </div>
  </div>
{/if}
