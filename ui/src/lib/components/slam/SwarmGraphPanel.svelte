<script lang="ts">
  import { Link2, Share2, X } from 'lucide-svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { robotDisplayName } from '$lib/robotDisplayName';
  import { summarizePeerSlam } from './peerStatus';
  import { replicaCatalogue } from '$lib/stores/replicaCatalogue.svelte';

  let { open = false, onclose = () => {} }: { open?: boolean; onclose?: () => void } = $props();
  // The largest verified multi-robot component of the current mission. The
  // deployment composite places every robot by surveyed start pose; it is not
  // a verified merge and must not count as merged membership.
  const peerComponent = $derived.by(() => {
    const catalogue = replicaCatalogue.catalogue;
    if (!catalogue || replicaCatalogue.error) return null;
    return catalogue.components
      .filter((entry) => entry.available && entry.status === 'ready' &&
        entry.session_id === catalogue.active_session_id && !entry.composite &&
        entry.robot_ids.length >= 2)
      .sort((a, b) => b.robot_ids.length - a.robot_ids.length)[0] ?? null;
  });
  const peers = $derived(fleet.robots.filter((robot) => fleet.isEnabled(robot.robot_id)));
  const peerStatus = $derived(summarizePeerSlam(peers));
  const rows = $derived(peers
    .filter((robot) => robot.online && (robot.peer_slam || robot.exploration_coordination))
    .sort((a, b) => a.robot_id.localeCompare(b.robot_id)));
  // Merged means a verified multi-robot component. The deployment composite
  // places robots by surveyed start pose and never counts.
  const joined = $derived(peerComponent?.robot_ids.length ?? 0);
  const keyframes = $derived(peerStatus.keyframes);
  const coordinationFrame = $derived(rows.map((robot) => robot.exploration_coordination?.frame).find(Boolean) ?? null);
  function decisionTone(decision: string | undefined) {
    return decision === 'granted' ? 'text-ok' : decision === 'conflict' ? 'text-warn' : 'text-fg-dim';
  }

  function closeModal() { onclose(); }
  function onKeyDown(event: KeyboardEvent) { if (open && event.key === 'Escape') closeModal(); }
  function onBackdropClick(event: MouseEvent) { if (event.target === event.currentTarget) closeModal(); }

  // Poll the shared catalogue faster while the panel is open.
  $effect(() => {
    if (!open) return;
    return replicaCatalogue.subscribe(2000);
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
        <div class="px-4 py-3"><div class="text-[10px] font-medium text-fg-dim">Merged robots</div><div class="mt-1 text-lg font-semibold tabular text-fg">{joined}/{Math.max(peers.filter((robot) => robot.online).length, 1)}</div></div>
        <div class="border-x border-border px-4 py-3"><div class="flex items-center gap-1.5 text-[10px] font-medium text-fg-dim"><Link2 class="h-3.5 w-3.5" /> Closures</div><div class="mt-1 text-lg font-semibold tabular text-fg">{peerStatus.reporters ? peerStatus.closures : '—'}</div></div>
        <div class="px-4 py-3"><div class="flex items-center gap-1.5 text-[10px] font-medium text-fg-dim"><Share2 class="h-3.5 w-3.5" /> Keyframes</div><div class="mt-1 text-lg font-semibold tabular text-fg">{keyframes}</div></div>
      </div>

      <div class="min-h-0 flex-1 overflow-y-auto p-4">
        {#if peerComponent}
          <p class="mb-3 rounded-[--radius-control] bg-ok/10 px-3 py-2 text-[11px] text-ok">Verified shared component: {peerComponent.robot_ids.map(robotDisplayName).join(', ')} · {peerComponent.submap_count} submaps</p>
        {:else}
          <p class="mb-3 rounded-[--radius-control] bg-surface-2 px-3 py-2 text-[11px] text-fg-dim">No verified inter-robot closure yet: every robot holds its own map component. The fleet map places them by surveyed start pose, which is a display composition, not a merge.</p>
        {/if}
        {#if rows.length}
          <div class="mb-3 text-[10px] font-semibold uppercase tracking-[0.08em] text-fg-dim">Robot graph status</div>
          <div class="mb-4 flex flex-col gap-2">
            {#each rows as robot (robot.robot_id)}
              {@const status = robot.peer_slam}
              {@const links = Object.entries(status?.by_peer ?? {}).filter(([, count]) => count > 0)}
              <article class="rounded-[--radius-card] border border-transparent bg-surface-2 p-4">
                <div class="flex items-start justify-between gap-4">
                  <span class="flex min-w-24 items-center gap-2 text-xs font-semibold text-fg"><i class="h-2.5 w-2.5 rounded-full" style="background:{fleet.colorOf(robot.robot_id)}"></i>{robotDisplayName(robot.robot_id)}</span>
                  <div class="min-w-0 flex-1 text-right text-[11px]">
                    {#if links.length}<div class="text-fg-muted">{#each links as [other, count], i}{i ? ' · ' : ''}{robotDisplayName(other)}<span class="text-fg-dim">×{count}</span>{/each}</div>{:else}<div class="text-fg-dim">No inter-robot closures yet</div>{/if}
                    {#if status}<div class="mt-1 text-fg-dim">{status.keyframes} keyframes · {status.verified} verified · {status.rejected} rejected closures</div>{/if}
                  </div>
                </div>
              </article>
            {/each}
          </div>
          <div class="mb-1 text-[10px] font-semibold uppercase tracking-[0.08em] text-fg-dim">Exploration coordination</div>
          <p class="mb-3 text-[10px] text-fg-dim">
            {#if coordinationFrame === 'deployment'}Frontier reservations are arbitrated in the surveyed deployment frame; each robot's planner keeps a keep-out disc around the peers it hears.
            {:else if coordinationFrame === 'component'}Frontier reservations are arbitrated inside the verified shared component.
            {:else}No shared frame: reservations are not exchanged and each robot explores independently.{/if}
          </p>
          <div class="flex flex-col gap-2">
            {#each rows as robot (robot.robot_id)}
              {@const c = robot.exploration_coordination}
              <article class="rounded-[--radius-card] border border-transparent bg-surface-2 p-4 text-[11px]">
                <div class="flex items-start justify-between gap-4">
                  <span class="flex min-w-24 items-center gap-2 text-xs font-semibold text-fg"><i class="h-2.5 w-2.5 rounded-full" style="background:{fleet.colorOf(robot.robot_id)}"></i>{robotDisplayName(robot.robot_id)}</span>
                  <div class="min-w-0 flex-1 text-right">
                    {#if !c}
                      <div class="text-fg-dim">Coordination off</div>
                    {:else}
                      {#if c.local}
                        <div class={decisionTone(c.local.decision)}>Frontier ({c.local.target[0].toFixed(1)}, {c.local.target[1].toFixed(1)}) r {c.local.radius_m.toFixed(1)} m · {c.local.decision}{#if c.local.decision === 'conflict' && c.local.winner} to {robotDisplayName(c.local.winner)}{/if}</div>
                      {:else}
                        <div class="text-fg-dim">No frontier claimed · {c.last_decision}</div>
                      {/if}
                      <div class="mt-1 text-fg-dim">Hears {c.peers_heard.filter((id) => id !== robot.robot_id).map(robotDisplayName).join(', ') || 'no peers'} · {Object.keys(c.peer_leases).length} peer frontier{Object.keys(c.peer_leases).length === 1 ? '' : 's'} · {c.peer_bodies.length} peer bod{c.peer_bodies.length === 1 ? 'y' : 'ies'} known</div>
                      <div class="mt-1 text-fg-dim">Granted {c.decisions.granted} · conflicts {c.decisions.conflict} · settling {c.decisions.pending}{#if Object.keys(c.reports).length} · reports {Object.entries(c.reports).map(([id, state]) => `${robotDisplayName(id)}: ${state}`).join(', ')}{/if}</div>
                    {/if}
                  </div>
                </div>
              </article>
            {/each}
          </div>
        {:else}
          <p class="rounded-[--radius-control] bg-surface-2 px-3 py-2 text-[11px] text-fg-dim">Waiting for peer SLAM status from online robots.</p>
        {/if}
        <p class="mt-3 text-[10px] text-fg-dim">Peer corrections are data carried by replica products; the navigation frame remains continuous odometry.</p>
      </div>
    </div>
  </div>
{/if}
