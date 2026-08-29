<script lang="ts">
  /**
   * Collaborative reconstruction health and the few merge knobs that are
   * safe to change on a live fleet — including real hardware.
   *
   * Opened from the bug button even before any robot has merged, so the
   * operator can tighten or loosen the solver before the first meeting.
   */
  import { Link2, Share2, TriangleAlert, X } from 'lucide-svelte';
  import Button from '$lib/components/ui/Button.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { mapStore } from '$lib/stores/mapstore.svelte';
  import { robotDisplayName } from '$lib/robotDisplayName';
  import type { SlamBackendStatus, SlamOperatorSettings } from '$lib/types/protocol';

  let {
    open = false,
    onclose = () => {}
  }: {
    open?: boolean;
    onclose?: () => void;
  } = $props();

  const emptySettings = (): SlamOperatorSettings => ({
    allow_inter_robot: true,
    min_support: 3,
    min_inter_robot_connections: 2,
    min_inter_robot_separation_m: 3,
    max_contiguous_gap_s: 60,
    min_temporal_registration_score: 0.5,
    odom_hint_weight: 0.5
  });

  let reachable = $state(false);
  let error = $state('');
  let saving = $state(false);
  let status = $state<SlamBackendStatus | null>(null);
  let draft = $state<SlamOperatorSettings>(emptySettings());
  let defaults = $state<SlamOperatorSettings>(emptySettings());

  const rows = $derived(
    Object.entries(mapStore.slamGraphs)
      .filter(([id]) => fleet.isEnabled(id))
      .sort(([a], [b]) => a.localeCompare(b))
  );
  const disagreement = $derived(mapStore.status?.cslam_disagreement ?? {});
  const closures = $derived(
    rows.reduce(
      (sum, [, graph]) =>
        sum +
        graph.inter_robot.reduce((count, link) => count + (typeof link === 'string' ? 1 : link.count), 0),
      0
    )
  );
  const joined = $derived(rows.filter(([, graph]) => graph.in_common_frame).length);
  const keyframes = $derived(
    status?.keyframes ?? rows.reduce((total, [, graph]) => total + graph.keyframes, 0)
  );
  const components = $derived(status?.components ?? []);
  const mergedComponents = $derived(components.filter((component) => component.robots.length >= 2));
  const riskyMerge = $derived(draft.min_inter_robot_connections < 2);

  function closeModal() {
    onclose();
  }

  function onKeyDown(event: KeyboardEvent) {
    if (open && event.key === 'Escape') closeModal();
  }

  function onBackdropClick(event: MouseEvent) {
    if (event.target === event.currentTarget) closeModal();
  }

  function linkLabel(link: { other: string; count: number } | string): {
    other: string;
    count: number;
  } {
    return typeof link === 'string' ? { other: link, count: 1 } : link;
  }

  async function loadBackend(includeSettings: boolean) {
    try {
      const response = await fetch('/api/slam/backend', { cache: 'no-store' });
      const body = (await response.json()) as {
        reachable?: boolean;
        error?: string;
        status?: SlamBackendStatus | null;
        settings?: SlamOperatorSettings | null;
        defaults?: SlamOperatorSettings | null;
      };
      reachable = Boolean(body.reachable);
      if (!saving) {
        error = reachable ? '' : (body.error || 'SLAM service is not reachable');
      }
      status = body.status ?? null;
      if (includeSettings && body.settings) {
        draft = { ...emptySettings(), ...body.settings };
      }
      if (body.defaults) defaults = { ...emptySettings(), ...body.defaults };
    } catch (cause) {
      reachable = false;
      error = cause instanceof Error ? cause.message : 'SLAM service is not reachable';
    }
  }

  async function applySettings() {
    if (!reachable || saving) return;
    saving = true;
    error = '';
    try {
      const response = await fetch('/api/slam/config', {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(draft)
      });
      const body = (await response.json()) as {
        error?: string;
        settings?: SlamOperatorSettings;
      };
      if (!response.ok) {
        error = body.error || `Could not apply settings (${response.status})`;
        return;
      }
      if (body.settings) draft = { ...emptySettings(), ...body.settings };
    } catch (cause) {
      error = cause instanceof Error ? cause.message : 'Could not apply settings';
    } finally {
      saving = false;
    }
  }

  function restoreDefaults() {
    draft = { ...draft, ...defaults };
  }

  $effect(() => {
    if (!open) return;
    void loadBackend(true);
    const timer = window.setInterval(() => void loadBackend(false), 2000);
    return () => window.clearInterval(timer);
  });
</script>

<svelte:window onkeydown={onKeyDown} />

{#if open}
  <div
    class="fixed inset-0 z-[100] grid place-items-center bg-fg/35 p-4 backdrop-blur-[3px]"
    role="presentation"
    onclick={onBackdropClick}
  >
    <div
      class="panel-glow flex max-h-[min(820px,calc(100vh-32px))] w-full max-w-xl
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
              {draft.registration_mode === 'graph'
                ? 'Pose-graph solver · loop closures'
                : 'Keyframe reconstruction · odometry is a heading vote only'}
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
          <div class="text-[10px] font-medium text-fg-dim">Merged groups</div>
          <div class="mt-1 text-lg font-semibold tabular text-fg">
            {mergedComponents.length || joined}/{Math.max(components.length, rows.length, 1)}
          </div>
        </div>
        <div class="border-x border-border px-4 py-3">
          <div class="flex items-center gap-1.5 text-[10px] font-medium text-fg-dim">
            <Link2 class="h-3.5 w-3.5" /> Closures
          </div>
          <div class="mt-1 text-lg font-semibold tabular text-fg">
            {status?.accepted_closures ?? closures}
          </div>
        </div>
        <div class="px-4 py-3">
          <div class="flex items-center gap-1.5 text-[10px] font-medium text-fg-dim">
            <Share2 class="h-3.5 w-3.5" /> Keyframes
          </div>
          <div class="mt-1 text-lg font-semibold tabular text-fg">{keyframes}</div>
        </div>
      </div>

      <div class="min-h-0 flex-1 overflow-y-auto p-4">
        {#if error || !reachable}
          <p class="mb-3 rounded-[--radius-control] bg-warn/10 px-3 py-2 text-[11px] text-warn">
            {error || 'SLAM service is not reachable'}
          </p>
        {/if}

        {#if status}
          <p class="mb-3 text-[11px] text-fg-dim">
            Queue {status.queued ?? 0}
            {#if (status.dropped ?? 0) > 0}
              · dropped {status.dropped}{/if}
            {#if status.inter_robot_closures}
              · {status.inter_robot_closures} cross-robot
            {/if}
            {#if status.last_error}
              · {status.last_error}{/if}
          </p>
        {/if}

        {#if rows.length}
          <div class="mb-3 text-[10px] font-semibold uppercase tracking-[0.08em] text-fg-dim">
            Robot graph status
          </div>
          <div class="mb-4 flex flex-col gap-2">
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
                          -->{i ? ' · ' : ''}{robotDisplayName(linkLabel(link).other)}<span class="text-fg-dim"
                            >&times;{linkLabel(link).count}</span
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
        {/if}

        <div class="flex items-center justify-between gap-4">
          <div>
            <div class="text-[11px] font-semibold text-fg">Reconstruction</div>
            <div class="mt-0.5 text-[10px] text-fg-dim">
              Takes effect on the next solve. Safe for hardware: odometry never pins poses.
            </div>
          </div>
          <button
            class="h-7 min-w-12 rounded-[--radius-control] border px-2 text-[9px] font-semibold
                   {draft.allow_inter_robot
              ? 'border-accent bg-accent/8 text-accent'
              : 'border-border text-fg-dim'}"
            disabled={!reachable}
            onclick={() => (draft.allow_inter_robot = !draft.allow_inter_robot)}
          >
            {draft.allow_inter_robot ? 'MERGE ON' : 'NO MERGE'}
          </button>
        </div>

        <div class="mt-3 grid gap-3 sm:grid-cols-2">
          <label class="space-y-1.5 text-[11px] font-medium text-fg-muted">
            Matches to join fragments
            <input
              type="number"
              min="2"
              max="8"
              step="1"
              bind:value={draft.min_support}
              disabled={!reachable}
              class="h-11 w-full rounded-[--radius-control] border border-transparent bg-surface-2 px-2 text-xs tabular text-fg outline-none disabled:opacity-40"
            />
          </label>
          <label class="space-y-1.5 text-[11px] font-medium text-fg-muted">
            Independent robot meetings
            <input
              type="number"
              min="1"
              max="4"
              step="1"
              bind:value={draft.min_inter_robot_connections}
              disabled={!reachable}
              class="h-11 w-full rounded-[--radius-control] border border-transparent bg-surface-2 px-2 text-xs tabular text-fg outline-none disabled:opacity-40"
            />
          </label>
          <label class="space-y-1.5 text-[11px] font-medium text-fg-muted">
            Meeting separation
            <div class="flex items-center rounded-[--radius-control] border border-transparent bg-surface-2 px-2">
              <input
                type="number"
                min="1"
                max="12"
                step="0.5"
                bind:value={draft.min_inter_robot_separation_m}
                disabled={!reachable}
                class="h-11 min-w-0 flex-1 bg-transparent text-xs tabular text-fg outline-none disabled:opacity-40"
              />
              <span class="text-fg-dim">m</span>
            </div>
          </label>
          <label class="space-y-1.5 text-[11px] font-medium text-fg-muted">
            Split after a gap of
            <div class="flex items-center rounded-[--radius-control] border border-transparent bg-surface-2 px-2">
              <input
                type="number"
                min="5"
                max="180"
                step="5"
                bind:value={draft.max_contiguous_gap_s}
                disabled={!reachable}
                class="h-11 min-w-0 flex-1 bg-transparent text-xs tabular text-fg outline-none disabled:opacity-40"
              />
              <span class="text-fg-dim">s</span>
            </div>
          </label>
          <label class="space-y-1.5 text-[11px] font-medium text-fg-muted">
            Tracking score floor
            <input
              type="number"
              min="0.3"
              max="0.8"
              step="0.05"
              bind:value={draft.min_temporal_registration_score}
              disabled={!reachable}
              class="h-11 w-full rounded-[--radius-control] border border-transparent bg-surface-2 px-2 text-xs tabular text-fg outline-none disabled:opacity-40"
            />
          </label>
          <label class="space-y-1.5 text-[11px] font-medium text-fg-muted">
            Odometry heading vote
            <input
              type="number"
              min="0"
              max="2"
              step="0.1"
              bind:value={draft.odom_hint_weight}
              disabled={!reachable}
              class="h-11 w-full rounded-[--radius-control] border border-transparent bg-surface-2 px-2 text-xs tabular text-fg outline-none disabled:opacity-40"
            />
          </label>
        </div>
        <p class="mt-2 text-[10px] text-fg-dim">
          Lower support and one meeting merge more aggressively. On hardware that is
          also how corridors get a 180° ghost. The defaults are the Botman/Tars gates.
        </p>
        {#if riskyMerge}
          <p class="mt-2 flex items-start gap-1.5 text-[11px] text-warn">
            <TriangleAlert class="mt-0.5 h-3.5 w-3.5 shrink-0" />
            One meeting can join two robots from a single symmetric corridor. Prefer 2 on the Bunker.
          </p>
        {/if}

        <div class="mt-4 flex justify-end gap-2">
          <Button variant="ghost" size="sm" disabled={!reachable} onclick={restoreDefaults}>
            Defaults
          </Button>
          <Button variant="primary" size="sm" disabled={!reachable || saving} onclick={() => void applySettings()}>
            {saving ? 'Applying…' : 'Apply'}
          </Button>
        </div>
      </div>
    </div>
  </div>
{/if}
