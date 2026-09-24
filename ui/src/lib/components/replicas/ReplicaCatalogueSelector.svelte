<script lang="ts">
  import { onMount } from 'svelte';
  import { RefreshCw } from 'lucide-svelte';
  import { replicaCatalogue } from '$lib/stores/replicaCatalogue.svelte';
  import { replicaTactical } from '$lib/stores/replicaTactical.svelte';
  import { catalogueLabel, catalogueSelection, type ReplicaCatalogueEntry } from './replicaCatalogue';

  const entries = $derived(replicaCatalogue.catalogue?.components ?? []);
  // Keep the last verified catalogue and current map selection visible
  // through a transient catalogue failure.
  const error = $derived(replicaCatalogue.error);
  const loading = $derived(replicaCatalogue.loading);

  function selectionValue(entry: ReplicaCatalogueEntry) {
    return `${entry.session_id}\u0000${entry.component_id}`;
  }

  function currentValue() {
    if (replicaTactical.preference === 'auto') return 'live';
    const selection = replicaTactical.selection;
    if (!selection) return 'live';
    if (selection.scope === 'fleet') return `${selection.sessionId}\u0000${selection.componentId}`;
    return 'robot-inspection';
  }

  function choose(value: string) {
    if (value === 'live') {
      replicaTactical.useAutomatic();
      return;
    }
    if (value === 'robot-inspection') return;
    const entry = entries.find((candidate) => selectionValue(candidate) === value);
    if (entry?.available) {
      replicaTactical.show(catalogueSelection(entry));
    }
  }

  onMount(() => replicaCatalogue.subscribe());
</script>

<div class="mt-2 rounded-[--radius-control] bg-surface-2/60 px-1.5 py-1.5">
  <div class="mb-1 flex items-center justify-between text-[9px] font-semibold uppercase tracking-[0.08em] text-fg-dim">
    <span>Map source</span>
    <button
      class="rounded p-1 text-fg-dim hover:bg-surface hover:text-fg disabled:opacity-40"
      title="Refresh verified map components"
      aria-label="Refresh verified map components"
      disabled={loading}
      onclick={() => void replicaCatalogue.refresh()}
    ><RefreshCw class="h-3 w-3 {loading ? 'animate-spin' : ''}" /></button>
  </div>
  <select
    aria-label="Map source"
    class="h-8 w-full rounded-[--radius-control] bg-surface px-1.5 text-[10px] text-fg"
    value={currentValue()}
    onchange={(event) => choose(event.currentTarget.value)}
  >
    <option value="live">Live map</option>
    {#if replicaTactical.selection?.scope === 'robot'}
      <option value="robot-inspection">Current robot inspection</option>
    {/if}
    {#each entries as entry (selectionValue(entry))}
      <option value={selectionValue(entry)} disabled={!entry.available}>
        {catalogueLabel(entry)}{entry.available ? '' : ` · ${entry.status}`}
      </option>
    {/each}
  </select>
  {#if error}
    <div class="mt-1 text-[9px] text-warn" role="status">{error}</div>
  {:else if !entries.length && !loading}
    <div class="mt-1 text-[9px] text-fg-dim">No replicated components available.</div>
  {/if}
</div>
