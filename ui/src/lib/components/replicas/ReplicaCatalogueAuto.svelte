<script lang="ts">
  import { onMount } from 'svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { mapStore } from '$lib/stores/mapstore.svelte';
  import { replicaTactical } from '$lib/stores/replicaTactical.svelte';
  import {
    automaticCatalogueEntry,
    automaticSelectionIsCoherent,
    activeMergedRobotIds,
    catalogueSelection,
    fetchReplicaCatalogue,
    type ReplicaCatalogue
  } from './replicaCatalogue';

  let { enabled = true } = $props<{ enabled?: boolean }>();
  let catalogue = $state<ReplicaCatalogue | null>(null);
  let refreshController: AbortController | null = null;

  function preferredRobotId() {
    if (mapStore.viewMode === 'local' && mapStore.viewRobot) return mapStore.viewRobot;
    return fleet.selected[0] ?? fleet.robots[0]?.robot_id ?? null;
  }

  function selectionKey(selection: NonNullable<typeof replicaTactical.selection>) {
    return `${selection.scope ?? 'robot'}\u0000${selection.robotId}\u0000${selection.sessionId}\u0000${selection.componentId}`;
  }

  function componentKey(selection: NonNullable<typeof replicaTactical.selection>) {
    return `${selection.sessionId}\u0000${selection.componentId}`;
  }

  function apply() {
    const current = replicaTactical.selection;
    if (catalogue) {
      replicaTactical.setMergedRobotIds(activeMergedRobotIds(
        catalogue,
        preferredRobotId(),
        replicaTactical.preference === 'component' ? current : null
      ));
    }
    if (replicaTactical.preference !== 'auto') {
      replicaTactical.setAutoStatus('idle');
      return;
    }
    if (!enabled || !catalogue) return;
    const local = mapStore.viewMode === 'local' && Boolean(mapStore.viewRobot);
    const preferred = preferredRobotId();
    const entry = automaticCatalogueEntry(catalogue, preferred, !local, local ? 1 : 2);
    if (!entry) {
      const currentEntry = current && catalogue.components.find(
        (candidate) => componentKey(current) === `${candidate.session_id}\u0000${candidate.component_id}`
      );
      const currentStillCoherent = automaticSelectionIsCoherent(
        current,
        currentEntry,
        catalogue.active_session_id,
        local,
        preferred
      );
      if (current && !currentStillCoherent) {
        replicaTactical.clear('auto');
      }
      replicaTactical.setAutoStatus(
        !local && Boolean(catalogue.active_session_id) ? 'waiting-global' : 'idle'
      );
      return;
    }
    replicaTactical.setAutoStatus('idle');
    const nextSelection = catalogueSelection(
      entry,
      local ? 'robot' : 'fleet',
      local ? (mapStore.viewRobot ?? preferred ?? 'fleet') : 'fleet'
    );
    if (current && selectionKey(nextSelection) === selectionKey(current)) return;
    replicaTactical.show(nextSelection, 'auto');
  }

  async function refresh() {
    if (refreshController) return;
    const controller = new AbortController();
    refreshController = controller;
    try {
      catalogue = await fetchReplicaCatalogue(undefined, controller.signal);
      replicaTactical.setActiveMissionPresent(Boolean(catalogue.active_session_id));
      apply();
    } catch (reason) {
      if (!(reason instanceof DOMException && reason.name === 'AbortError')) {
        // The visible selector reports catalogue failures; auto mode simply
        // keeps the last coherent selection through a transient outage.
      }
    } finally {
      if (refreshController === controller) refreshController = null;
    }
  }

  // Re-evaluate immediately when the operator changes local robot focus or
  // fleet selection; the ten-second poll is only for catalogue publication.
  $effect(() => {
    enabled;
    replicaTactical.preference;
    mapStore.viewMode;
    mapStore.viewRobot;
    fleet.selected.join(',');
    fleet.robots.map((robot) => robot.robot_id).join(',');
    apply();
  });

  onMount(() => {
    void refresh();
    const timer = window.setInterval(() => void refresh(), 10_000);
    return () => {
      window.clearInterval(timer);
      refreshController?.abort();
    };
  });
</script>
