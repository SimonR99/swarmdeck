<script lang="ts">
  import { onMount } from 'svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { mapStore } from '$lib/stores/mapstore.svelte';
  import { replicaCatalogue } from '$lib/stores/replicaCatalogue.svelte';
  import { replicaTactical } from '$lib/stores/replicaTactical.svelte';
  import {
    automaticCatalogueEntry,
    automaticSelectionIsCoherent,
    activeMergedRobotIds,
    catalogueSelection
  } from './replicaCatalogue';

  let { enabled = true } = $props<{ enabled?: boolean }>();

  function preferredRobotId() {
    if (mapStore.viewMode === 'local' && mapStore.viewRobot) return mapStore.viewRobot;
    return fleet.selected[0] ?? fleet.robotIds[0] ?? null;
  }

  function selectionKey(selection: NonNullable<typeof replicaTactical.selection>) {
    return `${selection.scope ?? 'robot'}\u0000${selection.robotId}\u0000${selection.sessionId}\u0000${selection.componentId}`;
  }

  function componentKey(selection: NonNullable<typeof replicaTactical.selection>) {
    return `${selection.sessionId}\u0000${selection.componentId}`;
  }

  function apply() {
    const catalogue = replicaCatalogue.catalogue;
    const current = replicaTactical.selection;
    if (catalogue) {
      replicaTactical.setActiveMissionPresent(Boolean(catalogue.active_session_id));
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
    // The deployment composite is a Global fallback only: a local view reads
    // one robot's own replica, which the composite id cannot address.
    const entry = automaticCatalogueEntry(catalogue, preferred, !local, local ? 1 : 2, !local);
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

  // Re-evaluate when the catalogue publishes something new, and immediately
  // when the operator changes local robot focus or fleet selection. Only the
  // roster is read, never a pose: reading the robots themselves re-ran this
  // effect on every `robot_state` message.
  $effect(() => {
    replicaCatalogue.catalogue;
    enabled;
    replicaTactical.preference;
    mapStore.viewMode;
    mapStore.viewRobot;
    fleet.selected.join(',');
    fleet.robotIds.join(',');
    apply();
  });

  // The ten-second poll is only for catalogue publication.
  onMount(() => replicaCatalogue.subscribe());
</script>
