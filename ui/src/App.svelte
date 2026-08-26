<script lang="ts">
  import TopBar from '$lib/components/TopBar.svelte';
  import FleetRail from '$lib/components/fleet/FleetRail.svelte';
  import MapView from '$lib/components/map2d/MapView.svelte';
  import CameraPanel from '$lib/components/video/CameraPanel.svelte';
  import DrivePanel from '$lib/components/controls/DrivePanel.svelte';
  import SwarmGraphPanel from '$lib/components/slam/SwarmGraphPanel.svelte';
  import AlertStack from '$lib/components/alerts/AlertStack.svelte';
  import DetectionReview from '$lib/components/detections/DetectionReview.svelte';
  import SettingsModal from '$lib/components/settings/SettingsModal.svelte';
  import { startConnection, teardown } from '$lib/api/connection';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { PanelLeftOpen } from 'lucide-svelte';

  let fleetOpen = $state(true);
  let videoExpanded = $state(false);
  let compactLayout = $state(false);
  let responsiveLayoutInitialised = false;
  let settingsOpen = $state(false);
  let swarmSlamOpen = $state(false);

  function openFleet() {
    fleetOpen = true;
    if (compactLayout) videoExpanded = false;
  }

  function toggleVideoExpanded() {
    videoExpanded = !videoExpanded;
    if (videoExpanded && compactLayout) fleetOpen = false;
  }

  function dismissOverlayPanels() {
    if (!compactLayout) return;
    fleetOpen = false;
    videoExpanded = false;
  }

  $effect(() => {
    startConnection();
    return () => teardown();
  });

  // Tablets start with Fleet retracted, while Robot Control remains available
  // at all times. After initialisation, the operator controls Fleet visibility.
  $effect(() => {
    const media = window.matchMedia('(max-width: 1180px)');
    const syncLayout = () => {
      compactLayout = media.matches;
      if (!responsiveLayoutInitialised) {
        responsiveLayoutInitialised = true;
        if (media.matches) fleetOpen = false;
      }
    };

    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key !== 'Escape') return;
      if (videoExpanded) videoExpanded = false;
      else dismissOverlayPanels();
    };

    syncLayout();
    media.addEventListener('change', syncLayout);
    window.addEventListener('keydown', onKeyDown);
    return () => {
      media.removeEventListener('change', syncLayout);
      window.removeEventListener('keydown', onKeyDown);
    };
  });
</script>

<div class="flex h-full flex-col overflow-hidden">
  <TopBar
    onsettings={() => (settingsOpen = true)}
    onswarmslam={() => (swarmSlamOpen = true)}
  />

  <main class="workspace">
    {#if compactLayout && fleetOpen}
      <button
        class="workspace-scrim"
        aria-label="Close open panel"
        onclick={dismissOverlayPanels}
      ></button>
    {/if}

    {#if fleetOpen}
      <div class="side-panel fleet-panel">
        <FleetRail oncollapse={() => (fleetOpen = false)} />
      </div>
    {:else}
      <button class="panel-tab panel-tab-left" aria-label="Show Fleet panel" onclick={openFleet}>
        <PanelLeftOpen class="h-4 w-4" />
        <span>Fleet</span>
        <span class="panel-tab-count">{fleet.count}</span>
      </button>
    {/if}

    <section class="map-workspace" aria-label="Fleet map workspace">
      <MapView />
      <AlertStack />
      <DetectionReview />
    </section>

    <aside
      class="side-panel control-panel"
      class:video-expanded={videoExpanded}
      aria-label="Robot control panel"
    >
      <CameraPanel expanded={videoExpanded} ontoggleexpand={toggleVideoExpanded} />
      {#if !videoExpanded}
        <DrivePanel />
      {/if}
    </aside>
  </main>
</div>

<SettingsModal open={settingsOpen} onclose={() => (settingsOpen = false)} />
<SwarmGraphPanel open={swarmSlamOpen} onclose={() => (swarmSlamOpen = false)} />
