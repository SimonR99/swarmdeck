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
  import { PanelLeftOpen, PanelRightOpen } from 'lucide-svelte';

  let fleetOpen = $state(true);
  let controlsOpen = $state(true);
  let videoExpanded = $state(false);
  let compactLayout = $state(false);
  let responsiveLayoutInitialised = false;
  let settingsOpen = $state(false);

  function openFleet() {
    fleetOpen = true;
    if (compactLayout) {
      controlsOpen = false;
      videoExpanded = false;
    }
  }

  function openControls() {
    controlsOpen = true;
    if (compactLayout) fleetOpen = false;
  }

  function closeControls() {
    videoExpanded = false;
    controlsOpen = false;
  }

  function toggleVideoExpanded() {
    videoExpanded = !videoExpanded;
    controlsOpen = true;
    if (videoExpanded && compactLayout) fleetOpen = false;
  }

  function dismissOverlayPanels() {
    if (!compactLayout) return;
    fleetOpen = false;
    controlsOpen = false;
    videoExpanded = false;
  }

  $effect(() => {
    startConnection();
    return () => teardown();
  });

  // Tablets start map-first. Panels remain one tap away without squeezing the
  // operational canvas into a narrow strip. After initialisation, the operator
  // remains in control of which panel is open.
  $effect(() => {
    const media = window.matchMedia('(max-width: 1180px)');
    const syncLayout = () => {
      compactLayout = media.matches;
      if (!responsiveLayoutInitialised) {
        responsiveLayoutInitialised = true;
        if (media.matches) {
          fleetOpen = false;
          controlsOpen = false;
        }
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
  <TopBar onsettings={() => (settingsOpen = true)} />

  <main class="workspace">
    {#if compactLayout && (fleetOpen || controlsOpen)}
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

    {#if controlsOpen}
      <aside
        class="side-panel control-panel"
        class:video-expanded={videoExpanded}
        aria-label="Robot control panel"
      >
        <CameraPanel
          expanded={videoExpanded}
          ontoggleexpand={toggleVideoExpanded}
          oncollapse={closeControls}
        />
        {#if !videoExpanded}
          <DrivePanel />
          <SwarmGraphPanel />
        {/if}
      </aside>
    {:else}
      <button
        class="panel-tab panel-tab-right"
        aria-label="Show Robot Control panel"
        onclick={openControls}
      >
        <PanelRightOpen class="h-4 w-4" />
        <span>Controls</span>
      </button>
    {/if}
  </main>
</div>

<SettingsModal open={settingsOpen} onclose={() => (settingsOpen = false)} />
