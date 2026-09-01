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
  import { cortexStore } from '$lib/stores/agent.svelte';
  import { Brain, PanelLeftOpen } from 'lucide-svelte';

  let videoExpanded = $state(false);
  let compactLayout = $state(false);
  let responsiveLayoutInitialised = false;
  let settingsOpen = $state(false);
  let swarmSlamOpen = $state(false);

  function openFleet() {
    cortexStore.openFleet();
    if (compactLayout) videoExpanded = false;
  }

  function openCortex() {
    cortexStore.openCortex();
    if (compactLayout) videoExpanded = false;
  }

  function toggleVideoExpanded() {
    videoExpanded = !videoExpanded;
    if (videoExpanded && compactLayout) cortexStore.close();
  }

  function dismissOverlayPanels() {
    if (!compactLayout) return;
    cortexStore.close();
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
        if (media.matches) cortexStore.close();
      }
    };

    const onKeyDown = (event: KeyboardEvent) => {
      if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === 'k') {
        event.preventDefault();
        cortexStore.toggleCortex();
        return;
      }
      if (event.key !== 'Escape') return;
      if (cortexStore.isOpen && compactLayout) {
        cortexStore.close();
        return;
      }
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
    {#if compactLayout && cortexStore.isOpen}
      <button
        class="workspace-scrim"
        aria-label="Close open panel"
        onclick={dismissOverlayPanels}
      ></button>
    {/if}

    {#if cortexStore.isOpen}
      <div class="side-panel fleet-panel">
        <FleetRail oncollapse={() => cortexStore.close()} />
      </div>
    {:else}
      <div class="panel-tab panel-tab-left flex items-center gap-1 p-0.5 bg-surface/95 border border-border/80 rounded-full shadow-md backdrop-blur-md z-30">
        <button
          class="flex items-center gap-1.5 px-2.5 py-1 rounded-full text-[11px] font-semibold transition-colors hover:bg-surface-3 text-fg"
          aria-label="Show Fleet panel"
          onclick={openFleet}
        >
          <PanelLeftOpen class="h-3.5 w-3.5 text-fg-dim" />
          <span>Fleet</span>
          <span class="panel-tab-count">{fleet.count}</span>
        </button>
        <div class="h-3.5 w-px bg-border/80"></div>
        <button
          class="flex items-center gap-1.5 px-2.5 py-1 rounded-full text-[11px] font-semibold transition-colors hover:bg-cyan-950/60 text-cyan-300"
          aria-label="Show Cortex AI Chat"
          onclick={openCortex}
        >
          <Brain class="h-3.5 w-3.5 text-cyan-400" />
          <span>Cortex</span>
          {#if cortexStore.isStreaming}
            <span class="h-1.5 w-1.5 rounded-full bg-cyan-300 animate-ping"></span>
          {/if}
        </button>
      </div>
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
