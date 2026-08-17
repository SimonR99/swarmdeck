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
  import { PanelRightClose, PanelRightOpen } from 'lucide-svelte';

  let cameraOpen = $state(true);
  let settingsOpen = $state(false);

  $effect(() => {
    startConnection();
    return () => teardown();
  });
</script>

<div class="flex h-full flex-col overflow-hidden">
  <TopBar onsettings={() => (settingsOpen = true)} />

  <main class="relative flex min-h-0 flex-1 gap-2 p-2">
    <!-- Fleet rail: fixed width, scrolls for up to 5 robots -->
    <div class="w-[208px] shrink-0 lg:w-[216px]">
      <FleetRail />
    </div>

    <!-- Map: takes all remaining space -->
    <div class="relative min-w-0 flex-1">
      <MapView />
      <AlertStack />
      <DetectionReview />
    </div>

    <!-- Camera: collapsible so the map can go full width on a 10in tablet -->
    {#if cameraOpen}
      <div class="flex w-[300px] shrink-0 flex-col gap-2 overflow-y-auto xl:w-[320px]">
        <CameraPanel />
        <DrivePanel />
        <SwarmGraphPanel />
      </div>
    {/if}

    <button
      class="panel-glow absolute right-2 top-2 z-40 grid h-8 w-8 place-items-center rounded-[4px]
             border border-border bg-surface/90 text-fg-muted backdrop-blur-xl transition-colors
             hover:bg-surface-2 {cameraOpen ? 'hidden' : ''}"
      title="Show camera"
      onclick={() => (cameraOpen = true)}
    >
      <PanelRightOpen class="h-4 w-4" />
    </button>

    {#if cameraOpen}
      <button
        class="panel-glow absolute right-[310px] top-2 z-40 grid h-8 w-8 place-items-center
               rounded-[4px] border border-border bg-surface/90 text-fg-muted backdrop-blur-xl
               transition-colors hover:bg-surface-2 xl:right-[330px]"
        title="Hide camera"
        onclick={() => (cameraOpen = false)}
      >
        <PanelRightClose class="h-4 w-4" />
      </button>
    {/if}
  </main>
</div>

<SettingsModal open={settingsOpen} onclose={() => (settingsOpen = false)} />
