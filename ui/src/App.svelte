<script lang="ts">
  import TopBar from '$lib/components/TopBar.svelte';
  import FleetRail from '$lib/components/fleet/FleetRail.svelte';
  import MapView from '$lib/components/map2d/MapView.svelte';
  import CameraPanel from '$lib/components/video/CameraPanel.svelte';
  import AlertStack from '$lib/components/alerts/AlertStack.svelte';
  import { startConnection } from '$lib/api/connection';
  import { PanelRightClose, PanelRightOpen } from 'lucide-svelte';

  let cameraOpen = $state(true);

  $effect(() => {
    startConnection();
  });
</script>

<div class="flex h-full flex-col overflow-hidden">
  <TopBar />

  <main class="relative flex min-h-0 flex-1 gap-2 p-2">
    <!-- Fleet rail: fixed width, scrolls for up to 5 robots -->
    <div class="w-[228px] shrink-0 lg:w-[248px]">
      <FleetRail />
    </div>

    <!-- Map: takes all remaining space -->
    <div class="relative min-w-0 flex-1">
      <MapView />
      <AlertStack />
    </div>

    <!-- Camera: collapsible so the map can go full width on a 10in tablet -->
    {#if cameraOpen}
      <div class="w-[288px] shrink-0 xl:w-[336px]">
        <CameraPanel />
      </div>
    {/if}

    <button
      class="absolute right-2 top-2 z-40 grid h-9 w-9 place-items-center rounded-lg border
             border-border bg-surface/90 text-fg-muted backdrop-blur transition-colors
             hover:bg-surface-2 {cameraOpen ? 'hidden' : ''}"
      title="Show camera"
      onclick={() => (cameraOpen = true)}
    >
      <PanelRightOpen class="h-4 w-4" />
    </button>

    {#if cameraOpen}
      <button
        class="absolute right-[296px] top-2 z-40 grid h-9 w-9 place-items-center rounded-lg
               border border-border bg-surface/90 text-fg-muted backdrop-blur
               transition-colors hover:bg-surface-2 xl:right-[344px]"
        title="Hide camera"
        onclick={() => (cameraOpen = false)}
      >
        <PanelRightClose class="h-4 w-4" />
      </button>
    {/if}
  </main>
</div>
