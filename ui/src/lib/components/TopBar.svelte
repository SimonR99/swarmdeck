<script lang="ts">
  import { Circle, Maximize2, Minimize2, Octagon, Settings2 } from 'lucide-svelte';
  import Button from './ui/Button.svelte';
  import Badge from './ui/Badge.svelte';
  import StatusDot from './ui/StatusDot.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { session } from '$lib/stores/session.svelte';
  import { actions } from '$lib/api/connection';

  let { onsettings = () => {} }: { onsettings?: () => void } = $props();

  type FullscreenDocument = Document & {
    webkitFullscreenElement?: Element | null;
    webkitExitFullscreen?: () => Promise<void> | void;
  };
  type FullscreenRoot = HTMLElement & {
    webkitRequestFullscreen?: () => Promise<void> | void;
  };

  let fullscreen = $state(false);

  function syncFullscreen() {
    const doc = document as FullscreenDocument;
    fullscreen = Boolean(doc.fullscreenElement || doc.webkitFullscreenElement);
  }

  async function toggleFullscreen() {
    const doc = document as FullscreenDocument;
    const root = document.documentElement as FullscreenRoot;
    try {
      if (doc.fullscreenElement || doc.webkitFullscreenElement) {
        if (doc.exitFullscreen) await doc.exitFullscreen();
        else if (doc.webkitExitFullscreen) await doc.webkitExitFullscreen();
      } else if (root.requestFullscreen) {
        await root.requestFullscreen();
      } else if (root.webkitRequestFullscreen) {
        await root.webkitRequestFullscreen();
      }
    } catch (error) {
      // Fullscreen can be refused by browser policy or an embedded webview.
      // Keep the dashboard usable; the browser's own fullscreen control remains
      // the fallback on platforms that do not expose the page API.
      console.warn('[swarmdeck] fullscreen unavailable', error);
    } finally {
      syncFullscreen();
    }
  }

  $effect(() => {
    document.addEventListener('fullscreenchange', syncFullscreen);
    document.addEventListener('webkitfullscreenchange', syncFullscreen);
    syncFullscreen();
    return () => {
      document.removeEventListener('fullscreenchange', syncFullscreen);
      document.removeEventListener('webkitfullscreenchange', syncFullscreen);
    };
  });

  const elapsed = $derived(
    `${String(Math.floor(session.elapsed_s / 60)).padStart(2, '0')}:${String(
      Math.floor(session.elapsed_s % 60)
    ).padStart(2, '0')}`
  );

  const connTone = $derived(
    session.connection === 'live' ? 'ok' : session.connection === 'mock' ? 'warn' : 'danger'
  );
</script>

<header class="flex h-12 shrink-0 items-center gap-2.5 border-b border-border bg-surface px-3">
  <div class="flex items-center gap-2">
    <img src="/logo.png" alt="SwarmDeck Logo" class="h-7 w-auto max-w-[140px] object-contain" />
    <span class="text-[13px] font-semibold tracking-[-0.015em]">SwarmDeck</span>
  </div>

  <div class="mx-1 h-5 w-px bg-border"></div>

  <div class="flex items-center gap-2">
    <StatusDot tone={connTone as never} pulse={session.connection === 'connecting'} />
    <span class="text-[10px] font-medium capitalize text-fg-muted">
      {session.connection === 'mock' ? 'simulated' : session.connection}
    </span>
  </div>

  <Badge tone="neutral">{fleet.online} of {fleet.count} online</Badge>

  {#if session.recording}
    <Badge tone="danger">
      <Circle class="h-2 w-2 fill-current" /> REC
    </Badge>
  {/if}

  <div class="flex-1"></div>

  <div class="flex items-center gap-3">
    {#if session.name}
      <span class="hidden text-[11px] text-fg-dim sm:inline">{session.name}</span>
    {/if}
    <span class="tabular text-xs font-medium text-fg-muted">{elapsed}</span>
    <Button
      variant="ghost"
      size="sm"
      class="px-2"
      title={fullscreen ? 'Exit fullscreen' : 'Enter fullscreen'}
      onclick={() => void toggleFullscreen()}
    >
      {#if fullscreen}
        <Minimize2 class="h-4 w-4" />
      {:else}
        <Maximize2 class="h-4 w-4" />
      {/if}
    </Button>
    <Button variant="ghost" size="sm" title="Settings" onclick={onsettings} class="px-2">
      <Settings2 class="h-4 w-4" />
    </Button>
    <Button variant="outline" size="sm" onclick={() => fleet.selectAll()}>
      {fleet.selected.length === fleet.count && fleet.count > 0 ? 'Deselect' : 'Select all'}
    </Button>
    <Button variant="danger" size="sm" onclick={() => actions.stopAll()}>
      <Octagon class="h-3.5 w-3.5" /> Stop all
    </Button>
  </div>
</header>
