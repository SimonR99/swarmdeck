<script lang="ts">
  import {
    Brain,
    Bug,
    Circle,
    Globe,
    Maximize2,
    Minimize2,
    Octagon,
    RotateCcw,
    Settings2,
    Sparkles,
  } from 'lucide-svelte';
  import Button from './ui/Button.svelte';
  import Badge from './ui/Badge.svelte';
  import StatusDot from './ui/StatusDot.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { mapStore } from '$lib/stores/mapstore.svelte';
  import { session } from '$lib/stores/session.svelte';
  import { cortexStore } from '$lib/stores/agent.svelte';
  import { robotDisplayName } from '$lib/robotDisplayName';
  import { actions } from '$lib/api/connection';

  let {
    onsettings = () => {},
    onswarmslam = () => {}
  }: {
    onsettings?: () => void;
    onswarmslam?: () => void;
  } = $props();

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

  // Capability-gated, not build-gated: this same GUI drives adapter_ros2 on real
  // hardware, where "teleport to spawn and forget the map" is not a thing that
  // can happen. No robot advertising `reset` means no button at all.
  const canReset = $derived(fleet.robots.some((r) => r.capabilities?.includes('reset')));

  // Which map the canvas is showing. This used to be a single button that
  // cycled, labelled with the mode it was ALREADY in — so it read as a status
  // line and gave no hint that the other view existed. Two explicit segments
  // show both options and which one is active.
  const selected = $derived(fleet.selected[0] ?? null);
  const isLocal = $derived(mapStore.viewMode === 'local');
  const members = $derived(mapStore.status?.global_members?.length ?? 0);
  const swarmGraphCount = $derived(
    Object.keys(mapStore.slamGraphs).filter((id) => fleet.isEnabled(id)).length
  );
  // A robot the merge has not accepted has no place on the shared map, so its
  // own map is the only honest thing to draw. Say so rather than letting the
  // operator wonder why Global looks empty for it.
  const inGlobal = $derived(!selected || (mapStore.status?.global_members ?? []).includes(selected));

  function showGlobal() {
    void mapStore.setViewPreference('global', selected);
  }
  function showLocal() {
    if (selected) void mapStore.setViewPreference('local', selected);
  }

  // Two clicks, because a reset throws away every map the fleet has built and
  // there is no undo. The armed state lapses on its own so a stray first click
  // cannot leave the button primed indefinitely.
  let armed = $state(false);
  let armedTimer: ReturnType<typeof setTimeout> | null = null;

  function disarm() {
    armed = false;
    if (armedTimer) clearTimeout(armedTimer);
    armedTimer = null;
  }

  function onReset() {
    if (session.resetting) return;
    if (!armed) {
      armed = true;
      armedTimer = setTimeout(disarm, 4000);
      return;
    }
    disarm();
    actions.resetSim();
  }
</script>

<header
  class="top-bar relative z-40 flex h-16 shrink-0 items-center gap-3 border-b border-border/70
         bg-surface/95 px-5 shadow-[0_2px_10px_-8px_rgb(25_32_42/0.4)] backdrop-blur-xl"
>
  <div class="flex shrink-0 items-center gap-2.5">
    <img src="/logo.png" alt="SwarmDeck Logo" class="h-8 w-auto max-w-[140px] object-contain" />
  </div>

  <div class="mx-1 h-6 w-px bg-border/80"></div>

  <div class="flex shrink-0 items-center gap-2">
    <StatusDot tone={connTone as never} pulse={session.connection === 'connecting'} />
    <span class="text-[11px] font-medium capitalize text-fg-muted">
      {session.connection === 'mock' ? 'simulated' : session.connection}
    </span>
  </div>

  <Badge tone="neutral" class="hidden md:inline-flex">{fleet.online} of {fleet.count} online</Badge>

  <div class="mx-1 h-6 w-px bg-border/80"></div>

  <!--
    Which map is on screen. Both options are always visible, so the view is a
    choice the operator can see rather than a mode they have to discover.
  -->
  <div
    class="topbar-map flex items-center rounded-full bg-surface-3 p-1"
    role="group"
    aria-label="Map view"
  >
    <button
      class="flex h-8 items-center gap-1.5 rounded-full px-3 text-[11px] font-semibold
             transition-colors {!isLocal
        ? 'bg-accent text-white shadow-[0_2px_6px_-4px_rgb(47_99_199/0.8)]'
        : 'text-fg-dim hover:text-fg-muted'}"
      aria-pressed={!isLocal}
      title="The merged fleet map — every registered robot's grid in one frame"
      onclick={showGlobal}
    >
      <Globe class="h-3 w-3" />
      Global
      <span class="tabular opacity-60">{members}/{fleet.count}</span>
    </button>
    <button
      class="flex h-8 items-center gap-1.5 rounded-full px-3 text-[11px] font-semibold
             transition-colors disabled:opacity-40 {isLocal
        ? 'bg-accent text-white shadow-[0_2px_6px_-4px_rgb(47_99_199/0.8)]'
        : 'text-fg-dim hover:text-fg-muted'}"
      aria-pressed={isLocal}
      disabled={!selected}
      title={selected
        ? "This robot's own SLAM map, in its own frame — how you tell a bad merge from a bad map"
        : 'Select a robot to view its own map'}
      onclick={showLocal}
    >
      Local
      {#if selected}
        <span class="opacity-60">{robotDisplayName(selected)}</span>
      {/if}
    </button>
  </div>

  {#if session.recording}
    <Badge tone="danger">
      <Circle class="h-2 w-2 fill-current" /> REC
    </Badge>
  {/if}

  <div class="flex-1"></div>

  <div class="flex shrink-0 items-center gap-2">
    {#if session.name}
      <span class="hidden text-[11px] text-fg-dim sm:inline">{session.name}</span>
    {/if}
    <span class="tabular text-xs font-medium text-fg-muted">{elapsed}</span>
    <Button
      variant="ghost"
      size="sm"
      class="px-2 sm:px-3"
      title="Swarm SLAM — merge health and reconstruction settings"
      onclick={onswarmslam}
    >
      <Bug class="h-4 w-4" />
      <span class="sr-only">Swarm SLAM</span>
      {#if swarmGraphCount > 0}
        <span
          class="grid h-5 min-w-5 place-items-center rounded-full bg-accent-container px-1
                 text-[10px] font-bold tabular text-accent-container-fg"
        >
          {swarmGraphCount}
        </span>
      {/if}
    </Button>
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
    <button
      class="flex h-8 items-center gap-1.5 rounded-lg px-2.5 text-xs font-semibold transition-all border shadow-xs {cortexStore.isOpen
        ? 'bg-cyan-500/20 text-cyan-300 border-cyan-500/50 shadow-cyan-500/20'
        : 'bg-slate-800/80 hover:bg-slate-700/80 text-slate-300 hover:text-white border-slate-700/60'}"
      title="Cortex — AI Fleet Intelligence & Codebase Copilot (Ctrl+K)"
      onclick={() => cortexStore.toggle()}
    >
      <Brain class="h-4 w-4 text-cyan-400" />
      <span class="hidden sm:inline">Cortex</span>
      {#if cortexStore.isStreaming}
        <span class="h-2 w-2 rounded-full bg-cyan-400 animate-ping"></span>
      {:else}
        <span class="h-1.5 w-1.5 rounded-full bg-emerald-400 shadow-[0_0_6px_rgba(52,211,153,0.8)]"></span>
      {/if}
    </button>
    <Button variant="ghost" size="sm" title="Settings" onclick={onsettings} class="px-2">
      <Settings2 class="h-4 w-4" />
    </Button>
    <Button variant="outline" size="sm" onclick={() => fleet.selectAll()} class="hidden xl:inline-flex">
      {fleet.selected.length === fleet.count && fleet.count > 0 ? 'Deselect' : 'Select all'}
    </Button>
    {#if canReset}
      <Button
        variant={armed ? 'danger' : 'outline'}
        size="sm"
        class="hidden 2xl:inline-flex"
        disabled={session.resetting}
        title="Return the simulation to its start state: robots back at their spawn poses, every map discarded"
        onclick={onReset}
      >
        <RotateCcw class="h-3.5 w-3.5 {session.resetting ? 'animate-spin' : ''}" />
        {session.resetting ? 'Resetting…' : armed ? 'Discard maps?' : 'Reset sim'}
      </Button>
    {/if}
    <Button variant="danger" size="sm" onclick={() => actions.stopAll()}>
      <Octagon class="h-3.5 w-3.5" /> Stop all
    </Button>
  </div>
</header>
