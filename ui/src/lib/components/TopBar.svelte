<script lang="ts">
  import { Octagon, Circle, Layers3, Settings2 } from 'lucide-svelte';
  import Button from './ui/Button.svelte';
  import Badge from './ui/Badge.svelte';
  import StatusDot from './ui/StatusDot.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { session } from '$lib/stores/session.svelte';
  import { actions } from '$lib/api/connection';

  let { onsettings = () => {} }: { onsettings?: () => void } = $props();

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
