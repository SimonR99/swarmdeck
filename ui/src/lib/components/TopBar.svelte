<script lang="ts">
  import { Octagon, Radio, Circle, Layers } from 'lucide-svelte';
  import Button from './ui/Button.svelte';
  import Badge from './ui/Badge.svelte';
  import StatusDot from './ui/StatusDot.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { session } from '$lib/stores/session.svelte';
  import { actions } from '$lib/api/connection';

  const elapsed = $derived(
    `${String(Math.floor(session.elapsed_s / 60)).padStart(2, '0')}:${String(
      Math.floor(session.elapsed_s % 60)
    ).padStart(2, '0')}`
  );

  const connTone = $derived(
    session.connection === 'live' ? 'ok' : session.connection === 'mock' ? 'warn' : 'danger'
  );
</script>

<header
  class="flex h-14 shrink-0 items-center gap-3 border-b border-border bg-surface px-3"
>
  <div class="flex items-center gap-2">
    <div class="grid h-8 w-8 place-items-center rounded-lg bg-accent/15">
      <Layers class="h-4 w-4 text-accent" />
    </div>
    <span class="text-sm font-bold tracking-tight">SwarmDeck</span>
  </div>

  <div class="mx-1 h-6 w-px bg-border"></div>

  <div class="flex items-center gap-2">
    <StatusDot tone={connTone as never} pulse={session.connection === 'connecting'} />
    <span class="text-[11px] uppercase tracking-wide text-fg-muted">
      {session.connection === 'mock' ? 'simulated' : session.connection}
    </span>
  </div>

  <Badge tone="neutral">{fleet.online}/{fleet.count} online</Badge>

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
    <span class="tabular text-sm font-semibold text-fg-muted">{elapsed}</span>
    <Button variant="outline" size="sm" onclick={() => fleet.selectAll()}>
      {fleet.selected.length === fleet.count && fleet.count > 0 ? 'Deselect' : 'Select all'}
    </Button>
    <Button variant="danger" size="md" onclick={() => actions.stopAll()}>
      <Octagon class="h-4 w-4" /> STOP ALL
    </Button>
  </div>
</header>
