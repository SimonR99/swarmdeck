<script lang="ts">
  import { Brain, PanelLeftClose, Trash2, UsersRound } from 'lucide-svelte';
  import RobotCard from './RobotCard.svelte';
  import CortexChatView from '$lib/components/agent/CortexChatView.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { settings } from '$lib/stores/settings.svelte';
  import { cortexStore } from '$lib/stores/agent.svelte';

  let { oncollapse = () => {} }: { oncollapse?: () => void } = $props();
</script>

<aside
  class="panel-glow flex h-full w-full flex-col overflow-hidden rounded-[--radius-panel]
         border border-transparent bg-surface-2"
>
  <header class="flex h-12 shrink-0 items-center justify-between border-b border-border/70 px-2.5 bg-surface-2">
    <!-- Segmented Tab: Fleet vs Cortex -->
    <div
      class="flex items-center rounded-lg bg-surface-3/90 p-0.5 border border-border/60"
      role="tablist"
      aria-label="Sidebar view"
    >
      <button
        role="tab"
        aria-selected={cortexStore.activeTab === 'fleet'}
        class="flex h-7 items-center gap-1.5 rounded-md px-2.5 text-[11px] font-semibold transition-all {cortexStore.activeTab === 'fleet'
          ? 'bg-surface text-fg shadow-xs'
          : 'text-fg-dim hover:text-fg'}"
        onclick={() => (cortexStore.activeTab = 'fleet')}
      >
        <UsersRound class="h-3.5 w-3.5 text-accent-container-fg" />
        <span>Fleet</span>
        <span class="tabular text-[10px] opacity-75">{fleet.online}/{fleet.count}</span>
      </button>

      <button
        role="tab"
        aria-selected={cortexStore.activeTab === 'cortex'}
        class="flex h-7 items-center gap-1.5 rounded-md px-2.5 text-[11px] font-semibold transition-all {cortexStore.activeTab === 'cortex'
          ? 'bg-cyan-600 text-white shadow-xs'
          : 'text-fg-dim hover:text-fg'}"
        onclick={() => (cortexStore.activeTab = 'cortex')}
      >
        <Brain class="h-3.5 w-3.5 {cortexStore.activeTab === 'cortex' ? 'text-white' : 'text-cyan-400'}" />
        <span>Cortex</span>
        {#if cortexStore.isStreaming}
          <span class="h-1.5 w-1.5 rounded-full bg-cyan-300 animate-ping"></span>
        {/if}
      </button>
    </div>

    <div class="flex items-center gap-0.5">
      {#if cortexStore.activeTab === 'cortex'}
        <button
          class="grid h-7 w-7 place-items-center rounded-md text-fg-dim hover:bg-surface-3 hover:text-fg transition-colors"
          title="Clear chat history"
          aria-label="Clear chat history"
          onclick={() => cortexStore.clear()}
        >
          <Trash2 class="h-3.5 w-3.5" />
        </button>
      {/if}
      <button
        class="grid h-7 w-7 place-items-center rounded-md text-fg-dim hover:bg-surface-3 hover:text-fg transition-colors"
        title="Collapse panel"
        aria-label="Collapse panel"
        onclick={oncollapse}
      >
        <PanelLeftClose class="h-4 w-4" />
      </button>
    </div>
  </header>

  {#if cortexStore.activeTab === 'fleet'}
    <div class="flex min-h-0 flex-1 flex-col gap-1.5 overflow-y-auto p-2">
      {#each fleet.robots as robot (robot.robot_id)}
        <RobotCard {robot} unattendedThreshold={settings.value.unattended_threshold_s} />
      {/each}

      {#if fleet.count === 0}
        <div
          class="grid place-items-center rounded-[--radius-card] border border-dashed border-border
                 px-4 py-8 text-center text-xs text-fg-dim"
        >
          No robots connected
        </div>
      {/if}
    </div>
  {:else}
    <div class="flex min-h-0 flex-1 flex-col overflow-hidden">
      <CortexChatView />
    </div>
  {/if}
</aside>
