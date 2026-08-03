<script lang="ts">
  import type { Snippet } from 'svelte';

  let {
    padded = true,
    interactive = false,
    selected = false,
    accent = '',
    onclick,
    children,
    class: klass = ''
  }: {
    padded?: boolean;
    interactive?: boolean;
    selected?: boolean;
    accent?: string;
    onclick?: (e: MouseEvent) => void;
    children?: Snippet;
    class?: string;
  } = $props();
</script>

<svelte:element
  this={interactive ? 'button' : 'div'}
  role={interactive ? 'button' : undefined}
  {onclick}
  style={`${accent ? `--card-accent:${accent}` : ''}${selected && accent ? `;border-color:${accent}` : ''}`}
  class="relative w-full overflow-hidden rounded-[--radius-card] border bg-surface text-left
         shadow-[0_1px_2px_rgb(0_0_0/0.025)] transition-colors duration-150
         {selected ? 'bg-surface shadow-sm' : 'border-border'}
         {interactive ? 'hover:border-border-strong hover:bg-surface-2/40' : ''}
         {padded ? 'p-3' : ''} {klass}"
>
  {#if accent}
    <span
      class="absolute inset-y-0 left-0 w-0.5 transition-opacity duration-150"
      style="background:{accent}; opacity:{selected ? 1 : 0}"
    ></span>
  {/if}
  {@render children?.()}
</svelte:element>
