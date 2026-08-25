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
    onclick?: (e: MouseEvent | KeyboardEvent) => void;
    children?: Snippet;
    class?: string;
  } = $props();

  function onKeyDown(event: KeyboardEvent) {
    if (
      !interactive ||
      event.target !== event.currentTarget ||
      (event.key !== 'Enter' && event.key !== ' ')
    ) return;
    event.preventDefault();
    onclick?.(event);
  }
</script>

{#snippet content()}
  {#if accent}
    <span
      class="absolute inset-y-0 left-0 w-0.5 transition-opacity duration-150"
      style="background:var(--color-accent); opacity:{selected ? 1 : 0}"
    ></span>
  {/if}
  {@render children?.()}
{/snippet}

{#if interactive}
  <div
    role="button"
    tabindex="0"
    {onclick}
    onkeydown={onKeyDown}
    style={accent ? `--card-accent:${accent}` : undefined}
    class="relative w-full cursor-pointer overflow-hidden rounded-[--radius-card] border text-left
           transition-[background,border-color,box-shadow,transform] duration-150
           {selected
             ? 'border-accent/25 bg-accent-container shadow-[0_2px_8px_-5px_rgb(47_99_199/0.5)]'
             : 'border-transparent bg-surface shadow-[0_1px_3px_rgb(25_32_42/0.06)]'}
           hover:bg-surface-3 active:scale-[0.995]
           {padded ? 'p-3' : ''} {klass}"
  >
    {@render content()}
  </div>
{:else}
  <div
    style={accent ? `--card-accent:${accent}` : undefined}
    class="relative w-full overflow-hidden rounded-[--radius-card] border text-left
           transition-[background,border-color,box-shadow] duration-150
           {selected
             ? 'border-accent/25 bg-accent-container shadow-[0_2px_8px_-5px_rgb(47_99_199/0.5)]'
             : 'border-transparent bg-surface shadow-[0_1px_3px_rgb(25_32_42/0.06)]'}
           {padded ? 'p-3' : ''} {klass}"
  >
    {@render content()}
  </div>
{/if}
