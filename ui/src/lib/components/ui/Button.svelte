<script lang="ts">
  import type { Snippet } from 'svelte';

  type Variant = 'default' | 'primary' | 'ghost' | 'danger' | 'outline';
  type Size = 'sm' | 'md' | 'lg';

  let {
    variant = 'default',
    size = 'md',
    active = false,
    disabled = false,
    title = '',
    onclick,
    children,
    class: klass = ''
  }: {
    variant?: Variant;
    size?: Size;
    active?: boolean;
    disabled?: boolean;
    title?: string;
    onclick?: (e: MouseEvent) => void;
    children?: Snippet;
    class?: string;
  } = $props();

  const variants: Record<Variant, string> = {
    default:
      'bg-surface text-fg border border-border hover:border-border-strong hover:bg-surface-2 active:bg-surface-3',
    primary:
      'bg-accent text-accent-fg border border-accent hover:brightness-95 active:brightness-90 font-semibold',
    ghost: 'bg-transparent text-fg-muted border border-transparent hover:bg-surface-2 hover:text-fg',
    danger:
      'bg-critical text-white border border-critical hover:brightness-95 active:brightness-90 font-semibold',
    outline: 'bg-surface text-fg border border-border hover:border-border-strong hover:bg-surface-2'
  };

  const sizes: Record<Size, string> = {
    sm: 'h-9 px-3 text-[11px] gap-1.5 rounded-[--radius-control]',
    md: 'h-10 px-4 text-xs gap-2 rounded-[--radius-control]',
    lg: 'h-11 px-5 text-sm gap-2.5 rounded-[--radius-control]'
  };
</script>

<button
  {title}
  {disabled}
  {onclick}
  class="inline-flex touch-target items-center justify-center whitespace-nowrap
         shadow-[0_1px_2px_rgb(16_24_40/0.04)] transition-[background,filter,border-color] duration-150 select-none
         disabled:opacity-40 disabled:pointer-events-none
         {variants[variant]} {sizes[size]} {active ? 'ring-2 ring-accent/70' : ''} {klass}"
>
  {@render children?.()}
</button>
