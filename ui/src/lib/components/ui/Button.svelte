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
      'bg-surface-3 text-fg border border-transparent hover:bg-border active:bg-border-strong/45',
    primary:
      'bg-accent text-accent-fg border border-transparent shadow-[0_2px_6px_-3px_rgb(47_99_199/0.65)] hover:brightness-105 active:brightness-95 font-semibold',
    ghost: 'bg-transparent text-fg-muted border border-transparent shadow-none hover:bg-surface-3 hover:text-fg',
    danger:
      'bg-critical text-white border border-transparent shadow-[0_2px_6px_-3px_rgb(220_38_38/0.65)] hover:brightness-105 active:brightness-95 font-semibold',
    outline: 'bg-transparent text-fg border border-border shadow-none hover:bg-surface-2'
  };

  const sizes: Record<Size, string> = {
    sm: 'h-10 px-4 text-[11px] gap-1.5 rounded-full',
    md: 'h-11 px-5 text-xs gap-2 rounded-full',
    lg: 'h-12 px-6 text-sm gap-2.5 rounded-full'
  };
</script>

<button
  {title}
  {disabled}
  {onclick}
  class="inline-flex touch-target items-center justify-center whitespace-nowrap
         transition-[background,filter,border-color,box-shadow,transform] duration-150 select-none
         active:scale-[0.98]
         disabled:opacity-40 disabled:pointer-events-none
         {variants[variant]} {sizes[size]} {active ? 'ring-4 ring-accent/15' : ''} {klass}"
>
  {@render children?.()}
</button>
