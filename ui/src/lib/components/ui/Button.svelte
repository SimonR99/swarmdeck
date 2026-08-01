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
      'bg-surface-2 text-fg border border-border hover:bg-surface-3 active:bg-surface-3',
    primary:
      'bg-accent text-accent-fg border border-accent hover:brightness-110 active:brightness-95 font-semibold',
    ghost: 'bg-transparent text-fg-muted border border-transparent hover:bg-surface-2 hover:text-fg',
    danger:
      'bg-critical text-white border border-critical hover:brightness-110 active:brightness-95 font-semibold',
    outline: 'bg-transparent text-fg border border-border-strong hover:bg-surface-2'
  };

  const sizes: Record<Size, string> = {
    sm: 'h-9 px-3 text-xs gap-1.5 rounded-lg',
    md: 'h-11 px-4 text-sm gap-2 rounded-xl',
    lg: 'h-13 px-5 text-base gap-2.5 rounded-xl'
  };
</script>

<button
  {title}
  {disabled}
  {onclick}
  class="inline-flex touch-target items-center justify-center whitespace-nowrap
         transition-[background,filter,border-color] duration-150 select-none
         disabled:opacity-40 disabled:pointer-events-none
         {variants[variant]} {sizes[size]} {active ? 'ring-2 ring-accent/70' : ''} {klass}"
>
  {@render children?.()}
</button>
