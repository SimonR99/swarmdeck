<script lang="ts">
  import type { Snippet } from 'svelte';

  type Tone = 'neutral' | 'ok' | 'warn' | 'danger' | 'accent';

  let {
    tone = 'neutral',
    children,
    class: klass = ''
  }: { tone?: Tone; children?: Snippet; class?: string } = $props();

  const tones: Record<Tone, string> = {
    neutral: 'bg-surface-3 text-fg-muted border-transparent',
    ok: 'bg-ok/12 text-ok border-transparent',
    warn: 'bg-warn/12 text-warn border-transparent',
    danger: 'bg-critical/12 text-danger border-transparent',
    accent: 'bg-accent-container text-accent-container-fg border-transparent'
  };
</script>

<span
  class="inline-flex min-h-6 items-center gap-1 rounded-full border px-2.5 py-0.5
         text-[10px] font-semibold tracking-[0.01em] {tones[tone]} {klass}"
>
  {@render children?.()}
</span>
