<script lang="ts">
  import type { Snippet } from 'svelte';

  type Tone = 'neutral' | 'ok' | 'warn' | 'danger' | 'accent';

  let {
    tone = 'neutral',
    children,
    class: klass = ''
  }: { tone?: Tone; children?: Snippet; class?: string } = $props();

  const tones: Record<Tone, string> = {
    neutral: 'bg-surface-2 text-fg-muted border-border',
    ok: 'bg-ok/12 text-ok border-ok/30',
    warn: 'bg-warn/12 text-warn border-warn/30',
    danger: 'bg-critical/15 text-danger border-critical/35',
    accent: 'bg-accent/12 text-accent border-accent/30'
  };
</script>

<span
  class="inline-flex items-center gap-1 rounded-full border px-2 py-0.5
         text-[10px] font-semibold tracking-[0.01em] {tones[tone]} {klass}"
>
  {@render children?.()}
</span>
