<script lang="ts">
  type Tone = 'ok' | 'warn' | 'danger' | 'idle';
  let { tone = 'idle', pulse = false }: { tone?: Tone; pulse?: boolean } = $props();

  const colors: Record<Tone, string> = {
    ok: 'var(--color-ok)',
    warn: 'var(--color-warn)',
    danger: 'var(--color-critical)',
    idle: 'var(--color-fg-dim)'
  };
</script>

<span class="relative inline-flex h-2 w-2 shrink-0">
  {#if pulse}
    <span
      class="animate-ping-ring absolute inline-flex h-full w-full rounded-full"
      style="color:{colors[tone]}"
    ></span>
  {/if}
  <span class="relative inline-flex h-2 w-2 rounded-full" style="background:{colors[tone]}"></span>
</span>
