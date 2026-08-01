<script lang="ts">
  let {
    value = 0,
    warn = 0.3,
    critical = 0.15
  }: { value?: number; warn?: number; critical?: number } = $props();

  const pct = $derived(Math.max(0, Math.min(1, value)) * 100);
  const tone = $derived(
    value <= critical ? 'var(--color-critical)' : value <= warn ? 'var(--color-warn)' : 'var(--color-ok)'
  );
</script>

<div class="h-1.5 w-full overflow-hidden rounded-full bg-surface-3">
  <div
    class="h-full rounded-full transition-[width] duration-500"
    style="width:{pct}%; background:{tone}"
  ></div>
</div>
