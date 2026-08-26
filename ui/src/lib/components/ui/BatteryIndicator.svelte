<script lang="ts">
  let {
    value = null,
    showPercentage = true,
    class: className = ''
  }: {
    value?: number | null;
    showPercentage?: boolean;
    class?: string;
  } = $props();

  const isAvailable = $derived(value !== null && value !== undefined && !Number.isNaN(value));
  const normalized = $derived(isAvailable ? Math.max(0, Math.min(1, value!)) : 0);
  const pct = $derived(Math.round(normalized * 100));

  const tone = $derived(
    !isAvailable
      ? 'muted'
      : normalized <= 0.15
        ? 'danger'
        : normalized <= 0.30
          ? 'warn'
          : 'ok'
  );

  const fillStyle = $derived(
    tone === 'danger'
      ? 'bg-danger'
      : tone === 'warn'
        ? 'bg-warn'
        : tone === 'ok'
          ? 'bg-ok'
          : 'bg-transparent'
  );

  const borderClass = $derived(
    tone === 'danger'
      ? 'border-danger/80 text-danger'
      : tone === 'warn'
        ? 'border-warn/80 text-warn'
        : tone === 'ok'
          ? 'border-fg-muted/70 text-fg-muted'
          : 'border-border-strong/50 text-fg-dim'
  );

  const textClass = $derived(
    tone === 'danger'
      ? 'text-danger font-bold'
      : tone === 'warn'
        ? 'text-warn font-semibold'
        : 'text-fg-muted font-medium'
  );

  const titleText = $derived(
    !isAvailable
      ? 'Battery level unavailable'
      : `Battery: ${pct}%${pct <= 15 ? ' (Critical)' : pct <= 30 ? ' (Low)' : ' (Normal)'}`
  );
</script>

<div
  class="inline-flex shrink-0 items-center gap-1.5 select-none {className}"
  title={titleText}
  aria-label={titleText}
>
  <!-- Phone-style horizontal battery icon with terminal cap -->
  <div class="relative flex items-center">
    <div
      class="flex h-[11px] w-[20px] items-center rounded-[3px] border-[1.2px] p-[1.5px] transition-colors {borderClass}"
    >
      <div
        class="h-full rounded-[1px] transition-all duration-500 ease-out {fillStyle}"
        style="width: {isAvailable ? Math.max(8, pct) : 0}%;"
      ></div>
    </div>
    <!-- Battery positive terminal nub -->
    <div
      class="h-[4.5px] w-[1.5px] -ml-[0.5px] rounded-r-[1px] opacity-75 {tone === 'danger' ? 'bg-danger' : tone === 'warn' ? 'bg-warn' : 'bg-fg-muted/60'}"
    ></div>
  </div>

  {#if showPercentage}
    <span class="text-[10px] leading-none tabular {textClass}">
      {isAvailable ? `${pct}%` : '--%'}
    </span>
  {/if}
</div>
