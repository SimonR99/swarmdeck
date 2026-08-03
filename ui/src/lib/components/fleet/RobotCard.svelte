<script lang="ts">
  import { Battery, Navigation, Video, CircleSlash, TriangleAlert } from 'lucide-svelte';
  import Card from '../ui/Card.svelte';
  import Badge from '../ui/Badge.svelte';
  import Meter from '../ui/Meter.svelte';
  import StatusDot from '../ui/StatusDot.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { actions } from '$lib/api/connection';
  import type { RobotState } from '$lib/types/protocol';

  let { robot, unattendedThreshold = 45 }: { robot: RobotState; unattendedThreshold?: number } =
    $props();

  const color = $derived(fleet.colorOf(robot.robot_id));
  const selected = $derived(fleet.isSelected(robot.robot_id));
  const isCamera = $derived(fleet.activeCamera === robot.robot_id);
  const stale = $derived(robot.unattended_s > unattendedThreshold);

  const statusTone = $derived(
    !robot.online
      ? 'danger'
      : robot.mode === 'estop'
        ? 'danger'
        : robot.nav_status === 'failed'
          ? 'warn'
          : robot.nav_status === 'active'
            ? 'ok'
            : 'idle'
  );

  const modeLabel = $derived(
    robot.mode === 'estop'
      ? 'E-STOP'
      : robot.nav_status === 'active'
        ? 'NAVIGATING'
        : robot.nav_status === 'failed'
          ? 'NAV FAILED'
          : robot.mode.toUpperCase()
  );

  const shortName = $derived(robot.robot_id.replace(/^robot_/, 'R'));
</script>

<Card
  interactive
  {selected}
  accent={color}
  onclick={(e: MouseEvent) => fleet.select(robot.robot_id, e.shiftKey)}
  class="pl-4"
>
  <div class="flex items-start justify-between gap-2">
    <div class="min-w-0">
      <div class="flex items-center gap-2">
        <StatusDot tone={statusTone as never} pulse={robot.nav_status === 'active'} />
        <span class="truncate text-[13px] font-semibold tracking-tight" style="color:{color}">
          {shortName}
        </span>
        {#if robot.robot_type !== 'diffdrive'}
          <span class="truncate text-[10px] uppercase tracking-wide text-fg-dim">
            {robot.robot_type}
          </span>
        {/if}
      </div>
      <div class="mt-1 text-[10px] tabular text-fg-dim">
        {robot.pose.x.toFixed(1)}, {robot.pose.y.toFixed(1)} m
      </div>
    </div>

    <div class="flex shrink-0 items-center gap-1">
      {#if stale}
        <span title="Unattended {Math.round(robot.unattended_s)}s">
          <TriangleAlert class="h-4 w-4 text-warn" />
        </span>
      {/if}
      {#if isCamera}
        <Video class="h-4 w-4 text-accent" />
      {/if}
    </div>
  </div>

  <div class="mt-2.5 flex items-center gap-2">
    <Badge
      tone={robot.mode === 'estop'
        ? 'danger'
        : robot.nav_status === 'active'
          ? 'accent'
          : robot.nav_status === 'failed'
            ? 'warn'
            : 'neutral'}
    >
      {modeLabel}
    </Badge>
    {#if !robot.online}
      <Badge tone="danger">OFFLINE</Badge>
    {/if}
  </div>

  {#if robot.battery !== null}
    <div class="mt-2.5 flex items-center gap-2">
      <Battery class="h-3.5 w-3.5 shrink-0 text-fg-dim" />
      <Meter value={robot.battery} />
      <span class="w-8 shrink-0 text-right text-[11px] tabular text-fg-muted">
        {Math.round(robot.battery * 100)}%
      </span>
    </div>
  {/if}

  {#if selected}
    <div class="mt-3 flex gap-1.5 border-t border-border pt-2.5">
      <button
        class="inline-flex h-8 flex-1 touch-target items-center justify-center gap-1.5 rounded-[4px]
               border border-border bg-surface text-[10px] font-medium text-fg-muted shadow-sm
               transition-colors hover:bg-surface-2 hover:text-fg disabled:opacity-40"
        disabled={!fleet.can(robot.robot_id, 'camera')}
        onclick={(e) => {
          e.stopPropagation();
          fleet.setCamera(robot.robot_id);
          actions.switchCamera(robot.robot_id);
        }}
      >
        <Video class="h-3.5 w-3.5" /> View
      </button>
      <button
        class="inline-flex h-8 flex-1 touch-target items-center justify-center gap-1.5 rounded-[4px]
               border border-border bg-surface text-[10px] font-medium text-fg-muted shadow-sm
               transition-colors hover:bg-surface-2 hover:text-fg disabled:opacity-40"
        disabled={robot.nav_status !== 'active'}
        onclick={(e) => {
          e.stopPropagation();
          actions.cancelGoal(robot.robot_id);
        }}
      >
        <CircleSlash class="h-3.5 w-3.5" /> Cancel
      </button>
    </div>
    {#if fleet.can(robot.robot_id, 'navigate')}
      <div class="mt-1.5 flex items-center gap-1.5 text-[10px] text-fg-dim">
        <Navigation class="h-3 w-3" /> Use Point nav to choose a destination
      </div>
    {/if}
  {/if}
</Card>
