<script lang="ts">
  import { Battery, ScanSearch, Video, TriangleAlert } from 'lucide-svelte';
  import Card from '../ui/Card.svelte';
  import Badge from '../ui/Badge.svelte';
  import Meter from '../ui/Meter.svelte';
  import StatusDot from '../ui/StatusDot.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { actions } from '$lib/api/connection';
  import { session } from '$lib/stores/session.svelte';
  import { robotDisplayName } from '$lib/robotDisplayName';
  import type { RobotState } from '$lib/types/protocol';

  let { robot, unattendedThreshold = 45 }: { robot: RobotState; unattendedThreshold?: number } =
    $props();

  const color = $derived(fleet.colorOf(robot.robot_id));
  const selected = $derived(fleet.isSelected(robot.robot_id));
  const isCamera = $derived(fleet.activeCamera === robot.robot_id);
  const stale = $derived(robot.unattended_s > unattendedThreshold);
  const detectionCount = $derived(
    session.detections.filter((detection) => detection.robot_id === robot.robot_id).length
  );

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

  // `recover` outranks the nav_status it follows: the goal has already failed,
  // and what the operator needs to see is that the robot is moving right now.
  const modeLabel = $derived(
    robot.mode === 'estop'
      ? 'E-STOP'
      : robot.mode === 'recover'
        ? 'BACKING OFF'
        : robot.nav_status === 'active'
          ? 'NAVIGATING'
          : robot.nav_status === 'failed'
            ? 'NAV FAILED'
            : robot.mode.toUpperCase()
  );

  const shortName = $derived(robotDisplayName(robot.robot_id));

  function chooseRobot(event: MouseEvent | KeyboardEvent) {
    if (event.shiftKey) fleet.select(robot.robot_id, true);
    else fleet.focus(robot.robot_id);
    actions.selectRobots(fleet.selected);
    if (fleet.can(robot.robot_id, 'camera')) actions.switchCamera(robot.robot_id);
  }
</script>

<Card
  interactive
  {selected}
  accent={color}
  onclick={chooseRobot}
  class="pl-3.5 pr-3 py-2.5"
>
  <div class="flex items-start justify-between gap-2">
    <div class="min-w-0">
      <div class="flex items-center gap-1.5">
        <StatusDot tone={statusTone as never} pulse={robot.nav_status === 'active'} />
        <span class="truncate text-[12.5px] font-semibold tracking-tight" style="color:{color}">
          {shortName}
        </span>
        <span
          class="inline-flex h-4.5 shrink-0 items-center gap-1 rounded-full bg-surface-3 px-1.5
                 text-[8.5px] font-semibold tabular text-fg-muted"
          title="{detectionCount} detection{detectionCount === 1 ? '' : 's'} from {shortName}"
          aria-label="{detectionCount} detection{detectionCount === 1 ? '' : 's'}"
        >
          <ScanSearch class="h-2.5 w-2.5" />
          {detectionCount}
        </span>
        {#if robot.robot_type !== 'diffdrive'}
          <span class="truncate text-[9.5px] font-medium tracking-wide text-fg-dim">
            {robot.robot_type}
          </span>
        {/if}
      </div>
      <div class="mt-0.5 text-[9.5px] tabular text-fg-dim">
        {robot.pose.x.toFixed(1)}, {robot.pose.y.toFixed(1)} m
      </div>
    </div>

    <div class="flex shrink-0 items-center gap-1">
      {#if stale}
        <span title="Unattended {Math.round(robot.unattended_s)}s">
          <TriangleAlert class="h-3.5 w-3.5 text-warn" />
        </span>
      {/if}
      {#if isCamera}
        <Video class="h-3.5 w-3.5 text-accent" />
      {/if}
    </div>
  </div>

  <div class="mt-1.5 flex items-center justify-between gap-2">
    <div class="flex items-center gap-1.5">
      <Badge
        tone={robot.mode === 'estop'
          ? 'danger'
          : robot.mode === 'recover'
            ? 'warn'
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
      <div class="flex items-center gap-1.5">
        <Battery class="h-3 w-3 shrink-0 text-fg-dim" />
        <Meter value={robot.battery} class="w-12" />
        <span class="text-[10px] tabular font-medium text-fg-muted">
          {Math.round(robot.battery * 100)}%
        </span>
      </div>
    {/if}
  </div>

  {#if selected && fleet.can(robot.robot_id, 'body')}
    <div
      class="mt-2 border-t border-border/60 pt-2"
      role="group"
      aria-label="Body control"
      onclick={(e) => e.stopPropagation()}
      onkeydown={(e) => e.stopPropagation()}
    >
      <div class="mb-1.5 flex items-center justify-between text-[9px] font-semibold uppercase tracking-wider text-fg-dim">
        <span>Body actions</span>
        <span class="h-1.5 w-1.5 rounded-full {robot.online ? 'bg-ok' : 'bg-danger'}"></span>
      </div>
      <div class="grid grid-cols-4 gap-1">
        <button
          class="inline-flex h-7 items-center justify-center rounded-md bg-accent-container px-1 text-[10px] font-semibold text-accent-container-fg hover:brightness-95 active:scale-95 disabled:opacity-40"
          disabled={!robot.online}
          onclick={() => actions.bodyCommand(robot.robot_id, 'claim')}
        >
          Claim
        </button>
        <button
          class="inline-flex h-7 items-center justify-center rounded-md bg-surface-3 px-1 text-[10px] font-semibold text-fg-muted hover:bg-border active:scale-95 disabled:opacity-40"
          disabled={!robot.online}
          onclick={() => actions.bodyCommand(robot.robot_id, 'release')}
        >
          Release
        </button>
        <button
          class="inline-flex h-7 items-center justify-center rounded-md bg-surface-3 px-1 text-[10px] font-semibold text-fg-muted hover:bg-border active:scale-95 disabled:opacity-40"
          disabled={!robot.online}
          onclick={() => actions.bodyCommand(robot.robot_id, 'sit')}
        >
          Sit
        </button>
        <button
          class="inline-flex h-7 items-center justify-center rounded-md bg-surface-3 px-1 text-[10px] font-semibold text-fg-muted hover:bg-border active:scale-95 disabled:opacity-40"
          disabled={!robot.online}
          onclick={() => actions.bodyCommand(robot.robot_id, 'stand')}
        >
          Stand
        </button>
      </div>
    </div>
  {/if}
</Card>
