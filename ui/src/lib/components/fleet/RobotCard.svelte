<script lang="ts">
  import { ScanSearch, SlidersHorizontal, Video, TriangleAlert, X } from 'lucide-svelte';
  import Card from '../ui/Card.svelte';
  import Badge from '../ui/Badge.svelte';
  import BatteryIndicator from '../ui/BatteryIndicator.svelte';
  import StatusDot from '../ui/StatusDot.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { actions } from '$lib/api/connection';
  import { session } from '$lib/stores/session.svelte';
  import { robotDisplayName } from '$lib/robotDisplayName';
  import type { RobotState } from '$lib/types/protocol';

  let { robot, unattendedThreshold = 45 }: { robot: RobotState; unattendedThreshold?: number } =
    $props();

  let bodyModalOpen = $state(false);

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

    <div class="flex shrink-0 items-center gap-1.5">
      <BatteryIndicator value={robot.battery} />
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
      {#if fleet.can(robot.robot_id, 'body')}
        <button
          class="inline-flex h-5 items-center gap-1 rounded-full bg-surface-3 px-2 text-[9px] font-semibold text-fg-muted transition-colors hover:bg-surface-4 hover:text-fg active:scale-95"
          title="Open {shortName} body actions"
          onclick={(e) => {
            e.stopPropagation();
            bodyModalOpen = true;
          }}
        >
          <SlidersHorizontal class="h-2.5 w-2.5" />
          Actions
        </button>
      {/if}
    </div>
  </div>
</Card>

{#if bodyModalOpen}
  <div
    class="fixed inset-0 z-50 flex items-center justify-center bg-black/45 p-4 backdrop-blur-sm"
    role="presentation"
    onclick={(e) => {
      e.stopPropagation();
      if (e.target === e.currentTarget) bodyModalOpen = false;
    }}
  >
    <div
      class="panel-glow w-full max-w-xs flex flex-col overflow-hidden rounded-[--radius-dialog] border border-border/80 bg-surface shadow-2xl"
      role="dialog"
      aria-modal="true"
      aria-labelledby="body-dialog-title"
      onclick={(e) => e.stopPropagation()}
    >
      <header class="flex h-12 shrink-0 items-center justify-between border-b border-border/70 px-4">
        <div class="flex items-center gap-2">
          <span class="h-2 w-2 rounded-full {robot.online ? 'bg-ok' : 'bg-danger'}"></span>
          <h3 id="body-dialog-title" class="text-xs font-semibold text-fg">
            {shortName} Actions
          </h3>
        </div>
        <button
          class="grid h-8 w-8 place-items-center rounded-full text-fg-muted hover:bg-surface-2 hover:text-fg"
          onclick={() => (bodyModalOpen = false)}
        >
          <X class="h-4 w-4" />
        </button>
      </header>

      <div class="p-4 flex flex-col gap-3">
        {#if robot.robot_type === 'unitree_g1' || robot.robot_id.startsWith('chris')}
          <div class="text-[11px] text-fg-dim">
            Direct humanoid commands backed by Chris's native G1 bridge for <strong class="text-fg">{shortName}</strong>.
          </div>

          <div class="grid grid-cols-2 gap-2 mt-0.5">
            <button
              class="inline-flex h-10 touch-target items-center justify-center rounded-full bg-surface-3 px-3 text-xs font-semibold text-fg-muted transition-[background,transform] hover:bg-border active:scale-[0.98] disabled:opacity-40"
              disabled={!robot.online}
              onclick={() => {
                actions.bodyCommand(robot.robot_id, 'damping');
                bodyModalOpen = false;
              }}
            >
              Damping
            </button>
            <button
              class="inline-flex h-10 touch-target items-center justify-center rounded-full bg-accent-container px-3 text-xs font-semibold text-accent-container-fg transition-[background,transform] hover:brightness-95 active:scale-[0.98] disabled:opacity-40"
              disabled={!robot.online}
              onclick={() => {
                actions.bodyCommand(robot.robot_id, 'lie_to_stand');
                bodyModalOpen = false;
              }}
            >
              Lie &rarr; Stand
            </button>
            <button
              class="inline-flex h-10 touch-target items-center justify-center rounded-full bg-surface-3 px-3 text-xs font-semibold text-fg-muted transition-[background,transform] hover:bg-border active:scale-[0.98] disabled:opacity-40"
              disabled={!robot.online}
              onclick={() => {
                actions.bodyCommand(robot.robot_id, 'sit');
                bodyModalOpen = false;
              }}
            >
              Sit
            </button>
            <button
              class="inline-flex h-10 touch-target items-center justify-center rounded-full bg-accent-container px-3 text-xs font-semibold text-accent-container-fg transition-[background,transform] hover:brightness-95 active:scale-[0.98] disabled:opacity-40"
              disabled={!robot.online}
              onclick={() => {
                actions.bodyCommand(robot.robot_id, 'stand');
                bodyModalOpen = false;
              }}
            >
              Start / Walk
            </button>
          </div>
        {:else}
          <div class="text-[11px] text-fg-dim">
            Direct quadruped posture and control commands for <strong class="text-fg">{shortName}</strong>.
          </div>

          <div class="grid grid-cols-2 gap-2 mt-0.5">
            <button
              class="inline-flex h-10 touch-target items-center justify-center rounded-full bg-accent-container px-3 text-xs font-semibold text-accent-container-fg transition-[background,transform] hover:brightness-95 active:scale-[0.98] disabled:opacity-40"
              disabled={!robot.online}
              onclick={() => {
                actions.bodyCommand(robot.robot_id, 'claim');
                bodyModalOpen = false;
              }}
            >
              Claim
            </button>
            <button
              class="inline-flex h-10 touch-target items-center justify-center rounded-full bg-surface-3 px-3 text-xs font-semibold text-fg-muted transition-[background,transform] hover:bg-border active:scale-[0.98] disabled:opacity-40"
              disabled={!robot.online}
              onclick={() => {
                actions.bodyCommand(robot.robot_id, 'release');
                bodyModalOpen = false;
              }}
            >
              Release
            </button>
            <button
              class="inline-flex h-10 touch-target items-center justify-center rounded-full bg-surface-3 px-3 text-xs font-semibold text-fg-muted transition-[background,transform] hover:bg-border active:scale-[0.98] disabled:opacity-40"
              disabled={!robot.online}
              onclick={() => {
                actions.bodyCommand(robot.robot_id, 'sit');
                bodyModalOpen = false;
              }}
            >
              Sit
            </button>
            <button
              class="inline-flex h-10 touch-target items-center justify-center rounded-full bg-surface-3 px-3 text-xs font-semibold text-fg-muted transition-[background,transform] hover:bg-border active:scale-[0.98] disabled:opacity-40"
              disabled={!robot.online}
              onclick={() => {
                actions.bodyCommand(robot.robot_id, 'stand');
                bodyModalOpen = false;
              }}
            >
              Stand
            </button>
          </div>
        {/if}
      </div>
    </div>
  </div>
{/if}
