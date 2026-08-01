<script lang="ts">
  import { VideoOff, Radio } from 'lucide-svelte';
  import Badge from '../ui/Badge.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { session } from '$lib/stores/session.svelte';

  /**
   * WebRTC camera view.
   * Streams arrive from MediaMTX via WHEP at /whep/<robot_id>. When no stream is
   * available (mock mode, or MediaMTX down) the panel degrades to a placeholder
   * rather than blocking the operator.
   */

  let video = $state<HTMLVideoElement | null>(null);
  let pc: RTCPeerConnection | null = null;
  let streamState = $state<'idle' | 'connecting' | 'live' | 'unavailable'>('idle');

  const activeId = $derived(fleet.activeCamera);
  const robot = $derived(activeId ? fleet.get(activeId) : undefined);
  const color = $derived(activeId ? fleet.colorOf(activeId) : 'var(--color-fg-dim)');
  const boxes = $derived(activeId ? session.bboxesFor(activeId) : []);

  async function connectWhep(robotId: string) {
    teardown();
    streamState = 'connecting';
    try {
      pc = new RTCPeerConnection({ iceServers: [] });
      pc.addTransceiver('video', { direction: 'recvonly' });
      pc.ontrack = (e) => {
        if (video) video.srcObject = e.streams[0];
        streamState = 'live';
      };
      const offer = await pc.createOffer();
      await pc.setLocalDescription(offer);

      const res = await fetch(`/whep/${robotId}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/sdp' },
        body: offer.sdp
      });
      if (!res.ok) throw new Error(`whep ${res.status}`);
      const answer = await res.text();
      await pc.setRemoteDescription({ type: 'answer', sdp: answer });
    } catch {
      streamState = 'unavailable';
      teardown();
    }
  }

  function teardown() {
    pc?.close();
    pc = null;
    if (video) video.srcObject = null;
  }

  $effect(() => {
    const id = activeId;
    if (!id || !fleet.can(id, 'camera')) {
      streamState = 'idle';
      teardown();
      return;
    }
    connectWhep(id);
    return () => teardown();
  });
</script>

<div
  class="flex h-full flex-col overflow-hidden rounded-[--radius-card] border border-border bg-surface"
>
  <div class="flex items-center justify-between border-b border-border px-3 py-2">
    <div class="flex items-center gap-2">
      <Radio class="h-3.5 w-3.5" style="color:{color}" />
      <span class="text-xs font-semibold" style="color:{color}">
        {activeId ? activeId.replace(/^robot_/, 'R') : 'No camera'}
      </span>
    </div>
    {#if streamState === 'live'}
      <Badge tone="ok">LIVE</Badge>
    {:else if streamState === 'connecting'}
      <Badge tone="accent">…</Badge>
    {:else if streamState === 'unavailable'}
      <Badge tone="warn">NO SIGNAL</Badge>
    {/if}
  </div>

  <div class="relative aspect-video w-full shrink-0 bg-black">
    <video
      bind:this={video}
      class="h-full w-full object-cover"
      autoplay
      muted
      playsinline
    ></video>

    {#if streamState !== 'live'}
      <div class="absolute inset-0 grid place-items-center bg-surface-2">
        <div class="flex flex-col items-center gap-2 text-fg-dim">
          <VideoOff class="h-7 w-7" />
          <span class="text-[11px]">
            {streamState === 'connecting' ? 'Connecting…' : 'Stream unavailable'}
          </span>
        </div>
      </div>
    {/if}

    <!-- detection overlay, sized to the video element (never an iframe) -->
    <div class="pointer-events-none absolute inset-0">
      {#each boxes as d (d.id)}
        {#if d.bbox}
          <div
            class="absolute rounded border-2 border-warn"
            style="left:{d.bbox[0] * 100}%; top:{d.bbox[1] * 100}%;
                   width:{d.bbox[2] * 100}%; height:{d.bbox[3] * 100}%"
          >
            <span
              class="absolute -top-5 left-0 whitespace-nowrap rounded bg-warn px-1.5 py-0.5
                     text-[10px] font-bold text-black"
            >
              {d.class}
              {Math.round(d.score * 100)}%
            </span>
          </div>
        {/if}
      {/each}
    </div>
  </div>

  <!-- camera switcher -->
  <div class="flex flex-wrap gap-1.5 border-t border-border p-2">
    {#each fleet.robots.filter((r) => r.capabilities?.includes('camera')) as r (r.robot_id)}
      <button
        class="h-9 touch-target flex-1 rounded-lg border px-2 text-[11px] font-semibold
               transition-colors
               {fleet.activeCamera === r.robot_id
          ? 'border-transparent text-bg'
          : 'border-border bg-surface-2 text-fg-muted hover:bg-surface-3'}"
        style={fleet.activeCamera === r.robot_id
          ? `background:${fleet.colorOf(r.robot_id)}`
          : ''}
        onclick={() => fleet.setCamera(r.robot_id)}
      >
        {r.robot_id.replace(/^robot_/, 'R')}
      </button>
    {/each}
    {#if fleet.robots.filter((r) => r.capabilities?.includes('camera')).length === 0}
      <span class="px-1 py-1.5 text-[11px] text-fg-dim">No camera-capable robots</span>
    {/if}
  </div>

  {#if robot}
    <div class="border-t border-border px-3 py-2 text-[11px] text-fg-dim">
      <div class="flex justify-between">
        <span>Detections</span>
        <span class="tabular text-fg-muted">
          {session.detections.filter((d) => d.robot_id === robot.robot_id).length}
        </span>
      </div>
    </div>
  {/if}
</div>
