<script lang="ts">
  import { tick, untrack } from 'svelte';
  import { VideoOff, Radio } from 'lucide-svelte';
  import Badge from '../ui/Badge.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { session } from '$lib/stores/session.svelte';
  import { detectionCatalog } from '$lib/stores/detection.svelte';
  import { actions } from '$lib/api/connection';
  import { robotDisplayName } from '$lib/robotDisplayName';

  /**
   * WebRTC camera view.
   * Streams arrive from MediaMTX via WHEP at /whep/<robot_id>. When no stream is
   * available (mock mode, or MediaMTX down) the panel degrades to a placeholder
   * rather than blocking the operator.
   */

  let video = $state<HTMLVideoElement | null>(null);
  let imageA = $state<HTMLImageElement | null>(null);
  let imageB = $state<HTMLImageElement | null>(null);
  let pc: RTCPeerConnection | null = null;
  let fallbackTimer: number | null = null;
  let whepRetryTimer: number | null = null;
  let objectA: string | null = null;
  let objectB: string | null = null;
  let activeFrame = $state<0 | 1>(0);
  let fallbackGeneration = 0;
  let streamSource = $state<'webrtc' | 'jpeg' | null>(null);
  let streamState = $state<'idle' | 'connecting' | 'live' | 'unavailable'>('idle');
  let mediaAspect = $state(16 / 9);

  const activeId = $derived(fleet.activeCamera);
  const robot = $derived(activeId ? fleet.get(activeId) : undefined);
  const color = $derived(activeId ? fleet.colorOf(activeId) : 'var(--color-fg-dim)');
  const boxes = $derived(activeId ? session.bboxesFor(activeId) : []);

  function setMediaAspect(width: number, height: number) {
    if (width > 0 && height > 0) mediaAspect = width / height;
  }

  function updateVideoAspect() {
    if (video) setMediaAspect(video.videoWidth, video.videoHeight);
  }

  async function waitForIceGathering(connection: RTCPeerConnection) {
    if (connection.iceGatheringState === 'complete') return;
    await new Promise<void>((resolve) => {
      const timeout = window.setTimeout(resolve, 2000);
      const changed = () => {
        if (connection.iceGatheringState !== 'complete') return;
        clearTimeout(timeout);
        connection.removeEventListener('icegatheringstatechange', changed);
        resolve();
      };
      connection.addEventListener('icegatheringstatechange', changed);
    });
  }

  async function connectWhep(robotId: string) {
    teardown();
    streamState = 'connecting';
    let connection: RTCPeerConnection | null = null;
    try {
      const candidate = new RTCPeerConnection({ iceServers: [] });
      connection = candidate;
      pc = candidate;
      const transceiver = candidate.addTransceiver('video', { direction: 'recvonly' });
      // Teleoperation values freshness over concealment. The standards-based
      // hint is best-effort: Chrome clamps it to what current network
      // conditions can sustain, while browsers without it keep their default.
      const receiver = transceiver.receiver as RTCRtpReceiver & {
        jitterBufferTarget?: number | null;
      };
      if ('jitterBufferTarget' in receiver) receiver.jitterBufferTarget = 0;
      candidate.ontrack = (e) => {
        if (video) video.srcObject = e.streams[0];
        streamSource = 'webrtc';
        streamState = 'live';
      };
      candidate.onconnectionstatechange = () => {
        if (
          pc === candidate &&
          (candidate.connectionState === 'failed' || candidate.connectionState === 'disconnected')
        ) {
          teardown();
          startJpegFallback(robotId);
        }
      };
      const offer = await candidate.createOffer();
      await candidate.setLocalDescription(offer);
      // This client deliberately does not implement trickle-ICE PATCHes. Wait
      // until host candidates are in the SDP before sending the one-shot WHEP
      // offer; otherwise a fast POST can contain no usable media candidate.
      await waitForIceGathering(candidate);
      if (pc !== candidate) return;

      const res = await fetch(`/whep/${robotId}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/sdp' },
        body: candidate.localDescription?.sdp
      });
      if (!res.ok) throw new Error(`whep ${res.status}`);
      const answer = await res.text();
      if (pc !== candidate) return;
      await candidate.setRemoteDescription({ type: 'answer', sdp: answer });
    } catch {
      if (pc !== connection) return;
      teardown();
      startJpegFallback(robotId);
    }
  }

  function startJpegFallback(robotId: string) {
    streamSource = 'jpeg';
    streamState = 'connecting';
    const generation = ++fallbackGeneration;

    // Fetch, decode, then atomically swap a local object URL. Updating the
    // visible image's HTTP URL directly exposes its empty/loading state and
    // causes a white/black flash between every JPEG frame.
    const refresh = async () => {
      try {
        const response = await fetch(
          `/api/camera/${encodeURIComponent(robotId)}?t=${Date.now()}`,
          { cache: 'no-store' }
        );
        if (!response.ok) throw new Error(`camera ${response.status}`);
        const objectUrl = URL.createObjectURL(await response.blob());
        const nextFrame: 0 | 1 = activeFrame === 0 ? 1 : 0;
        await tick();
        const target = nextFrame === 0 ? imageA : imageB;
        if (!target) throw new Error('camera buffer unavailable');
        const previousInactive = nextFrame === 0 ? objectA : objectB;
        if (previousInactive) URL.revokeObjectURL(previousInactive);
        if (nextFrame === 0) {
          objectA = objectUrl;
        } else {
          objectB = objectUrl;
        }
        await new Promise<void>((resolve, reject) => {
          target.onload = () => {
            setMediaAspect(target.naturalWidth, target.naturalHeight);
            resolve();
          };
          target.onerror = () => reject(new Error('camera frame decode failed'));
          target.src = objectUrl;
        });
        await target.decode();
        if (generation !== fallbackGeneration) {
          URL.revokeObjectURL(objectUrl);
          return;
        }
        activeFrame = nextFrame;
        streamState = 'live';
      } catch {
        if (streamState !== 'live') streamState = 'unavailable';
      } finally {
        if (generation === fallbackGeneration) {
          fallbackTimer = window.setTimeout(refresh, 200);
        }
      }
    };
    void refresh();
    // Keep the JPEG path useful during a media-server restart, but return to
    // low-latency video automatically once WHEP is available again.
    whepRetryTimer = window.setTimeout(() => connectWhep(robotId), 10_000);
  }

  function teardown() {
    fallbackGeneration++;
    const closing = pc;
    pc = null;
    closing?.close();
    if (fallbackTimer) clearTimeout(fallbackTimer);
    fallbackTimer = null;
    if (whepRetryTimer) clearTimeout(whepRetryTimer);
    whepRetryTimer = null;
    imageA?.removeAttribute('src');
    imageB?.removeAttribute('src');
    if (objectA) URL.revokeObjectURL(objectA);
    if (objectB) URL.revokeObjectURL(objectB);
    objectA = null;
    objectB = null;
    activeFrame = 0;
    streamSource = null;
    mediaAspect = 16 / 9;
    if (video) video.srcObject = null;
  }

  $effect(() => {
    const id = activeId;
    // Robot telemetry replaces the state object at 5 Hz. Capability checks
    // must not become an effect dependency or every telemetry packet tears
    // down and recreates the camera stream.
    if (!id || !untrack(() => fleet.can(id, 'camera'))) {
      streamState = 'idle';
      teardown();
      return;
    }
    untrack(() => connectWhep(id));
    return () => untrack(teardown);
  });
</script>

<div
  class="panel-glow flex flex-col overflow-hidden rounded-[--radius-card] border border-border bg-surface"
>
  <div class="flex h-11 items-center justify-between border-b border-border px-3">
    <div class="flex items-center gap-2">
      <Radio class="h-3.5 w-3.5" style="color:{color}" />
      <span class="text-[11px] font-semibold tracking-tight" style="color:{color}">
        {activeId ? robotDisplayName(activeId) : 'No camera'}
      </span>
    </div>
    {#if streamState === 'live'}
      <Badge tone="ok">Live {streamSource === 'jpeg' ? 'preview' : ''}</Badge>
    {:else if streamState === 'connecting'}
      <Badge tone="accent">…</Badge>
    {:else if streamState === 'unavailable'}
      <Badge tone="warn">NO SIGNAL</Badge>
    {/if}
  </div>

  <div
    class="relative m-2 mb-0 shrink-0 overflow-hidden rounded-[4px] bg-black"
    style="aspect-ratio: {mediaAspect}"
  >
    <video
      bind:this={video}
      class="h-full w-full object-contain {streamSource === 'webrtc' ? '' : 'hidden'}"
      autoplay
      muted
      playsinline
      onloadedmetadata={updateVideoAspect}
      onresize={updateVideoAspect}
    ></video>

    {#if streamSource === 'jpeg'}
      <img
        bind:this={imageA}
        class="absolute inset-0 h-full w-full object-contain transition-none {activeFrame === 0
          ? 'z-20'
          : 'z-10'}"
        data-active={activeFrame === 0}
        alt="Live camera for {activeId ? robotDisplayName(activeId) : 'robot'}"
      />
      <img
        bind:this={imageB}
        class="absolute inset-0 h-full w-full object-contain transition-none {activeFrame === 1
          ? 'z-20'
          : 'z-10'}"
        data-active={activeFrame === 1}
        alt="Live camera buffer for {activeId ? robotDisplayName(activeId) : 'robot'}"
      />
    {/if}

    {#if streamState !== 'live'}
      <div class="absolute inset-0 z-40 grid place-items-center bg-surface-2">
        <div class="flex flex-col items-center gap-2 text-fg-dim">
          <div class="grid h-10 w-10 place-items-center rounded-[4px] border border-border bg-surface">
            <VideoOff class="h-5 w-5" />
          </div>
          <span class="text-[10px] font-medium">
            {streamState === 'connecting' ? 'Connecting…' : 'Stream unavailable'}
          </span>
        </div>
      </div>
    {/if}

    <!-- detection overlay, sized to the video element (never an iframe) -->
    <div class="pointer-events-none absolute inset-0 z-30">
      <!-- Outlines first, in one normalized-coordinate SVG that stretches with
           the frame, so a mask always lines up with its own box. -->
      <svg
        class="absolute inset-0 h-full w-full"
        viewBox="0 0 1 1"
        preserveAspectRatio="none"
        aria-hidden="true"
      >
        {#each boxes as d (d.id)}
          {#if d.polygon && d.polygon.length > 2}
            <polygon
              points={d.polygon.map(([x, y]) => `${x},${y}`).join(' ')}
              fill={detectionCatalog.colorOf(d.class)}
              fill-opacity="0.22"
              stroke={detectionCatalog.colorOf(d.class)}
              stroke-opacity="0.9"
              stroke-width="0.004"
              vector-effect="non-scaling-stroke"
            />
          {/if}
        {/each}
      </svg>
      {#each boxes as d (d.id)}
        {#if d.bbox}
          <div
            class="absolute rounded border-2"
            style="left:{d.bbox[0] * 100}%; top:{d.bbox[1] * 100}%;
                   width:{d.bbox[2] * 100}%; height:{d.bbox[3] * 100}%;
                   border-color:{detectionCatalog.colorOf(d.class)}"
          >
            <span
              class="absolute -top-5 left-0 whitespace-nowrap rounded px-1.5 py-0.5
                     text-[10px] font-bold text-black"
              style="background:{detectionCatalog.colorOf(d.class)}"
            >
              {detectionCatalog.labelOf(d.class)}
              {Math.round(d.score * 100)}%
            </span>
          </div>
        {/if}
      {/each}
    </div>
  </div>

  <!-- camera switcher -->
  <div class="flex flex-wrap gap-1.5 p-2">
    {#each fleet.robots.filter((r) => r.capabilities?.includes('camera')) as r (r.robot_id)}
      <button
        class="h-8 touch-target flex-1 rounded-[4px] border px-2 text-[10px] font-semibold
               transition-colors
               {fleet.activeCamera === r.robot_id
          ? 'border-border-strong bg-surface-2 text-fg'
          : 'border-border bg-surface text-fg-muted hover:bg-surface-2'}"
        onclick={() => {
          fleet.focus(r.robot_id);
          actions.selectRobots(fleet.selected);
          actions.switchCamera(r.robot_id);
        }}
      >
        {robotDisplayName(r.robot_id)}
      </button>
    {/each}
    {#if fleet.robots.filter((r) => r.capabilities?.includes('camera')).length === 0}
      <span class="px-1 py-1.5 text-[11px] text-fg-dim">No camera-capable robots</span>
    {/if}
  </div>

  {#if robot}
    <div class="mt-auto border-t border-border px-3 py-2.5 text-[10px] text-fg-dim">
      <div class="flex justify-between">
        <span>Detections</span>
        <span class="tabular text-fg-muted">
          {session.detections.filter((d) => d.robot_id === robot.robot_id).length}
        </span>
      </div>
    </div>
  {/if}
</div>
