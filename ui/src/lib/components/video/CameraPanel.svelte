<script lang="ts">
  import { untrack } from 'svelte';
  import { Maximize2, Minimize2, PanelRightClose, Radio, VideoOff } from 'lucide-svelte';
  import Badge from '../ui/Badge.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { session } from '$lib/stores/session.svelte';
  import { detectionCatalog } from '$lib/stores/detection.svelte';
  import { actions } from '$lib/api/connection';
  import { robotDisplayName } from '$lib/robotDisplayName';

  let {
    expanded = false,
    ontoggleexpand = () => {},
    oncollapse = () => {}
  }: {
    expanded?: boolean;
    ontoggleexpand?: () => void;
    oncollapse?: () => void;
  } = $props();

  /**
   * H.264 camera view.
   * Streams arrive from MediaMTX via WHEP at /whep/<robot_id>. There is no JPEG
   * fallback: when H.264 is unavailable the panel reports NO SIGNAL.
  */

  let video = $state<HTMLVideoElement | null>(null);
  let pc: RTCPeerConnection | null = null;
  let whepRetryTimer: number | null = null;
  let whepProbeTimer: number | null = null;
  let whepFailures = 0;
  let streamSource = $state<'webrtc' | null>(null);
  let streamState = $state<'idle' | 'connecting' | 'live' | 'unavailable'>('idle');
  let mediaAspect = $state(4 / 3);
  let fps = $state(0);
  let pingLatencyMs = $state(0);

  const ROLLING_ALPHA = 0.2;
  let lastFrameTime = 0;
  let rfcHandle: number | null = null;
  let statsTimer: number | null = null;

  function recordFrame(now: number) {
    if (lastFrameTime > 0) {
      const dt = (now - lastFrameTime) / 1000;
      if (dt >= 0.025 && dt < 1.5) {
        const instantFps = Math.min(30, 1 / dt);
        fps = fps === 0 ? instantFps : fps * (1 - ROLLING_ALPHA) + instantFps * ROLLING_ALPHA;
      }
    }
    lastFrameTime = now;
  }

  function recordLatency(instantMs: number) {
    if (instantMs >= 0 && instantMs < 10000) {
      pingLatencyMs =
        pingLatencyMs === 0
          ? instantMs
          : pingLatencyMs * (1 - ROLLING_ALPHA) + instantMs * ROLLING_ALPHA;
    }
  }

  function startVideoFrameLoop() {
    stopVideoFrameLoop();
    if (!video) return;
    if ('requestVideoFrameCallback' in HTMLVideoElement.prototype) {
      const onFrame = (now: DOMHighResTimeStamp) => {
        recordFrame(now);
        if (streamSource === 'webrtc' && video) {
          rfcHandle = (video as any).requestVideoFrameCallback(onFrame);
        }
      };
      rfcHandle = (video as any).requestVideoFrameCallback(onFrame);
    }
  }

  function stopVideoFrameLoop() {
    if (rfcHandle !== null && video && 'cancelVideoFrameCallback' in HTMLVideoElement.prototype) {
      (video as any).cancelVideoFrameCallback(rfcHandle);
    }
    rfcHandle = null;
    lastFrameTime = 0;
  }

  function startStatsPolling() {
    stopStatsPolling();
    statsTimer = window.setInterval(async () => {
      if (!pc || streamSource !== 'webrtc') return;
      if (lastFrameTime > 0 && performance.now() - lastFrameTime > 1200) {
        fps = 0;
      }
      try {
        const stats = await pc.getStats();
        let foundRtt = false;
        stats.forEach((report) => {
          if (
            report.type === 'candidate-pair' &&
            (report.state === 'succeeded' || report.nominated)
          ) {
            const rtt =
              report.currentRoundTripTime ??
              (report.responsesReceived > 0
                ? report.totalRoundTripTime / report.responsesReceived
                : undefined);
            if (typeof rtt === 'number' && Number.isFinite(rtt)) {
              recordLatency(rtt * 1000);
              foundRtt = true;
            }
          }
        });
        if (!foundRtt) {
          stats.forEach((report) => {
            if (report.type === 'inbound-rtp') {
              if (typeof report.roundTripTime === 'number' && Number.isFinite(report.roundTripTime)) {
                recordLatency(report.roundTripTime * 1000);
              }
              if (rfcHandle === null && typeof report.framesPerSecond === 'number' && report.framesPerSecond > 0) {
                const instantFps = Math.min(30, report.framesPerSecond);
                fps = fps === 0 ? instantFps : fps * (1 - ROLLING_ALPHA) + instantFps * ROLLING_ALPHA;
              }
            }
          });
        }
      } catch {
        // Ignored
      }
    }, 1000);
  }

  function stopStatsPolling() {
    if (statsTimer) {
      clearInterval(statsTimer);
      statsTimer = null;
    }
  }

  // Back off between WHEP attempts. A robot may be booting its H.264 publisher
  // or temporarily disconnected, so retry without generating any JPEG traffic.
  const WHEP_RETRY_MIN_MS = 10_000;
  const WHEP_RETRY_MAX_MS = 160_000;
  // A negotiation that neither connects nor fails is the ordinary outcome when
  // ICE cannot reach the media server -- the state machine simply sits in
  // `connecting`. Nothing but a deadline ends it.
  const WHEP_PROBE_TIMEOUT_MS = 12_000;
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

  /** Negotiate the robot's H.264 WHEP stream. */
  async function connectWhep(robotId: string, background = false) {
    closePeer();
    if (!background) {
      streamSource = null;
      streamState = 'connecting';
    }
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
        // Attach the track but do not promote the panel yet: the video element
        // stays hidden until the connection reports itself connected, so a
        // probe that negotiates and then dies never blanks a working preview.
        if (pc === candidate && video) video.srcObject = e.streams[0];
      };
      candidate.onconnectionstatechange = () => {
        if (pc !== candidate) return;
        if (candidate.connectionState === 'connected') {
          // Media is flowing; only now is it safe to show the video element.
          if (whepProbeTimer) clearTimeout(whepProbeTimer);
          whepProbeTimer = null;
          whepFailures = 0;
          streamSource = 'webrtc';
          streamState = 'live';
          startVideoFrameLoop();
          startStatsPolling();
        } else if (
          candidate.connectionState === 'failed' ||
          candidate.connectionState === 'disconnected'
        ) {
          closePeer();
          handleWhepFailure(robotId);
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

      // Answered is not connected. Without a deadline, a negotiation that
      // stalls in `connecting` -- the usual shape of a blocked UDP path, which
      // is what an operator behind an HTTP tunnel always has -- would neither
      // promote to WebRTC nor ever schedule another attempt.
      //
      // Both this arming and the callback re-check `connectionState`, because a
      // fast local connection reaches `connected` before this line runs: the
      // handler above would then clear a timer that does not exist yet, and an
      // unguarded deadline would later tear down a working stream.
      if (pc !== candidate || candidate.connectionState === 'connected') return;
      whepProbeTimer = window.setTimeout(() => {
        whepProbeTimer = null;
        if (pc !== candidate || candidate.connectionState === 'connected') return;
        closePeer();
        handleWhepFailure(robotId);
      }, WHEP_PROBE_TIMEOUT_MS);
    } catch {
      if (pc !== connection) return;
      closePeer();
      handleWhepFailure(robotId);
    }
  }

  /** Report the H.264 outage and keep probing with a widening gap. */
  function handleWhepFailure(robotId: string) {
    whepFailures += 1;
    streamSource = null;
    streamState = 'unavailable';
    if (whepRetryTimer) clearTimeout(whepRetryTimer);
    whepRetryTimer = window.setTimeout(
      () => {
        whepRetryTimer = null;
        void connectWhep(robotId, true);
      },
      Math.min(WHEP_RETRY_MAX_MS, WHEP_RETRY_MIN_MS * 2 ** Math.min(whepFailures - 1, 4))
    );
  }

  /** Close the H.264 peer connection. */
  function closePeer() {
    stopVideoFrameLoop();
    stopStatsPolling();
    fps = 0;
    pingLatencyMs = 0;
    if (whepProbeTimer) clearTimeout(whepProbeTimer);
    whepProbeTimer = null;
    const closing = pc;
    pc = null;
    closing?.close();
    if (video) video.srcObject = null;
  }

  function teardown() {
    closePeer();
    stopVideoFrameLoop();
    stopStatsPolling();
    fps = 0;
    pingLatencyMs = 0;
    if (whepRetryTimer) clearTimeout(whepRetryTimer);
    whepRetryTimer = null;
    whepFailures = 0;
    streamSource = null;
    mediaAspect = 4 / 3;
  }

  $effect(() => {
    const id = activeId;
    // Robot telemetry replaces the state object at 5 Hz. Capability checks
    // must not become an effect dependency or every telemetry packet tears
    // down and recreates the camera stream.
    if (!id || !untrack(() => fleet.can(id, 'camera'))) {
      streamState = 'idle';
      teardown();
      // Releases the previous robot so it drops back to its idle rate. Without
      // this an operator who closes the panel leaves it uploading full-rate.
      untrack(() => actions.switchCamera(''));
      return;
    }
    // Keep the backend's selected-camera state in sync while negotiating the
    // H.264 stream. No camera frame travels over the adapter websocket.
    untrack(() => actions.switchCamera(id));
    untrack(() => connectWhep(id));
    return () => untrack(teardown);
  });
</script>

<div
  class="panel-glow flex flex-col overflow-hidden rounded-[--radius-panel] border border-transparent
         bg-surface {expanded ? 'h-full min-h-0' : 'shrink-0'}"
>
  <div class="flex h-14 shrink-0 items-center justify-between border-b border-border/70 px-4">
    <div class="flex min-w-0 items-center gap-2.5">
      <Radio class="h-3.5 w-3.5" style="color:{color}" />
      <span class="truncate text-xs font-semibold" style="color:{color}">
        {activeId ? robotDisplayName(activeId) : 'No camera'}
      </span>
    </div>
    <div class="flex items-center gap-1">
      {#if streamState === 'live'}
        <Badge tone="ok">Live H.264</Badge>
      {:else if streamState === 'connecting'}
        <Badge tone="accent">Connecting</Badge>
      {:else if streamState === 'unavailable'}
        <Badge tone="warn">No signal</Badge>
      {/if}
      <button
        class="grid h-10 w-10 touch-target place-items-center rounded-full
               text-fg-muted transition-colors hover:bg-surface-2 hover:text-fg"
        title={expanded ? 'Restore video' : 'Expand video'}
        aria-label={expanded ? 'Restore video' : 'Expand video'}
        onclick={ontoggleexpand}
      >
        {#if expanded}
          <Minimize2 class="h-4 w-4" />
        {:else}
          <Maximize2 class="h-4 w-4" />
        {/if}
      </button>
      <button
        class="grid h-10 w-10 touch-target place-items-center rounded-full
               text-fg-muted transition-colors hover:bg-surface-2 hover:text-fg"
        title="Collapse Robot Control panel"
        aria-label="Collapse Robot Control panel"
        onclick={oncollapse}
      >
        <PanelRightClose class="h-4 w-4" />
      </button>
    </div>
  </div>

  <div
    class="relative m-3 mb-0 overflow-hidden rounded-[--radius-control] bg-black
           {expanded ? 'min-h-0 flex-1' : 'shrink-0'}"
    style={expanded ? undefined : `aspect-ratio: ${mediaAspect}`}
  >
    <video
      bind:this={video}
      class="h-full w-full object-contain {streamSource === 'webrtc' ? '' : 'hidden'}"
      autoplay
      muted
      playsinline
      onplay={startVideoFrameLoop}
      onloadedmetadata={updateVideoAspect}
      onresize={updateVideoAspect}
    ></video>

    {#if streamState !== 'live'}
      <div class="absolute inset-0 z-40 grid place-items-center bg-surface-2">
        <div class="flex flex-col items-center gap-2 text-fg-dim">
          <div class="grid h-10 w-10 place-items-center rounded-[--radius-control] border border-border bg-surface">
            <VideoOff class="h-5 w-5" />
          </div>
          <span class="text-xs font-medium">
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
              {Math.round(d.score * 100)}% conf
            </span>
          </div>
        {/if}
      {/each}
    </div>

    <!-- Live diagnostics overlay: FPS & rolling average ping latency -->
    {#if streamState === 'live'}
      <div
        class="pointer-events-none absolute bottom-1.5 right-1.5 z-30 flex items-center gap-1.5 rounded-[--radius-control]
               border border-white/10 bg-black/65 px-1.5 py-0.5 font-mono text-[10px] font-medium
               tabular-nums text-white/90 shadow-sm backdrop-blur-md"
      >
        <span class="flex items-center gap-1">
          <span class="inline-block h-1.5 w-1.5 rounded-full bg-emerald-400"></span>
          <span>{fps > 0 ? Math.round(fps) : '--'} FPS</span>
        </span>
        <span class="text-white/30">·</span>
        <span class="text-white/80">{pingLatencyMs > 0 ? `${Math.round(pingLatencyMs)} ms` : '-- ms'}</span>
      </div>
    {/if}
  </div>

  <!-- camera switcher -->
  <div class="flex flex-wrap gap-2 p-3">
    {#each fleet.robots.filter((r) => r.capabilities?.includes('camera')) as r (r.robot_id)}
      <button
        class="h-10 touch-target flex-1 rounded-full border px-4 text-[11px] font-semibold
               transition-colors
               {fleet.activeCamera === r.robot_id
          ? 'border-transparent bg-accent-container text-accent-container-fg'
          : 'border-transparent bg-surface-3 text-fg-muted hover:bg-border'}"
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
    <div class="mt-auto border-t border-border/70 px-4 py-3 text-[10px] text-fg-dim">
      <div class="flex justify-between">
        <span>Detections</span>
        <span class="tabular text-fg-muted">
          {session.detections.filter((d) => d.robot_id === robot.robot_id).length}
        </span>
      </div>
    </div>
  {/if}
</div>
