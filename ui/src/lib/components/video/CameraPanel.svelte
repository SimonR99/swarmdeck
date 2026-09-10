<script lang="ts">
  import { untrack } from 'svelte';
  import type Hls from 'hls.js';
  import { Maximize2, Minimize2, Radio, VideoOff } from 'lucide-svelte';
  import Badge from '../ui/Badge.svelte';
  import { fleet } from '$lib/stores/fleet.svelte';
  import { session } from '$lib/stores/session.svelte';
  import { detectionCatalog } from '$lib/stores/detection.svelte';
  import { actions } from '$lib/api/connection';
  import { robotDisplayName } from '$lib/robotDisplayName';
  import {
    CameraStreamGate,
    cameraRetryDelayMs,
    decodedFrameIsStalled,
    hlsCameraUrl,
    type CameraStreamAttempt
  } from '$lib/video/cameraStream';

  let {
    expanded = false,
    ontoggleexpand = () => {}
  }: {
    expanded?: boolean;
    ontoggleexpand?: () => void;
  } = $props();

  /**
   * H.264 camera view.
   * Streams prefer MediaMTX WHEP at /whep/<robot_id>. If ICE is unreachable,
   * the same H.264 stream falls back to same-origin HLS at /hls/<robot_id>/.
  */

  let video = $state<HTMLVideoElement | null>(null);
  let containerEl = $state<HTMLDivElement | null>(null);
  let overlayEl = $state<HTMLDivElement | null>(null);

  let pc: RTCPeerConnection | null = null;
  let whepAbort: AbortController | null = null;
  let hls: Hls | null = null;
  let nativeHlsCleanup: (() => void) | null = null;
  let streamRetryTimer: number | null = null;
  let transportProbeTimer: number | null = null;
  let streamFailures = 0;
  const streamGate = new CameraStreamGate();
  let streamSource = $state<'webrtc' | 'hls' | null>(null);
  let streamState = $state<'idle' | 'connecting' | 'live' | 'unavailable'>('idle');
  let fps = $state(0);
  let pingLatencyMs = $state(0);

  function updateOverlayPosition() {
    if (!containerEl || !overlayEl) return;
    const cw = containerEl.clientWidth;
    const ch = containerEl.clientHeight;
    const vw = video?.videoWidth ?? 0;
    const vh = video?.videoHeight ?? 0;

    if (cw <= 0 || ch <= 0 || vw <= 0 || vh <= 0) {
      overlayEl.style.left = '0px';
      overlayEl.style.top = '0px';
      overlayEl.style.width = '100%';
      overlayEl.style.height = '100%';
      return;
    }

    const containerAspect = cw / ch;
    const videoAspect = vw / vh;
    let width = cw;
    let height = ch;
    let left = 0;
    let top = 0;

    if (containerAspect > videoAspect) {
      width = ch * videoAspect;
      left = (cw - width) / 2;
    } else {
      height = cw / videoAspect;
      top = (ch - height) / 2;
    }

    overlayEl.style.left = `${left}px`;
    overlayEl.style.top = `${top}px`;
    overlayEl.style.width = `${width}px`;
    overlayEl.style.height = `${height}px`;
  }

  const ROLLING_ALPHA = 0.2;
  let lastFrameTime = 0;
  let rawFps = 0;
  let rawLatency = 0;
  let rfcHandle: number | null = null;
  let frameEventCleanup: (() => void) | null = null;
  let statsTimer: number | null = null;

  function recordFrame(now: number) {
    if (lastFrameTime > 0) {
      const dt = (now - lastFrameTime) / 1000;
      if (dt >= 0.025 && dt < 1.5) {
        const instantFps = Math.min(30, 1 / dt);
        rawFps = rawFps === 0 ? instantFps : rawFps * (1 - ROLLING_ALPHA) + instantFps * ROLLING_ALPHA;
      }
    }
    lastFrameTime = now;
  }

  function recordLatency(instantMs: number) {
    if (instantMs >= 0 && instantMs < 10000) {
      rawLatency =
        rawLatency === 0
          ? instantMs
          : rawLatency * (1 - ROLLING_ALPHA) + instantMs * ROLLING_ALPHA;
    }
  }

  function startVideoFrameLoop(attempt: CameraStreamAttempt) {
    stopVideoFrameLoop();
    updateOverlayPosition();
    const target = video;
    if (!target) return;
    const onDecodedFrame = (now: number) => {
      if (!streamGate.isCurrent(attempt, activeId) || video !== target) return;
      recordFrame(now);
      updateOverlayPosition();
      if (streamState !== 'live') {
        if (transportProbeTimer) clearTimeout(transportProbeTimer);
        transportProbeTimer = null;
        streamFailures = 0;
        streamState = 'live';
        startStatsPolling(attempt);
      }
    };
    if ('requestVideoFrameCallback' in HTMLVideoElement.prototype) {
      const onFrame = (now: DOMHighResTimeStamp) => {
        onDecodedFrame(now);
        if (!streamGate.isCurrent(attempt, activeId) || video !== target) return;
        rfcHandle = (target as any).requestVideoFrameCallback(onFrame);
      };
      rfcHandle = (target as any).requestVideoFrameCallback(onFrame);
      return;
    }
    const onFrame = () => {
      if (target.readyState < HTMLMediaElement.HAVE_CURRENT_DATA || target.videoWidth <= 0) return;
      onDecodedFrame(performance.now());
    };
    target.addEventListener('loadeddata', onFrame);
    target.addEventListener('timeupdate', onFrame);
    frameEventCleanup = () => {
      target.removeEventListener('loadeddata', onFrame);
      target.removeEventListener('timeupdate', onFrame);
    };
  }

  function stopVideoFrameLoop() {
    if (rfcHandle !== null && video && 'cancelVideoFrameCallback' in HTMLVideoElement.prototype) {
      (video as any).cancelVideoFrameCallback(rfcHandle);
    }
    rfcHandle = null;
    frameEventCleanup?.();
    frameEventCleanup = null;
    lastFrameTime = 0;
    rawFps = 0;
  }

  function startStatsPolling(attempt: CameraStreamAttempt) {
    stopStatsPolling();
    statsTimer = window.setInterval(async () => {
      if (!streamGate.isCurrent(attempt, activeId)) return;
      const now = performance.now();
      const frameAge = lastFrameTime > 0 ? now - lastFrameTime : 0;
      if (frameAge > 1200) {
        fps = 0;
      } else {
        fps = rawFps;
      }
      if (decodedFrameIsStalled(lastFrameTime, now, FRAME_RECONNECT_TIMEOUT_MS)) {
        handleStreamFailure(attempt.robotId, attempt);
        return;
      }
      if (
        streamState === 'live' &&
        decodedFrameIsStalled(lastFrameTime, now, FRAME_STALL_TIMEOUT_MS)
      ) {
        // Hide a stale image immediately while leaving the current decoder a
        // longer window to recover its buffer before rebuilding the transport.
        streamState = 'connecting';
      }
      if (!pc || streamSource !== 'webrtc') return;
      try {
        const stats = await pc.getStats();
        if (!streamGate.isCurrent(attempt, activeId) || streamSource !== 'webrtc') return;
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
                rawFps = rawFps === 0 ? instantFps : rawFps * (1 - ROLLING_ALPHA) + instantFps * ROLLING_ALPHA;
              }
            }
          });
        }
        pingLatencyMs = rawLatency;
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

  // A negotiation that neither connects nor fails is the ordinary outcome when
  // ICE cannot reach the media server -- the state machine simply sits in
  // `connecting`. Nothing but a deadline ends it.
  const WHEP_PROBE_TIMEOUT_MS = 12_000;
  const HLS_PROBE_TIMEOUT_MS = 15_000;
  const FRAME_STALL_TIMEOUT_MS = 3_000;
  const FRAME_RECONNECT_TIMEOUT_MS = 10_000;
  const activeId = $derived(fleet.activeCamera);
  const color = $derived(activeId ? fleet.colorOf(activeId) : 'var(--color-fg-dim)');
  const boxes = $derived(activeId ? session.bboxesFor(activeId) : []);

  async function waitForIceGathering(connection: RTCPeerConnection) {
    if (connection.iceGatheringState === 'complete') return;
    await new Promise<void>((resolve) => {
      let timeout: number;
      const finish = () => {
        clearTimeout(timeout);
        connection.removeEventListener('icegatheringstatechange', changed);
        resolve();
      };
      const changed = () => {
        if (connection.iceGatheringState !== 'complete') return;
        finish();
      };
      timeout = window.setTimeout(finish, 2000);
      connection.addEventListener('icegatheringstatechange', changed);
    });
  }

  function beginAttempt(robotId: string, transport: 'webrtc' | 'hls') {
    streamGate.invalidate();
    closeTransport();
    const attempt = streamGate.begin(robotId, transport);
    streamSource = transport;
    streamState = 'connecting';
    fps = 0;
    pingLatencyMs = 0;
    return attempt;
  }

  function armTransportDeadline(
    attempt: CameraStreamAttempt,
    timeoutMs: number,
    onTimeout: () => void
  ) {
    if (transportProbeTimer) clearTimeout(transportProbeTimer);
    transportProbeTimer = window.setTimeout(() => {
      transportProbeTimer = null;
      if (!streamGate.isCurrent(attempt, activeId) || streamState === 'live') return;
      onTimeout();
    }, timeoutMs);
  }

  /** Negotiate the robot's preferred H.264 WHEP stream. */
  async function connectWhep(robotId: string) {
    const attempt = beginAttempt(robotId, 'webrtc');
    armTransportDeadline(attempt, WHEP_PROBE_TIMEOUT_MS, () => {
      void connectHls(robotId, attempt);
    });
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
        if (!streamGate.isCurrent(attempt, activeId) || pc !== candidate || !video) return;
        video.srcObject = e.streams[0];
        startVideoFrameLoop(attempt);
        void video.play().catch(() => {});
      };
      candidate.onconnectionstatechange = () => {
        if (!streamGate.isCurrent(attempt, activeId) || pc !== candidate) return;
        if (
          candidate.connectionState === 'failed' ||
          candidate.connectionState === 'disconnected'
        ) {
          void connectHls(robotId, attempt);
        }
      };
      const offer = await candidate.createOffer();
      if (!streamGate.isCurrent(attempt, activeId) || pc !== candidate) return;
      await candidate.setLocalDescription(offer);
      if (!streamGate.isCurrent(attempt, activeId) || pc !== candidate) return;
      // This client deliberately does not implement trickle-ICE PATCHes. Wait
      // until host candidates are in the SDP before sending the one-shot WHEP
      // offer; otherwise a fast POST can contain no usable media candidate.
      await waitForIceGathering(candidate);
      if (!streamGate.isCurrent(attempt, activeId) || pc !== candidate) return;

      const abort = new AbortController();
      whepAbort = abort;
      const res = await fetch(`/whep/${robotId}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/sdp' },
        body: candidate.localDescription?.sdp,
        signal: abort.signal
      });
      if (!streamGate.isCurrent(attempt, activeId) || pc !== candidate) return;
      if (!res.ok) throw new Error(`whep ${res.status}`);
      const answer = await res.text();
      if (!streamGate.isCurrent(attempt, activeId) || pc !== candidate) return;
      await candidate.setRemoteDescription({ type: 'answer', sdp: answer });
      if (!streamGate.isCurrent(attempt, activeId) || pc !== candidate) return;
    } catch {
      if (!streamGate.isCurrent(attempt, activeId) || pc !== connection) return;
      void connectHls(robotId, attempt);
    }
  }

  async function connectHls(robotId: string, failedWhep: CameraStreamAttempt) {
    if (!streamGate.isCurrent(failedWhep, activeId)) return;
    const attempt = beginAttempt(robotId, 'hls');
    const target = video;
    if (!target) {
      handleStreamFailure(robotId, attempt);
      return;
    }
    const url = hlsCameraUrl(robotId);
    startVideoFrameLoop(attempt);
    armTransportDeadline(attempt, HLS_PROBE_TIMEOUT_MS, () => {
      handleStreamFailure(robotId, attempt);
    });

    const connectNativeHls = () => {
      const failed = () => handleStreamFailure(robotId, attempt);
      target.addEventListener('error', failed);
      nativeHlsCleanup = () => {
        target.removeEventListener('error', failed);
      };
      target.src = url;
      target.load();
      void target.play().catch(() => {});
    };

    try {
      const { default: HlsPlayer } = await import('hls.js');
      if (!streamGate.isCurrent(attempt, activeId)) return;
      if (!HlsPlayer.isSupported()) {
        if (target.canPlayType('application/vnd.apple.mpegurl')) {
          connectNativeHls();
        } else {
          handleStreamFailure(robotId, attempt);
        }
        return;
      }
      const player = new HlsPlayer({
        lowLatencyMode: true,
        backBufferLength: 0,
        liveSyncDurationCount: 2,
        liveMaxLatencyDurationCount: 5
      });
      hls = player;
      player.on(HlsPlayer.Events.MEDIA_ATTACHED, () => {
        if (streamGate.isCurrent(attempt, activeId) && hls === player) {
          player.loadSource(url);
        }
      });
      player.on(HlsPlayer.Events.MANIFEST_PARSED, () => {
        if (streamGate.isCurrent(attempt, activeId) && hls === player) {
          void target.play().catch(() => {});
        }
      });
      player.on(HlsPlayer.Events.ERROR, (_event, data) => {
        if (data.fatal && streamGate.isCurrent(attempt, activeId) && hls === player) {
          handleStreamFailure(robotId, attempt);
        }
      });
      player.attachMedia(target);
    } catch {
      if (!streamGate.isCurrent(attempt, activeId)) return;
      hls?.destroy();
      hls = null;
      if (target.canPlayType('application/vnd.apple.mpegurl')) {
        connectNativeHls();
      } else {
        handleStreamFailure(robotId, attempt);
      }
    }
  }

  /** Report a full WHEP/HLS outage and retry the preferred path with backoff. */
  function handleStreamFailure(robotId: string, attempt: CameraStreamAttempt) {
    if (!streamGate.isCurrent(attempt, activeId)) return;
    streamGate.invalidate();
    closeTransport();
    streamFailures += 1;
    streamState = 'unavailable';
    if (streamRetryTimer) clearTimeout(streamRetryTimer);
    streamRetryTimer = window.setTimeout(() => {
      streamRetryTimer = null;
      if (activeId !== robotId) return;
      void connectWhep(robotId);
    }, cameraRetryDelayMs(streamFailures));
  }

  function closePeer() {
    whepAbort?.abort();
    whepAbort = null;
    const closing = pc;
    pc = null;
    if (closing) closing.onconnectionstatechange = null;
    closing?.close();
    if (video) video.srcObject = null;
  }

  function closeHls() {
    nativeHlsCleanup?.();
    nativeHlsCleanup = null;
    const closing = hls;
    hls = null;
    closing?.destroy();
    if (video) {
      video.pause();
      video.removeAttribute('src');
      video.load();
    }
  }

  function closeTransport() {
    if (transportProbeTimer) clearTimeout(transportProbeTimer);
    transportProbeTimer = null;
    stopVideoFrameLoop();
    stopStatsPolling();
    closePeer();
    closeHls();
    fps = 0;
    pingLatencyMs = 0;
    rawLatency = 0;
    streamSource = null;
    updateOverlayPosition();
  }

  function teardown() {
    streamGate.invalidate();
    if (streamRetryTimer) clearTimeout(streamRetryTimer);
    streamRetryTimer = null;
    closeTransport();
    streamFailures = 0;
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

  $effect(() => {
    if (!containerEl) return;
    const ro = new ResizeObserver(() => {
      updateOverlayPosition();
    });
    ro.observe(containerEl);
    return () => ro.disconnect();
  });
</script>

<div
  class="panel-glow relative flex flex-col overflow-hidden rounded-[--radius-panel] border border-transparent
         bg-surface-2 {expanded ? 'h-full min-h-0' : 'min-h-[180px] flex-1'}"
>
  <div
    bind:this={containerEl}
    class="relative h-full w-full flex-1 overflow-hidden"
  >
    <video
      bind:this={video}
      class="h-full w-full object-contain {streamSource ? '' : 'hidden'}"
      autoplay
      muted
      playsinline
      onplay={() => {
        updateOverlayPosition();
      }}
      onloadedmetadata={updateOverlayPosition}
      onresize={updateOverlayPosition}
      ontimeupdate={updateOverlayPosition}
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
    <div
      bind:this={overlayEl}
      class="pointer-events-none absolute z-30"
    >
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

    <!-- Resize button on bottom-left of video -->
    <button
      class="absolute bottom-1.5 left-1.5 z-30 grid h-6 w-6 place-items-center rounded-[--radius-control]
             border border-white/10 bg-black/65 text-white/80 shadow-sm backdrop-blur-md
             transition-colors hover:bg-black/85 hover:text-white active:scale-95"
      title={expanded ? 'Restore video' : 'Expand video'}
      aria-label={expanded ? 'Restore video' : 'Expand video'}
      onclick={ontoggleexpand}
    >
      {#if expanded}
        <Minimize2 class="h-3 w-3" />
      {:else}
        <Maximize2 class="h-3 w-3" />
      {/if}
    </button>

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
</div>
