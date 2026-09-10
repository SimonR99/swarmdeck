export type CameraTransport = 'webrtc' | 'hls';

export type CameraStreamAttempt = Readonly<{
  generation: number;
  robotId: string;
  transport: CameraTransport;
}>;

/** Fence callbacks from a previous robot or transport attempt. */
export class CameraStreamGate {
  #generation = 0;

  begin(robotId: string, transport: CameraTransport): CameraStreamAttempt {
    this.#generation += 1;
    return { generation: this.#generation, robotId, transport };
  }

  invalidate(): void {
    this.#generation += 1;
  }

  isCurrent(attempt: CameraStreamAttempt, robotId: string | null): boolean {
    return attempt.generation === this.#generation && attempt.robotId === robotId;
  }
}

export function hlsCameraUrl(robotId: string): string {
  return `/hls/${encodeURIComponent(robotId)}/index.m3u8`;
}

export function cameraRetryDelayMs(failures: number): number {
  const boundedFailures = Math.max(1, Math.min(5, Math.trunc(failures)));
  return Math.min(30_000, 2_000 * 2 ** (boundedFailures - 1));
}

export function decodedFrameIsStalled(
  lastFrameAt: number,
  now: number,
  timeoutMs: number
): boolean {
  return lastFrameAt > 0 && now - lastFrameAt > timeoutMs;
}
