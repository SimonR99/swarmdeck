/** Browser-safe UUID and bounded JSON transport for simulation reset recovery. */

/** Robot capabilities do not prove the host reset supervisor is running. */
export function canResetSimulation(
  status: { supervisor_available?: unknown } | undefined
): boolean {
  return status?.supervisor_available === true;
}

export function resetRequestId(
  randomValues: (array: Uint8Array<ArrayBuffer>) => void = (array) => {
    globalThis.crypto.getRandomValues(array);
  }
): string {
  const bytes = new Uint8Array(16);
  randomValues(bytes);
  bytes[6] = (bytes[6] & 0x0f) | 0x40;
  bytes[8] = (bytes[8] & 0x3f) | 0x80;
  const hex = [...bytes].map((value) => value.toString(16).padStart(2, '0'));
  return `${hex.slice(0, 4).join('')}-${hex.slice(4, 6).join('')}-${hex.slice(6, 8).join('')}-${hex.slice(8, 10).join('')}-${hex.slice(10).join('')}`;
}

export async function fetchJsonWithTimeout<T>(
  input: RequestInfo | URL,
  init: RequestInit,
  timeoutMs: number,
  fetcher: typeof fetch = globalThis.fetch
): Promise<[Response, T]> {
  const controller = new AbortController();
  const timer = globalThis.setTimeout(() => controller.abort(), timeoutMs);
  try {
    const response = await fetcher(input, { ...init, signal: controller.signal });
    // Keep the deadline active through body consumption. A proxy can deliver
    // headers and then stall the JSON body while the backend is restarting.
    const body = await response.json() as T;
    return [response, body];
  } finally {
    globalThis.clearTimeout(timer);
  }
}

export interface RobotMapResetStatus {
  phase: 'idle' | 'accepted' | 'stopping' | 'starting' | 'verifying' | 'done' | 'failed';
  ok: boolean | null;
  request_id?: string;
  robot_id?: string;
  map_epoch?: number;
  run_id?: string;
  error?: string;
}

/** One click owns one durable request; accepted is not successful completion. */
export async function resetRobotMap(
  robotId: string,
  requestId = resetRequestId(),
  fetcher: typeof fetch = globalThis.fetch,
  pause: () => Promise<void> = () => {
    const { promise, resolve } = Promise.withResolvers<void>();
    setTimeout(resolve, 1_000);
    return promise;
  },
  timeoutMs = 90_000
): Promise<RobotMapResetStatus> {
  const endpoint = `/api/map/reset/${encodeURIComponent(robotId)}`;
  const deadline = Date.now() + timeoutMs;
  let accepted = false;
  while (Date.now() < deadline) {
    let response: Response;
    let status: RobotMapResetStatus;
    try {
      [response, status] = await fetchJsonWithTimeout<RobotMapResetStatus>(
        `${endpoint}?request_id=${encodeURIComponent(requestId)}`,
        accepted ? { cache: 'no-store' } : { method: 'POST' },
        5_000, fetcher
      );
    } catch {
      // A lost POST response must retry the same UUID, not request a second run.
      await pause();
      continue;
    }
    if (!response.ok || status.phase === 'failed') {
      throw new Error(status.error ?? `Robot map reset failed (${response.status})`);
    }
    if (!status.request_id || status.robot_id !== robotId) {
      throw new Error('Robot map reset returned an invalid request identity');
    }
    if (!accepted) requestId = status.request_id;
    if (status.request_id === requestId) {
      accepted = true;
      if (status.phase === 'done') {
        if (status.ok !== true || !Number.isInteger(status.map_epoch) || !status.run_id) {
          throw new Error('Robot map reset did not verify a fresh mapping run');
        }
        return status;
      }
    }
    await pause();
  }
  throw new Error('Robot map reset did not complete within 90 seconds; inspect reset status before retrying');
}
