/** Browser-safe UUID and bounded JSON transport for simulation reset recovery. */

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
