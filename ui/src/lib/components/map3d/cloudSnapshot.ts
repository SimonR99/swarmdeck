import { inflate } from 'pako';

type Chunk = { id: string; points: number };
type Manifest = { version: number; headers: Record<string, string>; chunks: Chunk[] };

/** Immutable spatial tiles survive updates, visibility changes and map selection. */
export class CloudSnapshotCache {
  private chunks = new Map<string, Uint8Array>();
  private bytes = 0;

  async fetch(url: string, etag: string, signal: AbortSignal) {
    const response = await fetch(`${url}&manifest=1`, {
      signal, headers: etag ? { 'If-None-Match': etag } : {}
    });
    if (response.status === 304) return { response, raw: null };
    if (!response.ok) throw new Error(`Cloud unavailable (${response.status})`);
    // Compatible with a server deployed before chunk delivery.
    if (!response.headers.get('Content-Type')?.includes('application/json')) {
      return { response, raw: inflate(new Uint8Array(await response.arrayBuffer())) };
    }
    const manifest: Manifest = await response.json();
    const total = Number(response.headers.get('X-Cloud-Points'));
    const xyzBytes = response.headers.get('X-Cloud-Format') === 'xyz32' ? 12 : 6;
    const colorBytes = response.headers.get('X-Cloud-RGB') === '1' ? 3 : 0;
    const stride = xyzBytes + 1 + colorBytes;
    if (manifest.version !== 1 || !Array.isArray(manifest.chunks) ||
        !Number.isInteger(total) || total < 0 || total > 2_000_000 ||
        manifest.chunks.some(c => !/^[a-f0-9]{64}$/.test(c.id) || !Number.isInteger(c.points) || c.points <= 0) ||
        manifest.chunks.reduce((n, c) => n + c.points, 0) !== total) {
      throw new Error('Invalid cloud manifest');
    }
    const raw = new Uint8Array(total * stride);
    let next = 0;
    const offsets: number[] = [];
    let count = 0;
    for (const chunk of manifest.chunks) { offsets.push(count); count += chunk.points; }
    const download = async () => {
      while (next < manifest.chunks.length) {
        const index = next++;
        const chunk = manifest.chunks[index];
        let data = this.chunks.get(chunk.id);
        if (!data) {
          const part = await fetch(`/api/map/cloud?chunk=${chunk.id}`, { signal });
          if (!part.ok) throw new Error(`Cloud chunk unavailable (${part.status})`);
          data = inflate(new Uint8Array(await part.arrayBuffer()));
          if (data.length !== chunk.points * stride) throw new Error('Invalid cloud chunk');
          this.chunks.set(chunk.id, data);
          this.bytes += data.byteLength;
        } else {
          this.chunks.delete(chunk.id);
          this.chunks.set(chunk.id, data);
        }
        if (data.length !== chunk.points * stride) throw new Error('Invalid cached cloud chunk');
        const offset = offsets[index], n = chunk.points;
        raw.set(data.subarray(0, n * xyzBytes), offset * xyzBytes);
        raw.set(data.subarray(n * xyzBytes, n * (xyzBytes + 1)), total * xyzBytes + offset);
        if (colorBytes) raw.set(data.subarray(n * (xyzBytes + 1)), total * (xyzBytes + 1) + offset * 3);
        while (this.bytes > 64 * 1024 * 1024) {
          const first = this.chunks.keys().next().value!;
          this.bytes -= this.chunks.get(first)!.byteLength;
          this.chunks.delete(first);
        }
      }
    };
    await Promise.all(Array.from({ length: Math.min(4, manifest.chunks.length) }, download));
    return { response, raw };
  }
}
