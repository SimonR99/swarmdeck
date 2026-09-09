export const MAX_CHUNK_BYTES = 8 * 1024 * 1024;
export const MAX_PREVIEW_POINTS = 50_000;
export const MAX_CACHE_BYTES = 64 * 1024 * 1024;

export type PreviewChunk = {
  submapId: string;
  sha256: string;
  points: Float32Array;
};

export type ChunkRef = { sha256: string; point_count?: number; size_bytes?: number };

/** Bound downloads as well as GPU points, sampling across the full submap list. */
export function selectPreviewRefs(refs: ChunkRef[], maxBytes = MAX_CACHE_BYTES): ChunkRef[] {
  const unique = [...new Map(refs.map((ref) => [ref.sha256, ref])).values()];
  const size = (ref: ChunkRef) => {
    const bytes = ref.size_bytes ?? MAX_CHUNK_BYTES;
    if (!Number.isSafeInteger(bytes) || bytes < 16 || bytes > MAX_CHUNK_BYTES) {
      throw new Error('Invalid replica chunk size');
    }
    return bytes;
  };
  unique.forEach(size);
  let count = unique.length;
  while (count > 0) {
    const selected = Array.from({ length: count }, (_, i) => unique[Math.floor(i * unique.length / count)]);
    const bytes = selected.reduce((sum, ref) => sum + size(ref), 0);
    if (bytes <= maxBytes) return selected;
    count = Math.min(count - 1, Math.floor(count * maxBytes / bytes));
  }
  return [];
}

/** Parse the immutable XYZ-F32 transport without accepting unbounded input. */
export function parseXYZF32(bytes: Uint8Array): Float32Array {
  if (bytes.byteLength < 16 || bytes.byteLength > MAX_CHUNK_BYTES) {
    throw new Error('Replica chunk exceeds the browser safety limit');
  }
  const magic = new TextDecoder().decode(bytes.subarray(0, 8));
  if (magic !== 'SDXYZ1\0\0') throw new Error('Unsupported replica chunk encoding');
  const count = Number(new DataView(bytes.buffer, bytes.byteOffset + 8, 8).getBigUint64(0, true));
  const maxPoints = Math.floor((MAX_CHUNK_BYTES - 16) / 12);
  if (!Number.isSafeInteger(count) || count < 0 || count > maxPoints || bytes.byteLength !== 16 + count * 12) {
    throw new Error('Invalid replica chunk length');
  }
  const points = new Float32Array(count * 3);
  const values = new DataView(bytes.buffer, bytes.byteOffset + 16, count * 12);
  for (let index = 0; index < points.length; index += 1) points[index] = values.getFloat32(index * 4, true);
  return points;
}

/** A small LRU cache whose retained typed arrays have an explicit byte budget. */
export class ReplicaChunkCache {
  private readonly entries = new Map<string, Float32Array>();
  private retainedBytes = 0;
  private readonly maxBytes: number;

  constructor(maxBytes = MAX_CACHE_BYTES) {
    this.maxBytes = maxBytes;
  }

  get(key: string): Float32Array | undefined {
    const value = this.entries.get(key);
    if (value !== undefined) {
      this.entries.delete(key);
      this.entries.set(key, value);
    }
    return value;
  }

  set(key: string, value: Float32Array): boolean {
    const bytes = value.byteLength;
    if (bytes > this.maxBytes) return false;
    const previous = this.entries.get(key);
    if (previous) this.retainedBytes -= previous.byteLength;
    this.entries.delete(key);
    this.entries.set(key, value);
    this.retainedBytes += bytes;
    while (this.retainedBytes > this.maxBytes) {
      const oldest = this.entries.keys().next().value;
      if (oldest === undefined) break;
      const removed = this.entries.get(oldest);
      this.entries.delete(oldest);
      if (removed) this.retainedBytes -= removed.byteLength;
    }
    return this.entries.get(key) === value;
  }

  get sizeBytes(): number {
    return this.retainedBytes;
  }

  retain(keys: Set<string>): void {
    for (const [key, points] of this.entries) {
      if (!keys.has(key)) {
        this.entries.delete(key);
        this.retainedBytes -= points.byteLength;
      }
    }
  }
}

/** Apply one deterministic global point limit across all selected chunks. */
export function samplePreviewChunks(chunks: PreviewChunk[], maxPoints = MAX_PREVIEW_POINTS): PreviewChunk[] {
  const totalPoints = chunks.reduce((total, chunk) => total + Math.floor(chunk.points.length / 3), 0);
  if (totalPoints <= maxPoints) return chunks;
  const stride = Math.ceil(totalPoints / maxPoints);
  let offset = 0;
  return chunks.map((chunk) => {
    const sourceCount = Math.floor(chunk.points.length / 3);
    const first = (stride - offset % stride) % stride;
    offset += sourceCount;
    const sampled = new Float32Array(Math.max(0, Math.ceil((sourceCount - first) / stride)) * 3);
    let output = 0;
    for (let point = first; point < sourceCount; point += stride) {
      sampled[output++] = chunk.points[point * 3];
      sampled[output++] = chunk.points[point * 3 + 1];
      sampled[output++] = chunk.points[point * 3 + 2];
    }
    return { ...chunk, points: sampled };
  });
}

/** A generation token makes stale refresh results harmless across UI requests. */
export class ReplicaRequestGate {
  private generation = 0;
  private controller: AbortController | null = null;

  begin(): { generation: number; signal: AbortSignal } {
    this.controller?.abort();
    this.controller = new AbortController();
    this.generation += 1;
    return { generation: this.generation, signal: this.controller.signal };
  }

  isCurrent(generation: number): boolean {
    return generation === this.generation;
  }

  cancel(): void {
    this.controller?.abort();
    this.controller = null;
    this.generation += 1;
  }
}
