/** Largest reconstruction the map downloads, in bytes (header included). */
export const MAX_GAUSSIAN_BYTES = 112_000_016;
export const GAUSSIAN_TIMEOUT_MS = 15_000;

/** What one reconstruction request found. */
export type GaussianResult =
  | { kind: 'unchanged' }
  | { kind: 'missing' }
  | { kind: 'loaded'; buffer: ArrayBuffer; etag: string }
  | { kind: 'failed'; message: string };

export interface GaussianFetchView {
  /** The query naming the replica shown now; a response for another is dropped. */
  scope(): string;
  /** Whether a scene exists to show a response in. */
  hasScene(): boolean;
}

export interface GaussianFetchDeps {
  fetch: typeof fetch;
  setTimeout: (run: () => void, ms: number) => unknown;
  clearTimeout: (handle: unknown) => void;
}

const browserDeps: GaussianFetchDeps = {
  fetch: (input, init) => fetch(input, init),
  setTimeout: (run, ms) => globalThis.setTimeout(run, ms),
  clearTimeout: (handle) => globalThis.clearTimeout(handle as ReturnType<typeof setTimeout>)
};

/**
 * The 3D map's Gaussian reconstruction download, apart from the Svelte view.
 *
 * It sends the ETag of the reconstruction on show so an unchanged one costs a
 * 304, keeps one request in flight, and drops a response once the view shows
 * another replica. `fetch` resolves to null when the view has nothing to do,
 * else to what the server said; the view loads a buffer and then calls
 * `accept` with its ETag.
 */
export class GaussianFetch {
  private etag = '';
  private pending: AbortController | null = null;
  private readonly view: GaussianFetchView;
  private readonly deps: GaussianFetchDeps;

  constructor(view: GaussianFetchView, deps: Partial<GaussianFetchDeps> = {}) {
    this.view = view;
    this.deps = { ...browserDeps, ...deps };
  }

  get busy(): boolean {
    return this.pending !== null;
  }

  async fetch(): Promise<GaussianResult | null> {
    if (this.pending) return null;
    const scope = this.view.scope();
    const controller = new AbortController();
    this.pending = controller;
    const timeout = this.deps.setTimeout(() => controller.abort(), GAUSSIAN_TIMEOUT_MS);
    const current = () => this.view.hasScene() && scope === this.view.scope() && !controller.signal.aborted;
    try {
      const response = await this.deps.fetch(`/api/map/gaussians${scope}`, {
        signal: controller.signal,
        headers: this.etag ? { 'If-None-Match': this.etag } : {}
      });
      if (!current()) return null;
      if (response.status === 304) return { kind: 'unchanged' };
      if (response.status === 404) {
        this.etag = '';
        return { kind: 'missing' };
      }
      if (!response.ok) throw new Error(`Reconstruction unavailable (${response.status})`);
      if (Number(response.headers.get('Content-Length')) > MAX_GAUSSIAN_BYTES)
        throw new Error('Reconstruction exceeds download limit');
      const buffer = await response.arrayBuffer();
      if (!current()) return null;
      return { kind: 'loaded', buffer, etag: response.headers.get('ETag') ?? '' };
    } catch (reason) {
      if (controller.signal.aborted || scope !== this.view.scope()) return null;
      return { kind: 'failed', message: reason instanceof Error ? reason.message : String(reason) };
    } finally {
      this.deps.clearTimeout(timeout);
      if (this.pending === controller) this.pending = null;
    }
  }

  /** The reconstruction with this ETag is now on show. */
  accept(etag: string) {
    this.etag = etag;
  }

  /** Download the next reconstruction whole, whatever is on show. */
  forget() {
    this.etag = '';
  }

  abort() {
    this.pending?.abort();
    this.pending = null;
  }
}
