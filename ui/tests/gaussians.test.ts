import { test } from 'node:test';
import assert from 'node:assert/strict';
import { RenderScheduler } from '../src/lib/components/map3d/renderScheduler.ts';

/** Stands in for the sort worker: records posts, replies when the test says so. */
class FakeWorker {
  static last: FakeWorker | null = null;
  onmessage: ((event: MessageEvent) => void) | null = null;
  posted: { id: number; view?: number[] }[] = [];
  constructor() {
    FakeWorker.last = this;
  }
  postMessage(message: { id: number; view?: number[] }) {
    this.posted.push(message);
  }
  terminate() {}
  reply(id: number, order: Uint32Array) {
    this.onmessage?.({ data: { id, order } } as MessageEvent);
  }
}
(globalThis as unknown as { Worker: unknown }).Worker = FakeWorker;
const { GaussianLayer } = await import('../src/lib/components/map3d/gaussians.ts');

/** An SWGS v1 model of `n` unit splats along x. */
function model(n: number): ArrayBuffer {
  const buffer = new ArrayBuffer(16 + n * 56);
  const header = new DataView(buffer);
  header.setUint32(0, 0x53475753, true);
  header.setUint32(4, 1, true);
  header.setUint32(8, n, true);
  const records = new Float32Array(buffer, 16);
  for (let i = 0; i < n; i++) {
    records.set([i, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0.5, 0.5, 0.5, 1], i * 14);
  }
  return buffer;
}

/** A scene that has drawn its last frame and asks for no more. */
function quietScene() {
  const scheduler = new RenderScheduler();
  assert.deepEqual(scheduler.frame({ now: 0, hidden: false, moving: false, decorating: false, fps: 30 }), {
    render: true,
    again: false
  });
  const layer = new GaussianLayer();
  layer.onDirty = () => scheduler.markDirty();
  return { scheduler, layer, worker: FakeWorker.last! };
}

test('an accepted depth sort marks a quiet scene dirty', () => {
  const { scheduler, layer, worker } = quietScene();
  layer.load(model(3), 10);
  scheduler.frame({ now: 100, hidden: false, moving: false, decorating: false, fps: 30 });
  assert.equal(scheduler.pending, false);
  const { id } = worker.posted[0];
  worker.reply(id, new Uint32Array([2, 1, 0]));
  assert.equal(scheduler.pending, true);
});

test('a sort for a model that has since been replaced is ignored', () => {
  const { scheduler, layer, worker } = quietScene();
  layer.load(model(3), 10);
  const { id } = worker.posted[0];
  layer.load(model(2), 10);
  scheduler.frame({ now: 100, hidden: false, moving: false, decorating: false, fps: 30 });
  worker.reply(id, new Uint32Array([2, 1, 0]));
  assert.equal(scheduler.pending, false);
});

test('clearing loaded splats, as a 404 does, marks a quiet scene dirty', () => {
  const { scheduler, layer } = quietScene();
  layer.load(model(3), 10);
  scheduler.frame({ now: 100, hidden: false, moving: false, decorating: false, fps: 30 });
  layer.clear();
  assert.equal(scheduler.pending, true);
  scheduler.frame({ now: 200, hidden: false, moving: false, decorating: false, fps: 30 });
  // Nothing was drawn, so clearing again changes nothing on screen.
  layer.clear();
  assert.equal(scheduler.pending, false);
});
