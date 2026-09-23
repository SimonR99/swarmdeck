import { test } from 'node:test';
import assert from 'node:assert/strict';
import { RenderLoop, type FrameClock, type RenderLoopHost } from '../src/lib/components/map3d/renderLoop.ts';
import { DECORATION_FPS, MOTION_LINGER_MS } from '../src/lib/components/map3d/renderScheduler.ts';

/** A display refreshing at 60 Hz whose frames the test steps through. */
class FakeFrames implements FrameClock {
  time = 1000;
  private waiting = new Map<number, (timestamp: number) => void>();
  private next = 1;
  request = (callback: (timestamp: number) => void) => {
    const handle = this.next++;
    this.waiting.set(handle, callback);
    return handle;
  };
  cancel = (handle: number) => {
    this.waiting.delete(handle);
  };
  now = () => this.time;
  get pending() {
    return this.waiting.size;
  }
  /** Advance one display refresh and run the frames that were asked for. */
  step() {
    this.time += 1000 / 60;
    const callbacks = [...this.waiting.values()];
    this.waiting.clear();
    for (const callback of callbacks) callback(this.time);
  }
}

function harness(overrides: Partial<{ canStart: boolean; hidden: boolean; decorating: boolean; scene: boolean }> = {}) {
  const frames = new FakeFrames();
  const flags = { canStart: true, hidden: false, decorating: false, scene: true, ...overrides };
  const drawn: { timestamp: number; layers: boolean }[] = [];
  const host: RenderLoopHost = {
    canStart: () => flags.canStart,
    frameInputs: () => (flags.scene ? { hidden: flags.hidden, decorating: flags.decorating, fps: 30 } : null),
    draw: (timestamp, layers) => drawn.push({ timestamp, layers })
  };
  return { frames, flags, drawn, loop: new RenderLoop(host, frames) };
}

function run(frames: FakeFrames, ms: number) {
  const end = frames.time + ms;
  while (frames.time < end) frames.step();
}

test('a change draws one frame and the loop stops', () => {
  const { frames, drawn, loop } = harness();
  loop.request();
  loop.request();
  assert.equal(frames.pending, 1, 'requests while a frame is pending do not stack');
  run(frames, 1000);
  assert.equal(drawn.length, 1);
  assert.equal(frames.pending, 0);
});

test('nothing is requested while the view cannot draw, and nothing drawn without a scene', () => {
  const idle = harness({ canStart: false });
  idle.loop.request(true);
  assert.equal(idle.frames.pending, 0);
  const sceneless = harness({ scene: false });
  sceneless.loop.request(true);
  run(sceneless.frames, 1000);
  assert.equal(sceneless.drawn.length, 0);
  assert.equal(sceneless.frames.pending, 0);
});

test('movement is drawn at the tier cap for the linger period, then the loop stops', () => {
  const { frames, drawn, loop } = harness();
  loop.request(true);
  run(frames, 1000);
  const expected = Math.round((MOTION_LINGER_MS / 1000) * 30);
  assert.ok(Math.abs(drawn.length - expected) <= 2, `${drawn.length} frames, about ${expected} expected`);
  assert.equal(frames.pending, 0);
});

test('decoration keeps the loop alive at the decoration rate', () => {
  const { frames, drawn, loop } = harness({ decorating: true });
  loop.request();
  run(frames, 1000);
  drawn.length = 0;
  run(frames, 1000);
  assert.ok(Math.abs(drawn.length - DECORATION_FPS) <= 1, `${drawn.length} frames in a second`);
  assert.equal(frames.pending, 1);
});

test('pausing cancels the pending frame and the next request draws', () => {
  const { frames, drawn, loop } = harness({ decorating: true });
  loop.request();
  loop.pause();
  assert.equal(frames.pending, 0);
  loop.request();
  frames.step();
  assert.equal(drawn.length, 1);
  loop.stop();
  assert.equal(frames.pending, 0);
});

test('the layer budget is taken per drawn frame and a freshness expiry overrides it', () => {
  const { frames, drawn, loop } = harness();
  loop.request();
  frames.step();
  loop.request();
  frames.step();
  frames.step();
  loop.invalidateFreshness();
  loop.request();
  frames.step();
  frames.step();
  assert.deepEqual(drawn.map((frame) => frame.layers), [true, false, true]);
});
