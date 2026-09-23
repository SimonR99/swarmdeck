import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  DECORATION_FPS,
  DeadlineWakeup,
  RenderScheduler,
  type RenderRequest
} from '../src/lib/components/map3d/renderScheduler.ts';

import { LayerUpdateGate } from '../src/lib/components/map3d/layerUpdateGate.ts';

const FPS = 30;

function request(overrides: Partial<RenderRequest> = {}): RenderRequest {
  return { now: 0, hidden: false, moving: false, decorating: false, fps: FPS, ...overrides };
}

/** Frames drawn over a second of animation frames at 60 Hz. */
function framesInOneSecond(scheduler: RenderScheduler, overrides: Partial<RenderRequest>): number {
  let drawn = 0;
  for (let i = 1; i <= 60; i++) {
    const decision = scheduler.frame(request({ now: i * (1000 / 60), ...overrides }));
    if (decision.render) drawn++;
  }
  return drawn;
}

test('a moving fleet is drawn at the quality tier frame cap', () => {
  const scheduler = new RenderScheduler();
  scheduler.frame(request());
  const drawn = framesInOneSecond(scheduler, { moving: true });
  assert.ok(Math.abs(drawn - FPS) <= 1, `${drawn} frames`);
});

test('decoration alone is drawn at the decoration rate, not the frame cap', () => {
  const scheduler = new RenderScheduler();
  scheduler.frame(request());
  const drawn = framesInOneSecond(scheduler, { decorating: true });
  assert.ok(Math.abs(drawn - DECORATION_FPS) <= 1, `${drawn} frames`);
  assert.ok(drawn < FPS);
});

test('a parked fleet with nothing animated is not drawn at all', () => {
  const scheduler = new RenderScheduler();
  scheduler.frame(request());
  assert.equal(framesInOneSecond(scheduler, {}), 0);
  assert.equal(scheduler.frame(request({ now: 5000 })).again, false);
});

test('a hidden tab draws nothing and stops asking for frames', () => {
  const scheduler = new RenderScheduler();
  scheduler.frame(request());
  const decision = scheduler.frame(request({ now: 100, hidden: true, moving: true }));
  assert.deepEqual(decision, { render: false, again: false });
});

test('the first frame after the tab returns is drawn, however long it was away', () => {
  const scheduler = new RenderScheduler();
  scheduler.frame(request());
  scheduler.frame(request({ now: 100, hidden: true }));
  assert.equal(scheduler.frame(request({ now: 110 })).render, true);
  // And then it settles back to idle.
  assert.equal(scheduler.frame(request({ now: 120 })).render, false);
});

test('a store or input change is drawn at once, at the full budget', () => {
  const scheduler = new RenderScheduler();
  scheduler.frame(request({ now: 0 }));
  assert.equal(scheduler.frame(request({ now: 8 })).render, false);
  scheduler.markDirty();
  const decision = scheduler.frame(request({ now: 40 }));
  assert.deepEqual(decision, { render: true, again: false });
});

test('a change while only decoration animates is not delayed to the decoration rate', () => {
  const scheduler = new RenderScheduler();
  scheduler.frame(request({ now: 0, decorating: true }));
  assert.equal(scheduler.frame(request({ now: 20, decorating: true })).render, false);
  scheduler.markDirty();
  assert.equal(scheduler.frame(request({ now: 40, decorating: true })).render, true);
});

/** setTimeout stand-in driven by a hand-advanced clock. */
function fakeClock() {
  let now = 0;
  let next = 1;
  const pending = new Map<number, { at: number; run: () => void }>();
  return {
    now: () => now,
    timers: {
      set: (run: () => void, delayMs: number) => {
        const handle = next++;
        pending.set(handle, { at: now + delayMs, run });
        return handle;
      },
      clear: (handle: unknown) => void pending.delete(handle as number)
    },
    advanceTo(time: number) {
      for (;;) {
        const due = [...pending.entries()].filter(([, t]) => t.at <= time).sort((a, b) => a[1].at - b[1].at)[0];
        if (!due) break;
        pending.delete(due[0]);
        now = due[1].at;
        due[1].run();
      }
      now = time;
    },
    get pendingCount() {
      return pending.size;
    }
  };
}

test('an expiry with no further response still marks a quiet scene dirty at the deadline', () => {
  const clock = fakeClock();
  const scheduler = new RenderScheduler();
  // Settle the scene: one frame drawn, nothing moving, no more frames wanted.
  assert.deepEqual(scheduler.frame(request({ now: 0 })), { render: true, again: false });
  // Two drawn items go stale at 2000 and 3000; no new frame ever arrives.
  const deadlines = [2000, 3000];
  const wakeup = new DeadlineWakeup(
    (now) => deadlines.find((deadline) => deadline > now) ?? null,
    () => scheduler.markDirty(),
    clock.now,
    clock.timers
  );
  wakeup.arm();
  clock.advanceTo(1999);
  assert.equal(scheduler.pending, false);
  clock.advanceTo(2000);
  assert.equal(scheduler.pending, true);
  assert.equal(scheduler.frame(request({ now: 2000 })).render, true);
  clock.advanceTo(2999);
  assert.equal(scheduler.pending, false);
  clock.advanceTo(3000);
  assert.equal(scheduler.pending, true);
  assert.equal(clock.pendingCount, 0);
});

test('two freshness expiries within the layer budget both remove expired layers without later responses', () => {
  const clock = fakeClock();
  const scheduler = new RenderScheduler();
  const layers = new LayerUpdateGate();
  const expiries = [1001, 1101];
  let drawn = [...expiries];
  scheduler.frame(request());
  const wakeup = new DeadlineWakeup(
    (now) => expiries.find((expiry) => expiry > now) ?? null,
    () => { layers.invalidateFreshness(); scheduler.markDirty(); },
    clock.now, clock.timers
  );
  wakeup.arm();
  for (const now of expiries) {
    clock.advanceTo(now);
    assert.equal(scheduler.frame(request({ now })).render, true);
    if (layers.take(now)) drawn = expiries.filter((expiry) => expiry > now);
    assert.deepEqual(drawn, expiries.filter((expiry) => expiry > now));
  }
  assert.equal(clock.pendingCount, 0);
  assert.equal(scheduler.frame(request({ now: 1200 })).again, false);
});

test('re-arming replaces the pending wake-up instead of adding one', () => {
  const clock = fakeClock();
  let wakes = 0;
  let deadline: number | null = 2000;
  const wakeup = new DeadlineWakeup(() => deadline, () => wakes++, clock.now, clock.timers);
  wakeup.arm();
  deadline = 5000;
  wakeup.arm();
  assert.equal(clock.pendingCount, 1);
  clock.advanceTo(4000);
  assert.equal(wakes, 0);
  deadline = null;
  clock.advanceTo(5000);
  assert.equal(wakes, 1);
  wakeup.arm();
  wakeup.cancel();
  assert.equal(clock.pendingCount, 0);
});
