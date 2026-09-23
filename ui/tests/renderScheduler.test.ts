import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  DECORATION_FPS,
  RenderScheduler,
  type RenderRequest
} from '../src/lib/components/map3d/renderScheduler.ts';

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
