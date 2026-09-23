import { test } from 'node:test';
import assert from 'node:assert/strict';
import {
  MAP_POLL_TICK_MS,
  MAP_RASTER_INTERVAL_MS,
  MAP_SCOPES_INTERVAL_MS,
  MAP_STATUS_INTERVAL_MS,
  MapPollScheduler,
  type MapPollInputs,
  type MapPollWork
} from '../src/lib/components/map2d/mapPollScheduler.ts';

function inputs(overrides: Partial<MapPollInputs> = {}): MapPollInputs {
  return { now: 0, hidden: false, rasterVisible: true, rasterReady: true, ...overrides };
}

/** Run ten seconds of ticks and count what each part was asked to do. */
function overTenSeconds(
  scheduler: MapPollScheduler,
  overrides: Partial<MapPollInputs> = {}
): MapPollWork & { ticks: number } {
  const counts = { scopes: 0, status: 0, raster: 0, ticks: 0 };
  for (let now = 0; now < 10_000; now += MAP_POLL_TICK_MS) {
    const work = scheduler.due(inputs({ now, ...overrides }));
    counts.ticks++;
    if (work.scopes) counts.scopes++;
    if (work.status) counts.status++;
    if (work.raster) counts.raster++;
  }
  return counts as unknown as MapPollWork & { ticks: number };
}

test('each part keeps its own cadence on one shared tick', () => {
  const counts = overTenSeconds(new MapPollScheduler());
  assert.equal(counts.scopes, 10_000 / MAP_SCOPES_INTERVAL_MS);
  assert.equal(counts.status, Math.ceil(10_000 / MAP_STATUS_INTERVAL_MS));
  assert.equal(counts.raster, 10_000 / MAP_RASTER_INTERVAL_MS);
});

test('the raster image is not fetched while the 3D view is covering the canvas', () => {
  const counts = overTenSeconds(new MapPollScheduler(), { rasterVisible: false });
  assert.equal(counts.raster, 0);
  // The catalogue and the merge status still describe both views.
  assert.ok(counts.scopes > 0 && counts.status > 0);
});

test('a raster that has never loaded is fetched even under the 3D view', () => {
  const counts = overTenSeconds(new MapPollScheduler(), {
    rasterVisible: false,
    rasterReady: false
  });
  assert.equal(counts.raster, 10_000 / MAP_RASTER_INTERVAL_MS);
});

test('a hidden tab polls nothing and catches up the moment it returns', () => {
  const scheduler = new MapPollScheduler();
  assert.deepEqual(scheduler.due(inputs({ now: 0 })), {
    scopes: true,
    status: true,
    raster: true
  });
  for (let now = 500; now <= 60_000; now += MAP_POLL_TICK_MS) {
    assert.deepEqual(scheduler.due(inputs({ now, hidden: true })), {
      scopes: false,
      status: false,
      raster: false
    });
  }
  assert.deepEqual(scheduler.due(inputs({ now: 60_500 })), {
    scopes: true,
    status: true,
    raster: true
  });
});

test('switching back to the 2D canvas reloads the raster at once', () => {
  const scheduler = new MapPollScheduler();
  scheduler.due(inputs({ now: 0 }));
  assert.equal(scheduler.due(inputs({ now: 500, rasterVisible: false })).raster, false);
  scheduler.invalidate();
  assert.equal(scheduler.due(inputs({ now: 600 })).raster, true);
});
