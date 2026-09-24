import { test } from 'node:test';
import assert from 'node:assert/strict';
import { CanvasViewport, type CanvasView } from '../src/lib/components/map2d/canvasViewport.ts';

/*
 * The 2D viewport logic moved out of MapView.svelte. Each case runs the
 * component's code as it stood (copied below) and the controller from the
 * same starting view and compares the results.
 */

type View = CanvasView;
const start = (): View => ({ scale: 1.3, tx: 40, ty: -25, rotation: 0.4, initialised: true });

function legacyScreenOf(view: View, gx: number, gy: number) {
  const sx = gx * view.scale, sy = gy * view.scale;
  if (!view.rotation) return { sx: sx + view.tx, sy: sy + view.ty };
  const c = Math.cos(view.rotation), s = Math.sin(view.rotation);
  return { sx: sx * c - sy * s + view.tx, sy: sx * s + sy * c + view.ty };
}

function legacyGridOf(view: View, sx: number, sy: number) {
  const dx = sx - view.tx, dy = sy - view.ty;
  if (!view.rotation) return { gx: dx / view.scale, gy: dy / view.scale };
  const c = Math.cos(-view.rotation), s = Math.sin(-view.rotation);
  return { gx: (dx * c - dy * s) / view.scale, gy: (dx * s + dy * c) / view.scale };
}

function legacyZoom(view: View, factor: number, px: number, py: number) {
  const before = legacyGridOf(view, px, py);
  view.scale = Math.max(0.12, Math.min(6, view.scale * factor));
  const after = legacyScreenOf(view, before.gx, before.gy);
  view.tx += px - after.sx;
  view.ty += py - after.sy;
}

function legacyRotate(view: View, angleDelta: number, px: number, py: number) {
  const before = legacyGridOf(view, px, py);
  view.rotation += angleDelta;
  while (view.rotation > Math.PI) view.rotation -= Math.PI * 2;
  while (view.rotation < -Math.PI) view.rotation += Math.PI * 2;
  const after = legacyScreenOf(view, before.gx, before.gy);
  view.tx += px - after.sx;
  view.ty += py - after.sy;
}

function legacyFit(view: View, hostW: number, hostH: number, info: { width: number; height: number }) {
  const width = Math.max(1, hostW - 64), height = Math.max(1, hostH - 64);
  view.scale = Math.max(0.12, Math.min(6, Math.min(width / info.width, height / info.height)));
  view.tx = (hostW - info.width * view.scale) / 2;
  view.ty = (hostH - info.height * view.scale) / 2;
}

type XY = { x: number; y: number };
function legacyPinch(view: View, prev: XY, cur: XY, other: XY, rect: { left: number; top: number }) {
  const prevMid = { x: (prev.x + other.x) / 2 - rect.left, y: (prev.y + other.y) / 2 - rect.top };
  const curMid = { x: (cur.x + other.x) / 2 - rect.left, y: (cur.y + other.y) / 2 - rect.top };
  const prevD = Math.hypot(prev.x - other.x, prev.y - other.y);
  const curD = Math.hypot(cur.x - other.x, cur.y - other.y);
  let angleDelta = Math.atan2(other.y - cur.y, other.x - cur.x) - Math.atan2(other.y - prev.y, other.x - prev.x);
  while (angleDelta > Math.PI) angleDelta -= Math.PI * 2;
  while (angleDelta < -Math.PI) angleDelta += Math.PI * 2;
  const scaleFactor = prevD > 5 ? curD / prevD : 1.0;
  const anchor = legacyGridOf(view, prevMid.x, prevMid.y);
  view.scale = Math.max(0.12, Math.min(6, view.scale * scaleFactor));
  view.rotation += angleDelta;
  while (view.rotation > Math.PI) view.rotation -= Math.PI * 2;
  while (view.rotation < -Math.PI) view.rotation += Math.PI * 2;
  const after = legacyScreenOf(view, anchor.gx, anchor.gy);
  view.tx += curMid.x - after.sx;
  view.ty += curMid.y - after.sy;
}

function close(actual: View, expected: View) {
  for (const key of ['scale', 'tx', 'ty', 'rotation'] as const) {
    assert.ok(Math.abs(actual[key] - expected[key]) < 1e-9, `${key}: ${actual[key]} vs ${expected[key]}`);
  }
}

for (const rotation of [0, 0.4, -2.9]) {
  test(`screen and grid coordinates are unchanged at rotation ${rotation}`, () => {
    const view = { ...start(), rotation };
    const viewport = new CanvasViewport(view);
    assert.deepEqual(viewport.screenOf(12, -7), legacyScreenOf(view, 12, -7));
    assert.deepEqual(viewport.gridOf(310, 145), legacyGridOf(view, 310, 145));
  });
}

test('zooming keeps the cell under the cursor and the scale limits', () => {
  for (const factor of [1.12, 1 / 1.12, 100, 0.001]) {
    const expected = start();
    legacyZoom(expected, factor, 200, 120);
    const viewport = new CanvasViewport(start());
    viewport.zoomAt(factor, 200, 120);
    close(viewport.view, expected);
  }
});

test('rotating wraps to one turn about the anchor', () => {
  for (const angle of [Math.PI / 4, -Math.PI / 4, 3, -3]) {
    const expected = start();
    legacyRotate(expected, angle, 150, 90);
    const viewport = new CanvasViewport(start());
    viewport.rotateAt(angle, 150, 90);
    close(viewport.view, expected);
  }
});

test('fitting centres the raster inside the padding', () => {
  for (const [w, h, info] of [[800, 600, { width: 400, height: 900 }], [50, 40, { width: 10, height: 10 }]] as const) {
    const expected = start();
    legacyFit(expected, w, h, info);
    const viewport = new CanvasViewport(start());
    viewport.fit(w, h, info);
    close(viewport.view, expected);
  }
});

test('centring puts a cell in the middle of the canvas', () => {
  const viewport = new CanvasViewport({ ...start(), rotation: 0 });
  viewport.centreAt(10, 20, 800, 600);
  assert.deepEqual(viewport.screenOf(10, 20), { sx: 400, sy: 300 });
});

// The line from the moving finger to the still one turns from just below
// +180° to just above -180°: a raw step of about -360° + 0.02 rad. Only the
// worker test below runs it, so a wrap that hangs cannot stall this file.
const acrossPi: [XY, XY, XY] = [{ x: 300, y: 199 }, { x: 300, y: 201 }, { x: 200, y: 200 }];

test('a pinch zooms and rotates about the fingers', () => {
  const rect = { left: 12, top: 30 };
  const gestures: [XY, XY, XY][] = [
    [{ x: 300, y: 200 }, { x: 330, y: 190 }, { x: 200, y: 220 }],
    [{ x: 300, y: 200 }, { x: 302, y: 201 }, { x: 299, y: 199 }]
  ];
  for (const [prev, cur, other] of gestures) {
    const expected = start();
    legacyPinch(expected, prev, cur, other, rect);
    const viewport = new CanvasViewport(start());
    viewport.pinch(prev, cur, other, { x: rect.left, y: rect.top });
    close(viewport.view, expected);
  }
});

test('a pinch across ±180° turns the map by the small wrapped step about the fingers', async () => {
  const [prev, cur, other] = acrossPi;
  const angleOf = (finger: XY) => Math.atan2(other.y - finger.y, other.x - finger.x);
  assert.ok(angleOf(cur) - angleOf(prev) < -Math.PI, 'the fixture crosses ±180°');
  const step = angleOf(cur) - angleOf(prev) + Math.PI * 2;

  // A wrap that never terminates hangs synchronously, which no test timeout
  // can interrupt, so the gesture runs in a worker that is killed if it stalls.
  const { Worker } = await import('node:worker_threads');
  const moduleUrl = new URL('../src/lib/components/map2d/canvasViewport.ts', import.meta.url).href;
  const worker = new Worker(
    `const { parentPort, workerData } = require('node:worker_threads');
     import(workerData.moduleUrl).then(({ CanvasViewport }) => {
       const viewport = new CanvasViewport(workerData.view);
       const { prev, cur, other, origin } = workerData;
       const anchor = viewport.gridOf((prev.x + other.x) / 2 - origin.x, (prev.y + other.y) / 2 - origin.y);
       viewport.pinch(prev, cur, other, origin);
       parentPort.postMessage({ view: viewport.view, anchor: viewport.screenOf(anchor.gx, anchor.gy) });
     });`,
    { eval: true, workerData: { moduleUrl, view: start(), prev, cur, other, origin: { x: 12, y: 30 } } }
  );
  const result = await new Promise<{ view: View; anchor: { sx: number; sy: number } } | 'hung'>((resolve, reject) => {
    const timer = setTimeout(() => resolve('hung'), 5000);
    worker.once('message', (message) => {
      clearTimeout(timer);
      resolve(message);
    });
    worker.once('error', reject);
  });
  await worker.terminate();
  assert.notEqual(result, 'hung', 'the pinch returned');
  if (result === 'hung') return;
  assert.ok(Math.abs(step - 0.02) < 1e-3);
  assert.ok(Math.abs(result.view.rotation - (start().rotation + step)) < 1e-12, 'a small turn, not a full one');
  assert.ok(Math.abs(result.view.scale - start().scale) < 1e-12, 'equal finger spacing keeps the zoom');
  const midpoint = { sx: (cur.x + other.x) / 2 - 12, sy: (cur.y + other.y) / 2 - 30 };
  assert.ok(Math.abs(result.anchor.sx - midpoint.sx) < 1e-9 && Math.abs(result.anchor.sy - midpoint.sy) < 1e-9,
    'the cell under the fingers stays under them');
});

test('panning moves the view by the pointer delta', () => {
  const viewport = new CanvasViewport(start());
  viewport.pan(5, -3);
  assert.equal(viewport.view.tx, 45);
  assert.equal(viewport.view.ty, -28);
});

/** MapView.svelte's draw-time raster tracking as it stood. */
function legacyTracker(view: View) {
  let last: typeof raster | null = null;
  return {
    adopt(info: typeof raster | null, fleetCount: number): boolean {
      if (!view.initialised && info && fleetCount) {
        view.initialised = true;
        last = info;
        return true;
      } else if (view.initialised && last && info) {
        if (last !== info) {
          const pixelsPerMetre = view.scale / last.resolution;
          const dx = (last.origin.x - info.origin.x) * pixelsPerMetre;
          const dy = (info.height * info.resolution - last.height * last.resolution
            + info.origin.y - last.origin.y) * pixelsPerMetre;
          const c = Math.cos(view.rotation), s = Math.sin(view.rotation);
          view.scale = pixelsPerMetre * info.resolution;
          view.tx -= dx * c - dy * s;
          view.ty -= dx * s + dy * c;
        }
        last = info;
      } else if (info) {
        last = info;
      }
      return false;
    },
    forget() {
      last = null;
      view.initialised = false;
    }
  };
}

const raster = { resolution: 0.1, width: 100, height: 80, origin: { x: -2, y: -3 } };

test('the viewport initialises once for the first raster with robots and rebases later ones', () => {
  const sequence: [typeof raster | null, number, 'forget'?][] = [
    [null, 1],
    [raster, 0],
    [{ ...raster, height: 90 }, 0],
    [{ ...raster, height: 90 }, 2],
    [{ ...raster, height: 120, origin: { x: -2, y: -6 } }, 2],
    [{ ...raster, resolution: 0.05, height: 240, width: 200 }, 2],
    [null, 2, 'forget'],
    [raster, 2],
    [{ ...raster, width: 300 }, 2]
  ];
  const expected = { ...start(), initialised: false };
  const legacy = legacyTracker(expected);
  const viewport = new CanvasViewport({ ...start(), initialised: false });
  for (const [info, count, forget] of sequence) {
    if (forget) {
      legacy.forget();
      viewport.forgetRaster();
    }
    assert.equal(viewport.adoptRaster(info, count > 0), legacy.adopt(info, count));
    assert.equal(viewport.view.initialised, expected.initialised);
    close(viewport.view, expected);
  }
});
