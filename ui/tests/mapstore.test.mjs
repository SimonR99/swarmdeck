import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFile, mkdtemp, writeFile, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { pathToFileURL } from 'node:url';
import { deflateSync } from 'node:zlib';
import { build, transform } from 'esbuild';
import { compileModule } from 'svelte/compiler';

// Execute the real rune store, compiling it just as Vite does. Only the fleet
// and browser raster APIs are stubbed; fetch ordering is controlled by tests.
const bundle = await build({
  entryPoints: ['src/lib/stores/mapstore.svelte.ts'],
  absWorkingDir: new URL('..', import.meta.url).pathname,
  bundle: true, write: false, format: 'esm', platform: 'browser',
  plugins: [{ name: 'store-harness', setup(build) {
    build.onResolve({ filter: /^\$lib\/stores\/fleet.svelte$/ }, () => ({ path: 'fleet', namespace: 'stub' }));
    build.onLoad({ filter: /.*/, namespace: 'stub' }, () => ({ contents: 'export const fleet = { robots: [] };' }));
    build.onLoad({ filter: /mapstore.svelte.ts$/ }, async ({ path }) => {
      const js = await transform(await readFile(path, 'utf8'), { loader: 'ts' });
      return { contents: compileModule(js.code, { filename: path, generate: 'client' }).js.code };
    });
  } }]
});
const directory = await mkdtemp(join(tmpdir(), 'swarmdeck-mapstore-'));
let mapStore;
try {
  const path = join(directory, 'store.mjs');
  await writeFile(path, bundle.outputFiles[0].text);
  ({ mapStore } = await import(pathToFileURL(path).href));
} finally {
  await rm(directory, { recursive: true });
}
class TestCanvasContext {
  constructor(canvas) {
    this.canvas = canvas;
    this.fillStyle = 'rgb(0,0,0)';
  }

  clearRect(x, y, width, height) {
    this.#paint(x, y, width, height, [0, 0, 0, 0]);
  }

  fillRect(x, y, width, height) {
    const values = this.fillStyle.match(/\d+/g)?.map(Number) ?? [0, 0, 0];
    this.#paint(x, y, width, height, [...values.slice(0, 3), 255]);
  }

  putImageData(image, dx, dy) {
    this.#copy(image.data, image.width, image.height, dx, dy);
  }

  drawImage(source, dx, dy) {
    if (source instanceof TestCanvas) {
      this.#copy(source.pixels, source.width, source.height, dx, dy);
    }
  }

  getImageData(x, y, width, height) {
    const data = new Uint8ClampedArray(width * height * 4);
    for (let row = 0; row < height; row++) {
      for (let column = 0; column < width; column++) {
        const sx = x + column;
        const sy = y + row;
        if (sx < 0 || sy < 0 || sx >= this.canvas.width || sy >= this.canvas.height) continue;
        const source = (sy * this.canvas.width + sx) * 4;
        data.set(this.canvas.pixels.subarray(source, source + 4), (row * width + column) * 4);
      }
    }
    return { data };
  }

  #paint(x, y, width, height, rgba) {
    const data = new Uint8ClampedArray(width * height * 4);
    for (let index = 0; index < width * height; index++) data.set(rgba, index * 4);
    this.#copy(data, width, height, x, y);
  }

  #copy(source, width, height, dx, dy) {
    for (let row = 0; row < height; row++) {
      for (let column = 0; column < width; column++) {
        const tx = dx + column;
        const ty = dy + row;
        if (tx < 0 || ty < 0 || tx >= this.canvas.width || ty >= this.canvas.height) continue;
        const input = (row * width + column) * 4;
        const output = (ty * this.canvas.width + tx) * 4;
        this.canvas.pixels.set(source.subarray(input, input + 4), output);
      }
    }
  }
}

class TestCanvas {
  constructor() {
    this._width = 2;
    this._height = 2;
    this.pixels = new Uint8ClampedArray(2 * 2 * 4);
    this.context = new TestCanvasContext(this);
  }

  get width() { return this._width; }
  set width(value) { this._width = value; this.#resize(); }
  get height() { return this._height; }
  set height(value) { this._height = value; this.#resize(); }
  getContext() { return this.context; }
  #resize() { this.pixels = new Uint8ClampedArray(this._width * this._height * 4); }
}

globalThis.document = { createElement: () => new TestCanvas() };
globalThis.createImageBitmap = async () => ({ close() {} });
globalThis.ImageData = class {
  constructor(width, height) {
    this.width = width;
    this.height = height;
    this.data = new Uint8ClampedArray(width * height * 4);
  }
};
const info = { width: 2, height: 2, resolution: 0.1, origin: { x: 0, y: 0 }, seq: 1 };
const scope = { ...info, scope: 'robot:r1', robots: ['r1'] };
const UNKNOWN = [214, 218, 224, 255];
const FREE = [255, 255, 255, 255];
const OCCUPIED = [52, 58, 68, 255];
const pixel = (canvas, x, y) => Array.from(canvas.getContext('2d').getImageData(x, y, 1, 1).data);
const deferred = () => {
  let resolve;
  const promise = new Promise(r => { resolve = r; });
  return { promise, resolve };
};
function response(url) {
  if (url === '/api/map/optimized') return Response.json({ maps: [scope] });
  if (url.endsWith('/info')) return Response.json(info);
  if (url.includes('/network')) return new Response(null, { status: 404 });
  return new Response('png');
}
async function setup() {
  mapStore.reset();
  globalThis.fetch = async url => response(url);
  await mapStore.setViewPreference('local', 'r1');
  await mapStore.refreshLocalView();
  assert.equal(mapStore.ready, true);
  assert.equal(mapStore.showingOptimizedGrid, true);
}

test('graph refreshes retain the raster and coalesce concurrent notifications', async () => {
  await setup();
  const oldCanvas = mapStore.canvas;
  const oldInfo = mapStore.info;
  const index = deferred();
  const png = deferred();
  const started = deferred();
  let requests = 0;
  globalThis.fetch = async url => {
    requests++;
    if (url === '/api/map/optimized') return index.promise;
    if (url.startsWith('/api/map/optimized/')) { started.resolve(); return png.promise; }
    return response(url);
  };
  mapStore.applySlamGraph('r1', {});
  mapStore.applySlamGraph('r1', {});
  assert.equal(requests, 1);
  assert.equal(mapStore.ready, true);
  assert.equal(mapStore.canvas, oldCanvas);
  assert.equal(mapStore.info, oldInfo);
  index.resolve(response('/api/map/optimized'));
  await started.promise;
  assert.equal(mapStore.ready, true);
  assert.equal(mapStore.canvas, oldCanvas);
  assert.equal(mapStore.info, oldInfo);
  png.resolve(new Response('png'));
  await new Promise(resolve => setTimeout(resolve, 10));
  assert.equal(mapStore.ready, true);
});

test('a failed background refresh retains the map and its coordinate frame', async (t) => {
  await setup();
  t.mock.method(console, 'warn', () => {});
  const oldCanvas = mapStore.canvas;
  const oldInfo = mapStore.info;
  globalThis.fetch = async url => url === '/api/map/optimized'
    ? Response.json({ maps: [] }) : new Response(null, { status: 503 });
  await mapStore.refreshLocalView();
  assert.equal(mapStore.ready, true);
  assert.equal(mapStore.canvas, oldCanvas);
  assert.equal(mapStore.info, oldInfo);
  assert.equal(mapStore.showingOptimizedGrid, true);
});

test('a delayed graph index cannot reselect the previous robot', async () => {
  await setup();
  const index = deferred();
  globalThis.fetch = async url => url === '/api/map/optimized' ? index.promise : response(url);
  const refresh = mapStore.refreshLocalView();
  await mapStore.setViewPreference('local', 'r2');
  index.resolve(response('/api/map/optimized'));
  await refresh;
  assert.equal(mapStore.viewRobot, 'r2');
  assert.equal(mapStore.ready, true);
  mapStore.reset();
  assert.equal(mapStore.ready, false);
  assert.equal(mapStore.canvas, null);
});

for (const local of [true, false]) {
  test(`${local ? 'local' : 'global'} optimized image uses its own geometry when the index is stale`, async () => {
    await setup();
    const latest = { width: 5, height: 7, resolution: 0.05, origin: { x: -3, y: 9 } };
    globalThis.fetch = async url => {
      if (url === '/api/map/optimized') return Response.json({ maps: [
        { ...scope, scope: 'component:1', robots: ['r1', 'r2'] }, scope
      ] });
      if (url.startsWith('/api/map/optimized/')) return new Response('png', { headers: {
        'X-Map-Resolution': String(latest.resolution),
        'X-Map-Width': String(latest.width), 'X-Map-Height': String(latest.height),
        'X-Map-Origin-X': String(latest.origin.x), 'X-Map-Origin-Y': String(latest.origin.y)
      } });
      return response(url);
    };
    if (local) await mapStore.refreshLocalView();
    else {
      mapStore.setGlobalInfo(info);
      await mapStore.setViewPreference('global', null);
    }
    assert.equal(mapStore.info.width, latest.width);
    assert.equal(mapStore.info.height, latest.height);
    assert.equal(mapStore.info.resolution, latest.resolution);
    assert.deepEqual(mapStore.info.origin, latest.origin);
    assert.equal(mapStore.canvas.width, latest.width);
    assert.equal(mapStore.canvas.height, latest.height);
  });
}

test('an expanding live patch keeps the transform captured with its pixels', () => {
  mapStore.reset();
  mapStore.setFull({
    ...info,
    transforms: { r1: { x: 1, y: 2, yaw: 0 } }
  }, new Int8Array([100, 0, -1, 100]));
  const transforms = { r1: { x: 8, y: -3, yaw: 0.5 } };
  const data = deflateSync(new Int8Array([0])).toString('base64');
  mapStore.applyGlobalPatch({
    type: 'map_patch', seq: 2, resolution: 0.1,
    origin: { x: -0.1, y: -0.1 }, width: 3, height: 3,
    x0: 0, y0: 0, w: 1, h: 1, data, transforms
  });
  assert.deepEqual(mapStore.info.transforms, transforms);
  assert.equal(mapStore.info.seq, 2);
  assert.deepEqual(pixel(mapStore.canvas, 0, 2), FREE);
  assert.deepEqual(pixel(mapStore.canvas, 1, 0), UNKNOWN);
  assert.deepEqual(pixel(mapStore.canvas, 2, 0), OCCUPIED);
  assert.deepEqual(pixel(mapStore.canvas, 1, 1), OCCUPIED);
  assert.deepEqual(pixel(mapStore.canvas, 2, 1), FREE);
});

test('an older patch cannot regress pixels or their transform, while a duplicate can repaint', () => {
  mapStore.reset();
  mapStore.setFull({ ...info, seq: 5 }, new Int8Array(4));
  const currentTransforms = { r1: { x: 5, y: 6, yaw: 0.2 } };
  mapStore.applyGlobalPatch({
    type: 'map_patch', seq: 6, resolution: 0.1, origin: info.origin,
    width: 2, height: 2, x0: 0, y0: 0, w: 1, h: 1,
    data: deflateSync(new Int8Array([100])).toString('base64'),
    transforms: currentTransforms
  });
  const revision = mapStore.revision;
  assert.deepEqual(pixel(mapStore.canvas, 0, 1), OCCUPIED);

  mapStore.applyGlobalPatch({
    type: 'map_patch', seq: 5, resolution: 0.1, origin: info.origin,
    width: 2, height: 2, x0: 0, y0: 0, w: 1, h: 1,
    data: deflateSync(new Int8Array([0])).toString('base64'),
    transforms: { r1: { x: -8, y: -9, yaw: -0.5 } }
  });
  assert.equal(mapStore.revision, revision);
  assert.equal(mapStore.seq, 6);
  assert.equal(mapStore.info.seq, 6);
  assert.deepEqual(mapStore.info.transforms, currentTransforms);
  assert.deepEqual(pixel(mapStore.canvas, 0, 1), OCCUPIED);

  mapStore.applyGlobalPatch({
    type: 'map_patch', seq: 6, resolution: 0.1, origin: info.origin,
    width: 2, height: 2, x0: 0, y0: 0, w: 1, h: 1,
    data: deflateSync(new Int8Array([0])).toString('base64'),
    transforms: currentTransforms
  });
  assert.deepEqual(pixel(mapStore.canvas, 0, 1), FREE);
});

test('malformed patches leave sequence, metadata, dimensions, and pixels unchanged', (t) => {
  mapStore.reset();
  const transforms = { r1: { x: 1, y: 2, yaw: 0 } };
  mapStore.setFull({ ...info, seq: 3, transforms }, new Int8Array([100, 0, -1, 100]));
  t.mock.method(console, 'warn', () => {});
  const canvas = mapStore.canvas;
  const before = Array.from(canvas.pixels);
  const revision = mapStore.revision;

  for (const data of [
    'not-valid-zlib',
    deflateSync(new Int8Array([0])).toString('base64')
  ]) {
    mapStore.applyGlobalPatch({
      type: 'map_patch', seq: 4, resolution: 0.1,
      origin: { x: -0.1, y: -0.1 }, width: 3, height: 3,
      x0: 0, y0: 0, w: 2, h: 2, data,
      transforms: { r1: { x: 9, y: 9, yaw: 1 } }
    });
  }

  assert.equal(mapStore.canvas, canvas);
  assert.equal(canvas.width, 2);
  assert.equal(canvas.height, 2);
  assert.equal(mapStore.seq, 3);
  assert.equal(mapStore.info.seq, 3);
  assert.equal(mapStore.revision, revision);
  assert.deepEqual(mapStore.info.transforms, transforms);
  assert.deepEqual(Array.from(canvas.pixels), before);
});
