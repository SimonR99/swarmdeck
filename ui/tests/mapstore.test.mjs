import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFile, mkdtemp, writeFile, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { pathToFileURL } from 'node:url';
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
const context = {
  clearRect() {}, drawImage() {},
  getImageData: () => ({ data: new Uint8ClampedArray(16) })
};
globalThis.document = { createElement: () => ({ width: 2, height: 2, getContext: () => context }) };
globalThis.createImageBitmap = async () => ({ close() {} });
const info = { width: 2, height: 2, resolution: 0.1, origin: { x: 0, y: 0 }, seq: 1 };
const scope = { ...info, scope: 'robot:r1', robots: ['r1'] };
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
