// Transfer only sort indices; the model stays resident in this worker.
let centers: Float32Array = new Float32Array();
self.onmessage = (event: MessageEvent<{ centers?: Float32Array; view?: number[]; id: number }>) => {
  if (event.data.centers) centers = event.data.centers;
  if (!event.data.view) return;
  const m = event.data.view,
    n = centers.length / 3;
  const depth = new Float32Array(n);
  for (let i = 0; i < n; i++)
    depth[i] = m[2] * centers[i * 3] + m[6] * centers[i * 3 + 1] + m[10] * centers[i * 3 + 2];
  const order = Uint32Array.from({ length: n }, (_, i) => i);
  order.sort((a, b) => depth[a] - depth[b]); // camera-space far to near
  self.postMessage({ id: event.data.id, order }, { transfer: [order.buffer] });
};
