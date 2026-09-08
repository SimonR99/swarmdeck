import { prepareTerrain, type CloudInput } from './terrainData';
self.onmessage = (event: MessageEvent<CloudInput & { id: number }>) => {
  try {
    const data = prepareTerrain(event.data);
    const transfer = Object.values(data)
      .filter((v) => ArrayBuffer.isView(v))
      .map((v) => (v as ArrayBufferView).buffer as ArrayBuffer);
    self.postMessage({ id: event.data.id, data }, { transfer });
  } catch (error) {
    self.postMessage({ id: event.data.id, error: String(error) });
  }
};
