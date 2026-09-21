export interface RasterSnapshotInfo {
  resolution: number;
  width: number;
  height: number;
  origin: { x: number; y: number };
  seq: number;
  transforms?: Record<string, { x: number; y: number; yaw: number }>;
}

/** Parse geometry and transform provenance from an optimized raster response. */
export function mapSnapshotInfo(headers: Headers, fallback: RasterSnapshotInfo): RasterSnapshotInfo {
  const value = (name: string, previous: number) => {
    const raw = headers.get(name);
    if (raw === null) return previous;
    const parsed = Number(raw);
    if (!Number.isFinite(parsed)) throw new Error(`Invalid ${name}`);
    return parsed;
  };
  let transforms: RasterSnapshotInfo['transforms'] = undefined;
  const encoded = headers.get('X-Map-Transforms');
  if (encoded !== null) {
    const parsed = JSON.parse(encoded) as unknown;
    if (!parsed || typeof parsed !== 'object' || Array.isArray(parsed)) throw new Error('Invalid X-Map-Transforms');
    transforms = parsed as RasterSnapshotInfo['transforms'];
  }
  return {
    resolution: value('X-Map-Resolution', fallback.resolution),
    width: value('X-Map-Width', fallback.width),
    height: value('X-Map-Height', fallback.height),
    origin: {
      x: value('X-Map-Origin-X', fallback.origin.x),
      y: value('X-Map-Origin-Y', fallback.origin.y)
    },
    seq: value('X-Map-Seq', fallback.seq),
    transforms
  };
}
