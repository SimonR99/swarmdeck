import type { MapInfo } from '../types/protocol';

/** Image response metadata takes precedence over a potentially stale index. */
export function mapSnapshotInfo(headers: Headers, fallback: MapInfo): MapInfo {
  if (!headers.has('X-Map-Resolution')) return fallback; // Older server compatibility.
  const value = (name: string) => headers.has(name) ? Number(headers.get(name)) : NaN;
  const resolution = value('X-Map-Resolution');
  const width = value('X-Map-Width'), height = value('X-Map-Height');
  const x = value('X-Map-Origin-X'), y = value('X-Map-Origin-Y');
  if (!Number.isFinite(resolution) || resolution <= 0
    || !Number.isInteger(width) || width <= 0 || !Number.isInteger(height) || height <= 0
    || !Number.isFinite(x) || !Number.isFinite(y)) throw new Error('Invalid map snapshot geometry');
  return { ...fallback, resolution, width, height, origin: { x, y } };
}
