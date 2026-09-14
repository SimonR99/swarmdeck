/** Pure, bounded map preparation; also runs in the terrain worker. Coordinates are Z-up metres. */
export const QUALITY = {
  low: { points: 60000, voxels: 8000, splats: 30000, dpr: 1, fps: 30 },
  balanced: { points: 160000, voxels: 20000, splats: 80000, dpr: 1.5, fps: 45 },
  high: { points: 300000, voxels: 40000, splats: 150000, dpr: 2, fps: 60 }
} as const;
export type Quality = keyof typeof QUALITY;
export interface CloudInput {
  positions: Float32Array;
  owners: Uint8Array;
  rgb?: Uint8Array;
  quality: Quality;
}
export function prepareTerrain(input: CloudInput) {
  const { positions, owners, rgb, quality } = input;
  const budget = QUALITY[quality];
  const valid: number[] = [];
  for (let i = 0; i < positions.length / 3; i++) {
    if ([positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]].every(Number.isFinite))
      valid.push(i);
  }
  const n = Math.min(valid.length, budget.points);
  const xyz = new Float32Array(n * 3),
    source = new Uint8Array(n);
  const colors = rgb ? new Uint8Array(n * 3) : undefined;
  for (let j = 0; j < n; j++) {
    const i = valid[Math.floor((j * valid.length) / n)];
    xyz.set(positions.subarray(i * 3, i * 3 + 3), j * 3);
    source[j] = owners[i];
    if (colors && rgb) colors.set(rgb.subarray(i * 3, i * 3 + 3), j * 3);
  }
  // Coarsen the whole map instead of dropping arbitrary blocks and opening holes.
  let size = 0.15;
  let cells = new Map<string, { x: number; y: number; z: number; point: number }>();
  do {
    cells = new Map();
    for (let i = 0; i < n; i++) {
      const x = Math.floor(xyz[i * 3] / size),
        y = Math.floor(xyz[i * 3 + 1] / size),
        z = Math.floor(xyz[i * 3 + 2] / size);
      const key = `${x},${y},${z}`;
      if (!cells.has(key)) cells.set(key, { x, y, z, point: i });
    }
    if (cells.size <= budget.voxels) break;
    size *= 1.5;
  } while (true);
  const centers = new Float32Array(cells.size * 3),
    samples = new Uint32Array(cells.size);
  const vertices: number[] = [],
    normals: number[] = [],
    meshSamples: number[] = [];
  let cellIndex = 0;
  for (const cell of cells.values()) {
    centers.set(
      [(cell.x + 0.5) * size, (cell.y + 0.5) * size, (cell.z + 0.5) * size],
      cellIndex * 3
    );
    samples[cellIndex++] = cell.point;
  }

  const addTriangle = (...triangle: { x: number; y: number; z: number; point: number }[]) => {
    const [a, b, c] = triangle;
    const ax = b.x - a.x, ay = b.y - a.y, az = b.z - a.z;
    const bx = c.x - a.x, by = c.y - a.y, bz = c.z - a.z;
    let nx = ay * bz - az * by, ny = az * bx - ax * bz, nz = ax * by - ay * bx;
    if (nz < 0) return addTriangle(a, c, b);
    const length = Math.hypot(nx, ny, nz) || 1;
    nx /= length; ny /= length; nz /= length;
    for (const p of triangle) {
      vertices.push((p.x + 0.5) * size, (p.y + 0.5) * size, (p.z + 0.5) * size);
      normals.push(nx, ny, nz);
      meshSamples.push(p.point);
    }
  };
  const connected = (axis: 'x' | 'y' | 'z', ...points: ({ x: number; y: number; z: number } | undefined)[]) => {
    if (points.some((point) => !point)) return false;
    const offsets = points.map((point) => point![axis]);
    // Do not bridge disconnected surfaces across an empty voxel band.
    return Math.max(...offsets) - Math.min(...offsets) <= 2;
  };
  type Cell = { x: number; y: number; z: number; point: number };
  const triangulateProjection = (
    u: 'x' | 'y' | 'z', v: 'x' | 'y' | 'z', depth: 'x' | 'y' | 'z'
  ) => {
    const columns = new Map<string, { low: Cell; high: Cell }>();
    for (const cell of cells.values()) {
      const key = `${cell[u]},${cell[v]}`;
      const column = columns.get(key);
      if (!column) columns.set(key, { low: cell, high: cell });
      else {
        if (cell[depth] < column.low[depth]) column.low = cell;
        if (cell[depth] > column.high[depth]) column.high = cell;
      }
    }
    for (const side of ['low', 'high'] as const) {
      for (const column of columns.values()) {
        const cell = column[side];
        // A one-voxel-thick surface is represented once, rather than drawing
        // coincident min/max triangles and doubling its opacity.
        if (side === 'high' && column.high === column.low) continue;
        const east = columns.get(`${cell[u] + 1},${cell[v]}`)?.[side];
        const north = columns.get(`${cell[u]},${cell[v] + 1}`)?.[side];
        const diagonal = columns.get(`${cell[u] + 1},${cell[v] + 1}`)?.[side];
        if (connected(depth, cell, east, north)) addTriangle(cell, east!, north!);
        if (connected(depth, east, diagonal, north)) addTriangle(east!, diagonal!, north!);
      }
    }
  };
  // Three orthogonal projections retain floors/ceilings, both sides of walls,
  // and overhang silhouettes. Each surface only joins immediate neighbors and
  // may vary by at most two voxels in depth, bounding false cross-room joins.
  triangulateProjection('x', 'y', 'z');
  triangulateProjection('x', 'z', 'y');
  triangulateProjection('y', 'z', 'x');
  return {
    xyz,
    owners: source,
    rgb: colors,
    centers,
    samples,
    size,
    meshPositions: new Float32Array(vertices),
    meshNormals: new Float32Array(normals),
    meshSamples: new Uint32Array(meshSamples),
    total: valid.length
  };
}
export type TerrainData = ReturnType<typeof prepareTerrain>;
