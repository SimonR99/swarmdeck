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
  // Outward CCW faces. Mesh contains only the boundary of observed occupied cells.
  const faces = [
    {
      n: [1, 0, 0],
      v: [
        [1, 0, 0],
        [1, 1, 0],
        [1, 1, 1],
        [1, 0, 1]
      ]
    },
    {
      n: [-1, 0, 0],
      v: [
        [0, 1, 0],
        [0, 0, 0],
        [0, 0, 1],
        [0, 1, 1]
      ]
    },
    {
      n: [0, 1, 0],
      v: [
        [1, 1, 0],
        [0, 1, 0],
        [0, 1, 1],
        [1, 1, 1]
      ]
    },
    {
      n: [0, -1, 0],
      v: [
        [0, 0, 0],
        [1, 0, 0],
        [1, 0, 1],
        [0, 0, 1]
      ]
    },
    {
      n: [0, 0, 1],
      v: [
        [0, 0, 1],
        [1, 0, 1],
        [1, 1, 1],
        [0, 1, 1]
      ]
    },
    {
      n: [0, 0, -1],
      v: [
        [0, 1, 0],
        [1, 1, 0],
        [1, 0, 0],
        [0, 0, 0]
      ]
    }
  ];
  for (const cell of cells.values()) {
    centers.set(
      [(cell.x + 0.5) * size, (cell.y + 0.5) * size, (cell.z + 0.5) * size],
      cellIndex * 3
    );
    samples[cellIndex++] = cell.point;
    for (const f of faces) {
      if (cells.has(`${cell.x + f.n[0]},${cell.y + f.n[1]},${cell.z + f.n[2]}`)) continue;
      for (const corner of [0, 1, 2, 0, 2, 3]) {
        const v = f.v[corner];
        vertices.push((cell.x + v[0]) * size, (cell.y + v[1]) * size, (cell.z + v[2]) * size);
        normals.push(...f.n);
        meshSamples.push(cell.point);
      }
    }
  }
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
