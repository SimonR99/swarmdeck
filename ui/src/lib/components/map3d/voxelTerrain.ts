import * as THREE from 'three';
import type { CloudBounds } from './types';

export interface VoxelData {
  x: number;
  y: number;
  z: number;
  color: THREE.Color;
  ownerIndex: number;
  isCeiling: boolean;
  isWall: boolean;
  isFloor: boolean;
}

export class VoxelTerrain {
  public group = new THREE.Group();
  public instancedMesh: THREE.InstancedMesh | null = null;
  public ceilingClipPlane: THREE.Plane;
  public bounds: CloudBounds = {
    minX: -20,
    maxX: 20,
    minY: -20,
    maxY: 20,
    minZ: 0,
    maxZ: 3.0
  };
  public voxelSize = 0.14;
  public voxelCount = 0;
  private currentCutoff = 3.2;

  // Starcraft-style tactical palette
  private wallBaseColor = new THREE.Color('#465166');
  private floorBaseColor = new THREE.Color('#1e232c');
  private ceilingBaseColor = new THREE.Color('#2d3442');
  private obstacleBaseColor = new THREE.Color('#384252');

  constructor() {
    // Normal (0, 0, -1) with constant `currentCutoff` clips points where dot((x,y,z), (0,0,-1)) + cutoff < 0,
    // which means -z + cutoff < 0 => z > cutoff gets clipped!
    this.ceilingClipPlane = new THREE.Plane(new THREE.Vector3(0, 0, -1), this.currentCutoff);
  }

  public setCeilingCutoff(cutoffZ: number) {
    this.currentCutoff = cutoffZ;
    this.ceilingClipPlane.constant = cutoffZ;
  }

  /**
   * Build the 3D voxel terrain using the real 3D map points from the robots.
   */
  public buildFromPoints(
    positions: Float32Array,
    total: number,
    owners: Uint8Array,
    robotColors: string[]
  ): CloudBounds {
    this.clear();

    if (total === 0) {
      return this.buildMock3DEnvironment();
    }

    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;

    // 1. First pass: find real 3D bounds
    for (let i = 0; i < total; i++) {
      const x = positions[i * 3];
      const y = positions[i * 3 + 1];
      const z = positions[i * 3 + 2];
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
      if (z < minZ) minZ = z;
      if (z > maxZ) maxZ = z;
    }

    // Guard against degenerate bounds
    if (minZ === Infinity) {
      minX = -10; maxX = 10;
      minY = -10; maxY = 10;
      minZ = 0; maxZ = 2.8;
    }

    this.bounds = { minX, maxX, minY, maxY, minZ, maxZ };

    // 2. Voxel quantization: bin real points into discrete 3D cells
    const voxelMap = new Map<string, {
      x: number;
      y: number;
      z: number;
      ownerCounts: Map<number, number>;
      totalPoints: number;
    }>();

    const V = this.voxelSize;
    for (let i = 0; i < total; i++) {
      const x = positions[i * 3];
      const y = positions[i * 3 + 1];
      const z = positions[i * 3 + 2];
      const owner = owners[i] ?? 0;

      const ix = Math.floor(x / V);
      const iy = Math.floor(y / V);
      const iz = Math.floor(z / V);
      const key = `${ix},${iy},${iz}`;

      let cell = voxelMap.get(key);
      if (!cell) {
        cell = {
          x: (ix + 0.5) * V,
          y: (iy + 0.5) * V,
          z: (iz + 0.5) * V,
          ownerCounts: new Map(),
          totalPoints: 0
        };
        voxelMap.set(key, cell);
      }
      cell.ownerCounts.set(owner, (cell.ownerCounts.get(owner) ?? 0) + 1);
      cell.totalPoints++;
    }

    // 3. Assemble VoxelData with StarCraft styling
    const ceilingThreshold = maxZ - 0.35;
    const floorThreshold = minZ + 0.25;

    const voxels: VoxelData[] = [];
    const colorPalette = robotColors.map((c) => new THREE.Color(c));

    for (const cell of voxelMap.values()) {
      // Determine predominant robot owner
      let bestOwner = 0;
      let bestCount = -1;
      for (const [o, count] of cell.ownerCounts.entries()) {
        if (count > bestCount) {
          bestCount = count;
          bestOwner = o;
        }
      }

      const isCeiling = cell.z >= ceilingThreshold && maxZ - minZ > 1.2;
      const isFloor = cell.z <= floorThreshold;
      const isWall = !isCeiling && !isFloor && cell.z >= 0.8;

      let color = new THREE.Color();
      if (isCeiling) {
        color.copy(this.ceilingBaseColor);
      } else if (isFloor) {
        color.copy(this.floorBaseColor);
      } else if (isWall) {
        color.copy(this.wallBaseColor);
        // Subtle team accent along wall edges
        const teamColor = colorPalette[bestOwner];
        if (teamColor) {
          color.lerp(teamColor, 0.22);
        }
      } else {
        color.copy(this.obstacleBaseColor);
        const teamColor = colorPalette[bestOwner];
        if (teamColor) {
          color.lerp(teamColor, 0.25);
        }
      }

      voxels.push({
        x: cell.x,
        y: cell.y,
        z: cell.z,
        color,
        ownerIndex: bestOwner,
        isCeiling,
        isWall,
        isFloor
      });
    }

    this.createInstancedVoxels(voxels);
    return this.bounds;
  }

  /**
   * Generates a rich mock 3D facility (with real walls, doors, pillars, obstacles, and ceiling)
   * when no live 3D cloud has been published or in mock mode.
   */
  public buildMock3DEnvironment(): CloudBounds {
    this.clear();

    const minX = -18, maxX = 18;
    const minY = -18, maxY = 18;
    const minZ = 0.0, maxZ = 2.8;
    this.bounds = { minX, maxX, minY, maxY, minZ, maxZ };

    const voxels: VoxelData[] = [];
    const V = this.voxelSize;

    const addVoxel = (x: number, y: number, z: number, color: THREE.Color, isCeil = false, isWall = false) => {
      voxels.push({
        x,
        y,
        z,
        color,
        ownerIndex: 0,
        isCeiling: isCeil,
        isWall,
        isFloor: z <= 0.15
      });
    };

    // 1. Perimeter walls (height 0 to 2.8m)
    const wallColor = new THREE.Color('#444e61');
    const accentColor = new THREE.Color('#3870a0');

    const makeWall = (x0: number, y0: number, x1: number, y1: number, zMax = 2.8, hasDoor = false) => {
      const length = Math.hypot(x1 - x0, y1 - y0);
      const steps = Math.max(1, Math.round(length / V));
      const zSteps = Math.round(zMax / V);

      for (let i = 0; i <= steps; i++) {
        const t = i / steps;
        // Leave doorway in middle
        if (hasDoor && t > 0.42 && t < 0.58) continue;
        const wx = x0 + (x1 - x0) * t;
        const wy = y0 + (y1 - y0) * t;

        for (let zi = 0; zi <= zSteps; zi++) {
          const wz = zi * V;
          const col = (zi % 6 === 0 || i % 8 === 0) ? accentColor : wallColor;
          addVoxel(wx, wy, wz, col, false, true);
        }
      }
    };

    // Outer perimeter
    makeWall(-16, -16, 16, -16, 2.8, true);
    makeWall(16, -16, 16, 16, 2.8, true);
    makeWall(16, 16, -16, 16, 2.8, true);
    makeWall(-16, 16, -16, -16, 2.8, true);

    // Interior partitioning rooms
    makeWall(-6, -16, -6, 2, 2.6, true);
    makeWall(6, -16, 6, 2, 2.6, true);
    makeWall(-16, 2, 16, 2, 2.6, true);
    makeWall(0, 2, 0, 16, 2.6, true);

    // Pillars / Structural columns
    const pillarColor = new THREE.Color('#55627a');
    const pillarCoords = [
      [-10, -8], [10, -8], [-10, 8], [10, 8],
      [-3, -4], [3, -4], [-3, 8], [3, 8]
    ];
    for (const [px, py] of pillarCoords) {
      for (let dx = -0.15; dx <= 0.15; dx += V) {
        for (let dy = -0.15; dy <= 0.15; dy += V) {
          for (let pz = 0; pz <= 2.8; pz += V) {
            addVoxel(px + dx, py + dy, pz, pillarColor, false, true);
          }
        }
      }
    }

    // Crates / Sci-fi obstacles inside rooms
    const crateColor = new THREE.Color('#354050');
    const crateLocations = [
      [-12, -12], [-11, -12], [-12, -11],
      [12, -12], [11, -12],
      [-12, 12], [12, 12], [8, 10], [-4, 6]
    ];
    for (const [cx, cy] of crateLocations) {
      for (let cz = 0; cz <= 0.9; cz += V) {
        addVoxel(cx, cy, cz, crateColor, false, false);
      }
    }

    // Solid Ceiling slab at z = 2.8m across the facility
    const ceilingColor = new THREE.Color('#2d3440');
    for (let cx = -16; cx <= 16; cx += V * 2) {
      for (let cy = -16; cy <= 16; cy += V * 2) {
        // Grid pattern ceiling panels
        addVoxel(cx, cy, 2.8, ceilingColor, true, false);
      }
    }

    this.createInstancedVoxels(voxels);
    return this.bounds;
  }

  private createInstancedVoxels(voxels: VoxelData[]) {
    this.voxelCount = voxels.length;
    if (this.voxelCount === 0) return;

    const V = this.voxelSize;
    // Box geometry with small gap between voxels gives crisp tactical bevel lines
    const geom = new THREE.BoxGeometry(V * 0.92, V * 0.92, V * 0.92);

    const mat = new THREE.MeshStandardMaterial({
      roughness: 0.42,
      metalness: 0.32,
      clippingPlanes: [this.ceilingClipPlane],
      clipShadows: true,
      shadowSide: THREE.DoubleSide
    });

    this.instancedMesh = new THREE.InstancedMesh(geom, mat, this.voxelCount);
    this.instancedMesh.castShadow = true;
    this.instancedMesh.receiveShadow = true;

    const dummy = new THREE.Object3D();
    for (let i = 0; i < voxels.length; i++) {
      const v = voxels[i];
      dummy.position.set(v.x, v.y, v.z);
      dummy.rotation.set(0, 0, 0);
      dummy.scale.set(1, 1, 1);
      dummy.updateMatrix();
      this.instancedMesh.setMatrixAt(i, dummy.matrix);
      this.instancedMesh.setColorAt(i, v.color);
    }

    this.instancedMesh.instanceMatrix.needsUpdate = true;
    if (this.instancedMesh.instanceColor) {
      this.instancedMesh.instanceColor.needsUpdate = true;
    }

    this.group.add(this.instancedMesh);
  }

  public clear() {
    if (this.instancedMesh) {
      this.group.remove(this.instancedMesh);
      this.instancedMesh.geometry.dispose();
      if (Array.isArray(this.instancedMesh.material)) {
        this.instancedMesh.material.forEach((m) => m.dispose());
      } else {
        this.instancedMesh.material.dispose();
      }
      this.instancedMesh = null;
    }
    this.voxelCount = 0;
  }

  public dispose() {
    this.clear();
  }
}
