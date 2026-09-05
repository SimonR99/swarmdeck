import * as THREE from 'three';
import type { CloudBounds } from './types';
import type { MapInfo } from '$lib/types/protocol';

export interface VoxelPoint {
  x: number;
  y: number;
  z: number;
  color?: THREE.Color;
  isWall?: boolean;
  isCeiling?: boolean;
}

export class VoxelTerrain {
  public group = new THREE.Group();
  public wallMesh: THREE.InstancedMesh | null = null;
  public wallCapMesh: THREE.InstancedMesh | null = null;
  public floorMesh: THREE.Mesh | null = null;
  public ceilingMesh: THREE.Mesh | null = null;
  public cloudMesh: THREE.InstancedMesh | null = null;

  public ceilingClipPlane: THREE.Plane;
  public bounds: CloudBounds = {
    minX: -16,
    maxX: 16,
    minY: -16,
    maxY: 16,
    minZ: 0,
    maxZ: 2.6
  };
  public voxelSize = 0.15; // 15cm blocks match the real 15cm wall thickness
  public voxelCount = 0;
  private currentCutoff = 2.3; // Default: interior rooms open

  // High-contrast, bright tactical palette
  private wallColor = new THREE.Color('#94a3b8'); // Bright titanium/slate (Slate 400)
  private wallCapColor = new THREE.Color('#f8fafc'); // Pure platinum/white top edge highlight
  private ceilingColor = new THREE.Color('#8293a7');

  private floorCanvas: HTMLCanvasElement | null = null;
  private floorTexture: THREE.CanvasTexture | null = null;

  constructor() {
    // Normal (0, 0, -1) with constant cutoff clips geometry where z > cutoff
    this.ceilingClipPlane = new THREE.Plane(new THREE.Vector3(0, 0, -1), this.currentCutoff);
  }

  public setCeilingCutoff(cutoffZ: number) {
    this.currentCutoff = cutoffZ;
    this.ceilingClipPlane.constant = cutoffZ;
  }

  /**
   * Build 3D walls, floor, and ceiling directly from the robots' real map.
   */
  public buildFromMapGrid(
    canvas: HTMLCanvasElement | null,
    info: MapInfo,
    occupiedArray?: Uint8Array | null
  ): CloudBounds {
    this.clear();

    const width = info.width;
    const height = info.height;
    const res = info.resolution;
    const origX = info.origin.x;
    const origY = info.origin.y;

    // 1. Recover occupancy data if not provided
    let occupied: Uint8Array | null = occupiedArray ?? null;
    if (!occupied && canvas) {
      const ctx = canvas.getContext('2d');
      if (ctx) {
        const imgData = ctx.getImageData(0, 0, width, height).data;
        occupied = new Uint8Array(width * height);
        for (let i = 0; i < occupied.length; i++) {
          occupied[i] = imgData[i * 4] < 100 ? 1 : 0;
        }
      }
    }

    if (!occupied) {
      return this.bounds;
    }

    // 2. Downsample to 0.15m voxels (grouping cells to match 15cm wall thickness)
    const step = Math.max(1, Math.round(this.voxelSize / res)); // 3 cells at 0.05m res
    const actualVoxelSize = step * res;
    const wallPositions: { x: number; y: number }[] = [];

    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;

    for (let gy = 0; gy < height - step + 1; gy += step) {
      for (let gx = 0; gx < width - step + 1; gx += step) {
        let isOcc = false;
        for (let dy = 0; dy < step; dy++) {
          const row = (gy + dy) * width;
          for (let dx = 0; dx < step; dx++) {
            if (occupied[row + gx + dx] === 1) {
              isOcc = true;
              break;
            }
          }
          if (isOcc) break;
        }

        if (isOcc) {
          const wx = origX + (gx + step / 2) * res;
          const wy = origY + (height - 1 - (gy + step / 2)) * res;
          wallPositions.push({ x: wx, y: wy });

          if (wx < minX) minX = wx;
          if (wx > maxX) maxX = wx;
          if (wy < minY) minY = wy;
          if (wy > maxY) maxY = wy;
        }
      }
    }

    if (wallPositions.length === 0 || minX === Infinity) {
      minX = origX;
      maxX = origX + width * res;
      minY = origY;
      maxY = origY + height * res;
    }

    this.bounds = {
      minX: minX - 1.0,
      maxX: maxX + 1.0,
      minY: minY - 1.0,
      maxY: maxY + 1.0,
      minZ: 0,
      maxZ: 2.6
    };

    // 3. Build 3D Walls (height 2.4m, centered at z = 1.2)
    const wallHeight = 2.4;
    const wallBox = new THREE.BoxGeometry(
      actualVoxelSize * 0.98,
      actualVoxelSize * 0.98,
      wallHeight
    );

    const wallMat = new THREE.MeshStandardMaterial({
      color: this.wallColor,
      roughness: 0.35,
      metalness: 0.25,
      clippingPlanes: [this.ceilingClipPlane],
      clipShadows: true,
      shadowSide: THREE.DoubleSide
    });

    this.wallMesh = new THREE.InstancedMesh(wallBox, wallMat, wallPositions.length);
    this.wallMesh.castShadow = true;
    this.wallMesh.receiveShadow = true;

    // Top cap highlight: bright platinum border at wall top
    const capBox = new THREE.BoxGeometry(
      actualVoxelSize * 1.02,
      actualVoxelSize * 1.02,
      0.08
    );
    const capMat = new THREE.MeshStandardMaterial({
      color: this.wallCapColor,
      emissive: new THREE.Color('#334155'),
      roughness: 0.2,
      metalness: 0.4,
      clippingPlanes: [this.ceilingClipPlane]
    });
    this.wallCapMesh = new THREE.InstancedMesh(capBox, capMat, wallPositions.length);

    const dummy = new THREE.Object3D();
    for (let i = 0; i < wallPositions.length; i++) {
      const pos = wallPositions[i];
      // Wall column from z=0 to z=wallHeight
      dummy.position.set(pos.x, pos.y, wallHeight / 2);
      dummy.updateMatrix();
      this.wallMesh.setMatrixAt(i, dummy.matrix);

      // Cap at top of wall
      dummy.position.set(pos.x, pos.y, wallHeight + 0.04);
      dummy.updateMatrix();
      this.wallCapMesh.setMatrixAt(i, dummy.matrix);
    }

    this.wallMesh.instanceMatrix.needsUpdate = true;
    this.wallCapMesh.instanceMatrix.needsUpdate = true;

    this.group.add(this.wallMesh);
    this.group.add(this.wallCapMesh);

    // 4. Build Tactical Floor from Real Map Canvas
    this.buildTacticalFloor(canvas, info);

    // 5. Build Ceiling Slab over Explored Facility
    this.buildCeiling(minX - 0.5, maxX + 0.5, minY - 0.5, maxY + 0.5, wallHeight);

    this.voxelCount = wallPositions.length;
    return this.bounds;
  }

  /**
   * Build the bright tactical floor using the robot's real explored map texture.
   */
  private buildTacticalFloor(canvas: HTMLCanvasElement | null, info: MapInfo) {
    if (this.floorMesh) {
      this.group.remove(this.floorMesh);
      this.floorMesh.geometry.dispose();
      (this.floorMesh.material as THREE.Material).dispose();
      this.floorMesh = null;
    }

    const widthM = info.width * info.resolution;
    const heightM = info.height * info.resolution;
    const centerX = info.origin.x + widthM / 2;
    const centerY = info.origin.y + heightM / 2;

    const floorGeom = new THREE.PlaneGeometry(widthM, heightM);

    // Create styled high-contrast canvas texture
    if (canvas) {
      if (!this.floorCanvas) {
        this.floorCanvas = document.createElement('canvas');
      }
      this.floorCanvas.width = canvas.width;
      this.floorCanvas.height = canvas.height;
      const fCtx = this.floorCanvas.getContext('2d');
      if (fCtx) {
        // Render styled bright tactical floor:
        // free space (white) -> bright clean tactical tile (#b8c7d9)
        // occupied space (dark) -> wall base (#475569)
        // unknown space (grey) -> command background (#22272e)
        const srcData = canvas.getContext('2d')?.getImageData(0, 0, canvas.width, canvas.height);
        if (srcData) {
          const outImg = fCtx.createImageData(canvas.width, canvas.height);
          const s = srcData.data;
          const d = outImg.data;
          for (let i = 0; i < s.length; i += 4) {
            const r = s[i];
            if (r < 100) {
              // Occupied wall base: dark slate
              d[i] = 71;
              d[i + 1] = 85;
              d[i + 2] = 105;
              d[i + 3] = 255;
            } else if (r > 240) {
              // Explored free space: bright clean high-contrast floor
              // Subtle tile seam pattern every 20 pixels (1 metre)
              const pixelX = (i / 4) % canvas.width;
              const pixelY = Math.floor((i / 4) / canvas.width);
              const isSeam = pixelX % 20 === 0 || pixelY % 20 === 0;

              if (isSeam) {
                d[i] = 148;
                d[i + 1] = 163;
                d[i + 2] = 184;
              } else {
                d[i] = 196;
                d[i + 1] = 207;
                d[i + 2] = 222;
              }
              d[i + 3] = 255;
            } else {
              // Unknown/unexplored space: clean subtle dark command floor
              d[i] = 34;
              d[i + 1] = 39;
              d[i + 2] = 46;
              d[i + 3] = 255;
            }
          }
          fCtx.putImageData(outImg, 0, 0);
        }
      }

      if (this.floorTexture) {
        this.floorTexture.dispose();
      }
      this.floorTexture = new THREE.CanvasTexture(this.floorCanvas);
      this.floorTexture.magFilter = THREE.NearestFilter;
      this.floorTexture.minFilter = THREE.LinearFilter;
    }

    const floorMat = new THREE.MeshStandardMaterial({
      map: this.floorTexture ?? undefined,
      color: this.floorTexture ? 0xffffff : 0x22272e,
      roughness: 0.6,
      metalness: 0.15
    });

    this.floorMesh = new THREE.Mesh(floorGeom, floorMat);
    this.floorMesh.position.set(centerX, centerY, 0.0);
    this.floorMesh.receiveShadow = true;
    this.group.add(this.floorMesh);
  }

  /**
   * Build ceiling slab over the mapped building area at z = 2.4m.
   */
  private buildCeiling(minX: number, maxX: number, minY: number, maxY: number, heightZ: number) {
    if (this.ceilingMesh) {
      this.group.remove(this.ceilingMesh);
      this.ceilingMesh.geometry.dispose();
      (this.ceilingMesh.material as THREE.Material).dispose();
      this.ceilingMesh = null;
    }

    const w = Math.max(1, maxX - minX);
    const h = Math.max(1, maxY - minY);
    const geom = new THREE.BoxGeometry(w, h, 0.12);
    const mat = new THREE.MeshStandardMaterial({
      color: this.ceilingColor,
      roughness: 0.5,
      metalness: 0.3,
      clippingPlanes: [this.ceilingClipPlane],
      clipShadows: true
    });

    this.ceilingMesh = new THREE.Mesh(geom, mat);
    this.ceilingMesh.position.set((minX + maxX) / 2, (minY + maxY) / 2, heightZ + 0.06);
    this.ceilingMesh.castShadow = true;
    this.group.add(this.ceilingMesh);
  }

  /**
   * Build 3D voxel terrain from registered 3D point clouds (when available).
   */
  public buildFromPoints(
    positions: Float32Array,
    total: number,
    owners: Uint8Array,
    robotColors: string[]
  ): CloudBounds {
    if (total === 0) {
      return this.bounds;
    }

    this.clear();

    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;

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

    this.bounds = { minX, maxX, minY, maxY, minZ: 0, maxZ: Math.max(maxZ, 2.5) };

    // Voxel quantization: bin real points into discrete 3D cells
    const voxelMap = new Map<string, { x: number; y: number; z: number; count: number }>();
    const V = this.voxelSize;

    for (let i = 0; i < total; i++) {
      const x = positions[i * 3];
      const y = positions[i * 3 + 1];
      const z = positions[i * 3 + 2];
      const ix = Math.floor(x / V);
      const iy = Math.floor(y / V);
      const iz = Math.floor(z / V);
      const key = `${ix},${iy},${iz}`;
      let cell = voxelMap.get(key);
      if (!cell) {
        cell = { x: (ix + 0.5) * V, y: (iy + 0.5) * V, z: (iz + 0.5) * V, count: 0 };
        voxelMap.set(key, cell);
      }
      cell.count++;
    }

    const geom = new THREE.BoxGeometry(V * 0.94, V * 0.94, V * 0.94);
    const mat = new THREE.MeshStandardMaterial({
      color: this.wallColor,
      roughness: 0.35,
      metalness: 0.25,
      clippingPlanes: [this.ceilingClipPlane],
      clipShadows: true
    });

    const cells = Array.from(voxelMap.values());
    this.cloudMesh = new THREE.InstancedMesh(geom, mat, cells.length);
    this.cloudMesh.castShadow = true;
    this.cloudMesh.receiveShadow = true;

    const dummy = new THREE.Object3D();
    for (let i = 0; i < cells.length; i++) {
      const c = cells[i];
      dummy.position.set(c.x, c.y, c.z);
      dummy.updateMatrix();
      this.cloudMesh.setMatrixAt(i, dummy.matrix);
    }
    this.cloudMesh.instanceMatrix.needsUpdate = true;
    this.group.add(this.cloudMesh);

    this.voxelCount = cells.length;
    return this.bounds;
  }

  public clear() {
    if (this.wallMesh) {
      this.group.remove(this.wallMesh);
      this.wallMesh.geometry.dispose();
      (this.wallMesh.material as THREE.Material).dispose();
      this.wallMesh = null;
    }
    if (this.wallCapMesh) {
      this.group.remove(this.wallCapMesh);
      this.wallCapMesh.geometry.dispose();
      (this.wallCapMesh.material as THREE.Material).dispose();
      this.wallCapMesh = null;
    }
    if (this.ceilingMesh) {
      this.group.remove(this.ceilingMesh);
      this.ceilingMesh.geometry.dispose();
      (this.ceilingMesh.material as THREE.Material).dispose();
      this.ceilingMesh = null;
    }
    if (this.cloudMesh) {
      this.group.remove(this.cloudMesh);
      this.cloudMesh.geometry.dispose();
      (this.cloudMesh.material as THREE.Material).dispose();
      this.cloudMesh = null;
    }
    this.voxelCount = 0;
  }

  public dispose() {
    this.clear();
    if (this.floorMesh) {
      this.group.remove(this.floorMesh);
      this.floorMesh.geometry.dispose();
      (this.floorMesh.material as THREE.Material).dispose();
      this.floorMesh = null;
    }
    if (this.floorTexture) {
      this.floorTexture.dispose();
      this.floorTexture = null;
    }
    this.floorCanvas = null;
  }
}
