import * as THREE from 'three';
import type { CloudBounds, Map3DColorMode, Map3DRenderMode } from './types';

function createCirclePointTexture(): THREE.CanvasTexture {
  const c = document.createElement('canvas');
  c.width = 32;
  c.height = 32;
  const ctx = c.getContext('2d');
  if (ctx) {
    const rad = 14;
    const grad = ctx.createRadialGradient(16, 16, 0, 16, 16, rad);
    grad.addColorStop(0, 'rgba(255,255,255,1)');
    grad.addColorStop(0.7, 'rgba(255,255,255,0.95)');
    grad.addColorStop(1, 'rgba(255,255,255,0)');
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.arc(16, 16, rad, 0, Math.PI * 2);
    ctx.fill();
  }
  const tex = new THREE.CanvasTexture(c);
  tex.minFilter = THREE.LinearFilter;
  tex.magFilter = THREE.LinearFilter;
  return tex;
}

function elevationToRgb(z: number, minZ: number, maxZ: number, outColor: THREE.Color) {
  const range = Math.max(0.1, maxZ - minZ);
  const t = Math.max(0, Math.min(1, (z - minZ) / range));
  // Multi-stop tactical terrain ramp:
  // Low (pits, floors): deep cyan / blue
  // Mid (slopes, terrain, walls): vibrant emerald green -> amber
  // High (peaks, upper structures, cave roofs): warm coral -> crimson
  if (t < 0.25) {
    const u = t / 0.25;
    outColor.setRGB(
      THREE.MathUtils.lerp(0.15, 0.02, u),
      THREE.MathUtils.lerp(0.45, 0.75, u),
      THREE.MathUtils.lerp(0.95, 0.85, u)
    );
  } else if (t < 0.5) {
    const u = (t - 0.25) / 0.25;
    outColor.setRGB(
      THREE.MathUtils.lerp(0.02, 0.06, u),
      THREE.MathUtils.lerp(0.75, 0.78, u),
      THREE.MathUtils.lerp(0.85, 0.45, u)
    );
  } else if (t < 0.75) {
    const u = (t - 0.5) / 0.25;
    outColor.setRGB(
      THREE.MathUtils.lerp(0.06, 0.95, u),
      THREE.MathUtils.lerp(0.78, 0.72, u),
      THREE.MathUtils.lerp(0.45, 0.05, u)
    );
  } else {
    const u = (t - 0.75) / 0.25;
    outColor.setRGB(
      THREE.MathUtils.lerp(0.95, 0.94, u),
      THREE.MathUtils.lerp(0.72, 0.25, u),
      THREE.MathUtils.lerp(0.05, 0.25, u)
    );
  }
}

export class VoxelTerrain {
  public group = new THREE.Group();

  // Primary 3D geometry
  public pointsMesh: THREE.Points | null = null;
  public surfaceMesh: THREE.InstancedMesh | null = null;
  public gridHelper: THREE.GridHelper | null = null;

  // GPU Clipping Plane for ceiling / height cutoff
  public ceilingClipPlane: THREE.Plane;
  private currentCutoff = 3.0;

  // Cloud metrics
  public bounds: CloudBounds = {
    minX: -15,
    maxX: 15,
    minY: -15,
    maxY: 15,
    minZ: -1.0,
    maxZ: 3.0
  };
  public pointCount = 0;
  public voxelCount = 0;

  // Display Configuration
  public renderMode: Map3DRenderMode = 'points';
  public colorMode: Map3DColorMode = 'robot';
  public pointSize = 2.6;

  // Cached color buffers for instant switching without re-allocating
  private rawPositions: Float32Array | null = null;
  private robotColorsBuf: Float32Array | null = null;
  private elevationColorsBuf: Float32Array | null = null;
  private voxelColorsRobot: Float32Array | null = null;
  private voxelColorsElevation: Float32Array | null = null;
  private pointTexture: THREE.CanvasTexture | null = null;

  // Spatial ground map for robot elevation snapping
  private groundElevationMap = new Map<string, number>();
  private groundCellSize = 0.25; // 25cm XY bins for ground tracking

  constructor() {
    this.ceilingClipPlane = new THREE.Plane(new THREE.Vector3(0, 0, -1), this.currentCutoff);
    this.pointTexture = createCirclePointTexture();

    // Subtle tactical reference ground grid at z = 0
    this.gridHelper = new THREE.GridHelper(40, 40, 0x475569, 0x334155);
    this.gridHelper.rotation.x = Math.PI / 2; // Lie on Z=0 plane
    this.gridHelper.position.set(0, 0, -0.01);
    this.group.add(this.gridHelper);
  }

  public setCeilingCutoff(cutoffZ: number) {
    this.currentCutoff = cutoffZ;
    this.ceilingClipPlane.constant = cutoffZ;
  }

  public setRenderMode(mode: Map3DRenderMode) {
    this.renderMode = mode;
    this.updateVisibility();
  }

  public setColorMode(mode: Map3DColorMode) {
    if (this.colorMode === mode) return;
    this.colorMode = mode;
    this.applyColorMode();
  }

  public setPointSize(size: number) {
    this.pointSize = Math.max(1.0, Math.min(8.0, size));
    if (this.pointsMesh) {
      (this.pointsMesh.material as THREE.PointsMaterial).size = this.pointSize;
    }
  }

  private updateVisibility() {
    // Points and meshes swapped per user request:
    // "points" mode displays the discrete 3D voxel terrain (instanced boxes)
    // "mesh" mode displays the dense LiDAR point cloud surface
    if (this.pointsMesh) {
      this.pointsMesh.visible = this.renderMode === 'mesh' || this.renderMode === 'both';
    }
    if (this.surfaceMesh) {
      this.surfaceMesh.visible = this.renderMode === 'points' || this.renderMode === 'both';
    }
  }

  private applyColorMode() {
    if (this.pointsMesh) {
      const geom = this.pointsMesh.geometry;
      if (this.colorMode === 'robot' && this.robotColorsBuf) {
        geom.setAttribute('color', new THREE.BufferAttribute(this.robotColorsBuf, 3));
        geom.attributes.color.needsUpdate = true;
      } else if (this.colorMode === 'elevation' && this.elevationColorsBuf) {
        geom.setAttribute('color', new THREE.BufferAttribute(this.elevationColorsBuf, 3));
        geom.attributes.color.needsUpdate = true;
      }
    }
    if (this.surfaceMesh && this.surfaceMesh.instanceColor) {
      const colors = this.colorMode === 'robot' ? this.voxelColorsRobot : this.voxelColorsElevation;
      if (colors) {
        const col = new THREE.Color();
        for (let i = 0; i < this.voxelCount; i++) {
          col.setRGB(colors[i * 3], colors[i * 3 + 1], colors[i * 3 + 2]);
          this.surfaceMesh.setColorAt(i, col);
        }
        this.surfaceMesh.instanceColor.needsUpdate = true;
      }
    }
  }

  /**
   * Return the terrain ground surface height Z at coordinates (x, y).
   * Snaps the robot and waypoints to real terrain contours (caves, slopes, outdoor terrain).
   */
  public getGroundZ(x: number, y: number): number {
    const ix = Math.round(x / this.groundCellSize);
    const iy = Math.round(y / this.groundCellSize);
    const key = `${ix},${iy}`;
    const direct = this.groundElevationMap.get(key);
    if (direct !== undefined) return direct;

    // Search immediate 8-neighbor cells
    let minZ = Infinity;
    for (let dx = -1; dx <= 1; dx++) {
      for (let dy = -1; dy <= 1; dy++) {
        const nKey = `${ix + dx},${iy + dy}`;
        const z = this.groundElevationMap.get(nKey);
        if (z !== undefined && z < minZ) {
          minZ = z;
        }
      }
    }
    return minZ !== Infinity ? minZ : 0.0;
  }

  /**
   * Build 3D Point Cloud and Voxel Surface Reconstruction directly from the real 3D points.
   */
  public buildFromPoints(
    positions: Float32Array,
    total: number,
    owners: Uint8Array,
    robotColorHexes: string[]
  ): CloudBounds {
    if (total === 0) {
      return this.bounds;
    }

    this.clearGeometry();
    this.pointCount = total;
    this.rawPositions = positions;

    // 1. Calculate bounding box & ground heightmap
    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;

    this.groundElevationMap.clear();

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

      // Track lowest point in XY cell for ground snapping
      const gix = Math.round(x / this.groundCellSize);
      const giy = Math.round(y / this.groundCellSize);
      const gKey = `${gix},${giy}`;
      const curG = this.groundElevationMap.get(gKey);
      if (curG === undefined || z < curG) {
        this.groundElevationMap.set(gKey, z);
      }
    }

    this.bounds = {
      minX: minX - 0.5,
      maxX: maxX + 0.5,
      minY: minY - 0.5,
      maxY: maxY + 0.5,
      minZ: Math.min(minZ, 0.0),
      maxZ: Math.max(maxZ, 2.5)
    };

    // Update ground grid position to follow center
    if (this.gridHelper) {
      this.gridHelper.position.set((minX + maxX) / 2, (minY + maxY) / 2, Math.min(0, minZ) - 0.01);
    }

    // 2. Build Team and Elevation Color Buffers
    const palette = robotColorHexes.map((hex) => new THREE.Color(hex));
    const fallbackColor = new THREE.Color('#94a3b8');

    const robotColors = new Float32Array(total * 3);
    const elevationColors = new Float32Array(total * 3);
    const tmpColor = new THREE.Color();

    for (let i = 0; i < total; i++) {
      const z = positions[i * 3 + 2];
      const owner = owners[i];
      const rc = palette[owner] ?? fallbackColor;

      robotColors[i * 3] = rc.r;
      robotColors[i * 3 + 1] = rc.g;
      robotColors[i * 3 + 2] = rc.b;

      elevationToRgb(z, minZ, maxZ, tmpColor);
      elevationColors[i * 3] = tmpColor.r;
      elevationColors[i * 3 + 1] = tmpColor.g;
      elevationColors[i * 3 + 2] = tmpColor.b;
    }

    this.robotColorsBuf = robotColors;
    this.elevationColorsBuf = elevationColors;

    // 3. Construct High-Performance THREE.Points
    const pointsGeom = new THREE.BufferGeometry();
    pointsGeom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    pointsGeom.setAttribute(
      'color',
      new THREE.BufferAttribute(this.colorMode === 'robot' ? robotColors : elevationColors, 3)
    );

    const pointsMat = new THREE.PointsMaterial({
      size: this.pointSize,
      vertexColors: true,
      sizeAttenuation: true,
      map: this.pointTexture ?? undefined,
      transparent: true,
      alphaTest: 0.1,
      clippingPlanes: [this.ceilingClipPlane],
      clipShadows: true
    });

    this.pointsMesh = new THREE.Points(pointsGeom, pointsMat);
    this.group.add(this.pointsMesh);

    // 4. Construct 3D Mesh / Surface Reconstruction (3D Voxel Surfels)
    // Downsample voxels to keep max 10,000 voxels in view for 60fps performance
    const V = 0.25; // 25cm discrete 3D spatial bins
    const voxelMap = new Map<string, { x: number; y: number; z: number; owner: number }>();

    for (let i = 0; i < total; i++) {
      const x = positions[i * 3];
      const y = positions[i * 3 + 1];
      const z = positions[i * 3 + 2];
      const ix = Math.floor(x / V);
      const iy = Math.floor(y / V);
      const iz = Math.floor(z / V);
      const key = `${ix},${iy},${iz}`;
      if (!voxelMap.has(key)) {
        voxelMap.set(key, {
          x: (ix + 0.5) * V,
          y: (iy + 0.5) * V,
          z: (iz + 0.5) * V,
          owner: owners[i]
        });
      }
    }

    const MAX_VOXELS = 10000;
    let voxels = Array.from(voxelMap.values());
    if (voxels.length > MAX_VOXELS) {
      const step = voxels.length / MAX_VOXELS;
      const sampled: typeof voxels = new Array(MAX_VOXELS);
      for (let i = 0; i < MAX_VOXELS; i++) {
        sampled[i] = voxels[Math.floor(i * step)];
      }
      voxels = sampled;
    }
    this.voxelCount = voxels.length;

    if (voxels.length > 0) {
      const boxGeom = new THREE.BoxGeometry(V * 0.94, V * 0.94, V * 0.94);
      const surfMat = new THREE.MeshStandardMaterial({
        roughness: 0.45,
        metalness: 0.2,
        clippingPlanes: [this.ceilingClipPlane],
        clipShadows: true
      });

      this.surfaceMesh = new THREE.InstancedMesh(boxGeom, surfMat, voxels.length);
      this.surfaceMesh.castShadow = true;
      this.surfaceMesh.receiveShadow = true;

      this.voxelColorsRobot = new Float32Array(voxels.length * 3);
      this.voxelColorsElevation = new Float32Array(voxels.length * 3);

      const dummy = new THREE.Object3D();
      for (let i = 0; i < voxels.length; i++) {
        const v = voxels[i];
        dummy.position.set(v.x, v.y, v.z);
        dummy.updateMatrix();
        this.surfaceMesh.setMatrixAt(i, dummy.matrix);

        // Precompute both robot and elevation colors for instant switching
        const rc = palette[v.owner] ?? fallbackColor;
        this.voxelColorsRobot[i * 3] = rc.r;
        this.voxelColorsRobot[i * 3 + 1] = rc.g;
        this.voxelColorsRobot[i * 3 + 2] = rc.b;

        elevationToRgb(v.z, minZ, maxZ, tmpColor);
        this.voxelColorsElevation[i * 3] = tmpColor.r;
        this.voxelColorsElevation[i * 3 + 1] = tmpColor.g;
        this.voxelColorsElevation[i * 3 + 2] = tmpColor.b;

        if (this.colorMode === 'robot') {
          this.surfaceMesh.setColorAt(i, rc);
        } else {
          this.surfaceMesh.setColorAt(i, tmpColor);
        }
      }

      this.surfaceMesh.instanceMatrix.needsUpdate = true;
      if (this.surfaceMesh.instanceColor) {
        this.surfaceMesh.instanceColor.needsUpdate = true;
      }
      this.group.add(this.surfaceMesh);
    }

    this.updateVisibility();
    return this.bounds;
  }

  public clearGeometry() {
    if (this.pointsMesh) {
      this.group.remove(this.pointsMesh);
      this.pointsMesh.geometry.dispose();
      (this.pointsMesh.material as THREE.Material).dispose();
      this.pointsMesh = null;
    }
    if (this.surfaceMesh) {
      this.group.remove(this.surfaceMesh);
      this.surfaceMesh.geometry.dispose();
      (this.surfaceMesh.material as THREE.Material).dispose();
      this.surfaceMesh = null;
    }
    this.pointCount = 0;
    this.voxelCount = 0;
  }

  public clear() {
    this.clearGeometry();
    this.groundElevationMap.clear();
    this.rawPositions = null;
    this.robotColorsBuf = null;
    this.elevationColorsBuf = null;
    this.voxelColorsRobot = null;
    this.voxelColorsElevation = null;
  }

  public dispose() {
    this.clear();
    if (this.gridHelper) {
      this.group.remove(this.gridHelper);
      this.gridHelper.geometry.dispose();
      (this.gridHelper.material as THREE.Material).dispose();
      this.gridHelper = null;
    }
    if (this.pointTexture) {
      this.pointTexture.dispose();
      this.pointTexture = null;
    }
  }
}
