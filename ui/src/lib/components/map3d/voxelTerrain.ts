import * as THREE from 'three';
import type { CloudBounds, Map3DColorMode, Map3DRenderMode } from './types';
import type { TerrainData } from './terrainData';

export class VoxelTerrain {
  public group = new THREE.Group();
  public pointsMesh: THREE.Points | null = null;
  public surfaceMesh: THREE.Mesh | null = null;
  public voxelMesh: THREE.InstancedMesh | null = null;
  public ceilingClipPlane = new THREE.Plane(new THREE.Vector3(0, 0, -1), 3);
  public bounds: CloudBounds = {
    minX: -15,
    maxX: 15,
    minY: -15,
    maxY: 15,
    minZ: 0,
    maxZ: 3
  };
  public pointCount = 0;
  public voxelCount = 0;
  public renderMode: Map3DRenderMode = 'voxels';
  public colorMode: Map3DColorMode = 'elevation';
  public pointSize = 0.07;
  private data: TerrainData | null = null;
  private colors: Float32Array | null = null;
  private palette: THREE.Color[] = [];
  private ground = new Map<string, number>();
  public setCeilingCutoff(z: number) {
    this.ceilingClipPlane.constant = z;
  }
  public setPointSize(size: number) {
    this.pointSize = Math.max(0.01, Math.min(0.5, size));
    if (this.pointsMesh) (this.pointsMesh.material as THREE.PointsMaterial).size = this.pointSize;
  }
  public setRenderMode(mode: Map3DRenderMode) {
    this.renderMode = mode;
    this.ensureGeometry();
    if (this.pointsMesh) this.pointsMesh.visible = mode === 'points';
    if (this.voxelMesh) this.voxelMesh.visible = mode === 'voxels';
    if (this.surfaceMesh) this.surfaceMesh.visible = mode === 'mesh';
  }
  public setColorMode(mode: Map3DColorMode) {
    if (this.colorMode === mode) return;
    this.colorMode = mode;
    this.colors = null;
    this.applyColors();
  }
  public getGroundZ(x: number, y: number) {
    return this.ground.get(`${Math.floor(x / 0.5)},${Math.floor(y / 0.5)}`) ?? 0;
  }
  public build(data: TerrainData, hexes: string[]): CloudBounds {
    this.clear();
    this.data = data;
    this.palette = hexes.map((h) => new THREE.Color(h));
    this.pointCount = data.xyz.length / 3;
    this.voxelCount = data.samples.length;
    if (!this.pointCount) return this.bounds;
    const box = new THREE.Box3().setFromBufferAttribute(new THREE.BufferAttribute(data.xyz, 3));
    this.bounds = {
      minX: box.min.x,
      maxX: box.max.x,
      minY: box.min.y,
      maxY: box.max.y,
      minZ: box.min.z,
      maxZ: box.max.z
    };
    for (let i = 0; i < this.pointCount; i++) {
      const x = data.xyz[i * 3],
        y = data.xyz[i * 3 + 1],
        z = data.xyz[i * 3 + 2],
        key = `${Math.floor(x / 0.5)},${Math.floor(y / 0.5)}`;
      this.ground.set(key, Math.min(this.ground.get(key) ?? Infinity, z));
    }
    this.setRenderMode(this.renderMode);
    return this.bounds;
  }
  private ensureGeometry() {
    const d = this.data;
    if (!d || !this.pointCount) return;
    if (this.renderMode === 'points' && !this.pointsMesh) {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(d.xyz, 3));
      this.pointsMesh = new THREE.Points(
        g,
        new THREE.PointsMaterial({
          size: this.pointSize,
          vertexColors: true,
          clippingPlanes: [this.ceilingClipPlane]
        })
      );
      this.group.add(this.pointsMesh);
      this.applyColors('points');
    }
    if (this.renderMode === 'voxels' && !this.voxelMesh) {
      this.voxelMesh = new THREE.InstancedMesh(
        new THREE.BoxGeometry(d.size * 0.94, d.size * 0.94, d.size * 0.94),
        new THREE.MeshLambertMaterial({
          clippingPlanes: [this.ceilingClipPlane]
        }),
        this.voxelCount
      );
      const m = new THREE.Matrix4();
      for (let i = 0; i < this.voxelCount; i++)
        this.voxelMesh.setMatrixAt(
          i,
          m.makeTranslation(d.centers[i * 3], d.centers[i * 3 + 1], d.centers[i * 3 + 2])
        );
      this.voxelMesh.computeBoundingSphere();
      this.group.add(this.voxelMesh);
      this.applyColors('voxels');
    }
    if (this.renderMode === 'mesh' && !this.surfaceMesh) {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(d.meshPositions, 3));
      g.setAttribute('normal', new THREE.BufferAttribute(d.meshNormals, 3));
      this.surfaceMesh = new THREE.Mesh(
        g,
        new THREE.MeshLambertMaterial({
          vertexColors: true,
          side: THREE.DoubleSide,
          clippingPlanes: [this.ceilingClipPlane]
        })
      );
      this.group.add(this.surfaceMesh);
      this.applyColors('mesh');
    }
  }
  private pointColors() {
    if (this.colors) return this.colors;
    const d = this.data!;
    const colors = new Float32Array(this.pointCount * 3),
      c = new THREE.Color();
    const height = Math.max(0.1, this.bounds.maxZ - this.bounds.minZ);
    for (let i = 0; i < this.pointCount; i++) {
      if (this.colorMode === 'camera' && d.rgb)
        c.setRGB(
          d.rgb[i * 3] / 255,
          d.rgb[i * 3 + 1] / 255,
          d.rgb[i * 3 + 2] / 255,
          THREE.SRGBColorSpace
        );
      else if (this.colorMode === 'robot') c.copy(this.palette[d.owners[i]] ?? c.set('#94a3b8'));
      else
        c.setHSL(
          0.58 - 0.52 * Math.max(0, Math.min(1, (d.xyz[i * 3 + 2] - this.bounds.minZ) / height)),
          0.65,
          0.52
        );
      c.toArray(colors, i * 3);
    }
    // Keep only the current palette, shared by lazily created representations.
    this.colors = colors;
    return colors;
  }
  private applyColors(only?: Map3DRenderMode) {
    const d = this.data;
    if (!d) return;
    const colors = this.pointColors(),
      c = new THREE.Color();
    if (this.pointsMesh && (!only || only === 'points'))
      this.pointsMesh.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    if (this.voxelMesh && (!only || only === 'voxels')) {
      for (let i = 0; i < this.voxelCount; i++)
        this.voxelMesh.setColorAt(i, c.fromArray(colors, d.samples[i] * 3));
      if (this.voxelMesh.instanceColor) this.voxelMesh.instanceColor.needsUpdate = true;
    }
    if (this.surfaceMesh && (!only || only === 'mesh')) {
      const meshColors = new Float32Array(d.meshSamples.length * 3);
      for (let i = 0; i < d.meshSamples.length; i++) {
        const source = d.meshSamples[i] * 3;
        meshColors[i * 3] = colors[source];
        meshColors[i * 3 + 1] = colors[source + 1];
        meshColors[i * 3 + 2] = colors[source + 2];
      }
      this.surfaceMesh.geometry.setAttribute('color', new THREE.BufferAttribute(meshColors, 3));
    }
  }
  public clear() {
    for (const mesh of [this.pointsMesh, this.voxelMesh, this.surfaceMesh])
      if (mesh) {
        this.group.remove(mesh);
        mesh.geometry.dispose();
        (mesh.material as THREE.Material).dispose();
        if (mesh instanceof THREE.InstancedMesh) mesh.dispose();
      }
    this.pointsMesh = null;
    this.voxelMesh = null;
    this.surfaceMesh = null;
    this.data = null;
    this.colors = null;
    this.pointCount = 0;
    this.voxelCount = 0;
    this.ground.clear();
    this.bounds = {
      minX: -15,
      maxX: 15,
      minY: -15,
      maxY: 15,
      minZ: 0,
      maxZ: 3
    };
  }
  public dispose() {
    this.clear();
  }
}
