import * as THREE from 'three';
import { VoxelTerrain } from './voxelTerrain';
import { Robot3DManager } from './robot3d';
import { Map3DLayers } from './map3dLayers';
import type { MapRobot } from '../map2d/mapLayers';
import type { MapInfo } from '$lib/types/protocol';

export class Map3DScene {
  public canvas: HTMLCanvasElement;
  public renderer: THREE.WebGLRenderer;
  public scene: THREE.Scene;
  public camera: THREE.PerspectiveCamera;
  public raycaster = new THREE.Raycaster();

  public terrain: VoxelTerrain;
  public robotManager: Robot3DManager;
  public layers: Map3DLayers;

  // Tactical 3D Orbit Camera: in world frame, Z is up!
  public target = new THREE.Vector3(0, 0, 0.5);
  public yaw = -0.75; // ~45 deg isometric
  public pitch = 0.95; // ~55 deg tactical viewing angle
  public distance = 26.0;

  private sunLight: THREE.DirectionalLight;
  private hemiLight: THREE.HemisphereLight;
  private ambientLight: THREE.AmbientLight;
  private cameraFillLight: THREE.PointLight;
  private groundPlane: THREE.Plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);

  private raf = 0;
  private isDisposed = false;

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;

    // High-performance renderer with local clipping for ceiling removal
    this.renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      preserveDrawingBuffer: true,
      powerPreference: 'high-performance'
    });
    this.renderer.localClippingEnabled = true;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    // Scene & Camera
    this.scene = new THREE.Scene();
    // Clean, high-contrast tactical slate background (not pitch black)
    this.scene.background = new THREE.Color(0x1e242d);
    this.scene.fog = new THREE.FogExp2(0x1e242d, 0.003);

    const aspect = canvas.clientWidth / Math.max(1, canvas.clientHeight);
    this.camera = new THREE.PerspectiveCamera(42, aspect, 0.2, 500);
    // Crucial: Set camera UP to Z axis so Z is UP everywhere without axis rotation bugs!
    this.camera.up.set(0, 0, 1);

    // Bright, high-visibility 3D Tactical Lighting
    // 1. Ambient hemisphere light (bright white sky, soft slate ground)
    this.hemiLight = new THREE.HemisphereLight(0xffffff, 0x94a3b8, 2.2);
    this.scene.add(this.hemiLight);

    // 2. Global soft ambient fill to eliminate dark, unreadable shadows
    this.ambientLight = new THREE.AmbientLight(0xffffff, 0.8);
    this.scene.add(this.ambientLight);

    // 3. Primary directional sun lighting for depth and crisp surface edges
    this.sunLight = new THREE.DirectionalLight(0xffffff, 2.8);
    this.sunLight.position.set(25, -20, 35);
    this.sunLight.castShadow = true;
    this.sunLight.shadow.mapSize.width = 2048;
    this.sunLight.shadow.mapSize.height = 2048;
    this.sunLight.shadow.camera.near = 1;
    this.sunLight.shadow.camera.far = 120;
    const d = 35;
    this.sunLight.shadow.camera.left = -d;
    this.sunLight.shadow.camera.right = d;
    this.sunLight.shadow.camera.top = d;
    this.sunLight.shadow.camera.bottom = -d;
    this.sunLight.shadow.bias = -0.0005;
    this.scene.add(this.sunLight);

    // 4. Camera fill light for front-facing facets
    this.cameraFillLight = new THREE.PointLight(0xf1f5f9, 1.2, 150);
    this.scene.add(this.cameraFillLight);

    // Submodules
    this.terrain = new VoxelTerrain();
    this.scene.add(this.terrain.group);

    this.robotManager = new Robot3DManager();
    this.scene.add(this.robotManager.group);

    this.layers = new Map3DLayers();
    this.scene.add(this.layers.group);

    this.updateCamera();
  }

  public updateCamera() {
    this.pitch = Math.max(0.12, Math.min(1.48, this.pitch));
    this.distance = Math.max(3.0, Math.min(140.0, this.distance));

    const cp = Math.cos(this.pitch);
    const sp = Math.sin(this.pitch);
    const cy = Math.cos(this.yaw);
    const sy = Math.sin(this.yaw);

    this.camera.position.set(
      this.target.x + this.distance * cp * cy,
      this.target.y + this.distance * cp * sy,
      this.target.z + this.distance * sp
    );

    this.camera.lookAt(this.target);
    this.cameraFillLight.position.copy(this.camera.position);

    // Update sunlight focal point to target
    this.sunLight.target.position.copy(this.target);
    this.sunLight.target.updateMatrixWorld();
  }

  public resize() {
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    if (!w || !h) return;
    const dpr = window.devicePixelRatio || 1;
    this.renderer.setPixelRatio(dpr);
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.render();
  }

  public render() {
    if (this.isDisposed) return;
    this.renderer.render(this.scene, this.camera);
  }

  public setCeiling(cutoffZ: number) {
    this.terrain.setCeilingCutoff(cutoffZ);
  }

  /**
   * Raycast ground plane at z=0 to get world XY coordinate for navigation goal placement.
   */
  public raycastGround(ndc: THREE.Vector2): THREE.Vector3 | null {
    this.raycaster.setFromCamera(ndc, this.camera);
    const hit = new THREE.Vector3();
    const intersects = this.raycaster.ray.intersectPlane(this.groundPlane, hit);
    return intersects ? hit : null;
  }

  /**
   * Raycast robots to pick selected robot.
   */
  public raycastRobot(ndc: THREE.Vector2): string | null {
    this.raycaster.setFromCamera(ndc, this.camera);
    const hits = this.raycaster.intersectObjects(this.robotManager.group.children, true);
    for (const hit of hits) {
      let cur: THREE.Object3D | null = hit.object;
      while (cur && cur !== this.robotManager.group) {
        if (cur.userData?.robotId) {
          return cur.userData.robotId as string;
        }
        cur = cur.parent;
      }
    }
    return null;
  }

  /**
   * Screen coordinate projection (world -> screen pixel) for overlays.
   */
  public worldToScreen(pos: THREE.Vector3): { sx: number; sy: number; visible: boolean } {
    const v = pos.clone().project(this.camera);
    const isBehind = v.z > 1;
    const sx = ((v.x + 1) * this.canvas.clientWidth) / 2;
    const sy = ((-v.y + 1) * this.canvas.clientHeight) / 2;
    return {
      sx,
      sy,
      visible: !isBehind && v.x >= -1.1 && v.x <= 1.1 && v.y >= -1.1 && v.y <= 1.1
    };
  }

  // Camera Orbit, Pan & Zoom helpers
  public orbitBy(deltaYaw: number, deltaPitch: number) {
    this.yaw -= deltaYaw;
    this.pitch = Math.max(0.12, Math.min(1.48, this.pitch + deltaPitch));
    this.updateCamera();
  }

  public panBy(deltaScreenX: number, deltaScreenY: number) {
    // Pan in camera plane parallel to ground
    const forward = new THREE.Vector3();
    this.camera.getWorldDirection(forward);
    forward.z = 0;
    forward.normalize();

    const right = new THREE.Vector3();
    right.crossVectors(forward, new THREE.Vector3(0, 0, 1)).normalize();

    const panSpeed = this.distance * 0.0018;
    this.target.addScaledVector(right, -deltaScreenX * panSpeed);
    this.target.addScaledVector(forward, deltaScreenY * panSpeed);
    this.updateCamera();
  }

  public zoomBy(factor: number) {
    this.distance = Math.max(3.0, Math.min(140.0, this.distance / factor));
    this.updateCamera();
  }

  public rotateBy(angleDelta: number) {
    this.yaw += angleDelta;
    this.updateCamera();
  }

  public resetRotation() {
    this.yaw = -0.75;
    this.pitch = 0.95;
    this.updateCamera();
  }

  public centreRobots(robots: MapRobot[], ids?: Set<string>) {
    const targets = robots.filter((r) => !ids || ids.has(r.robot_id));
    if (!targets.length) return;
    const avgX = targets.reduce((sum, r) => sum + r.pose.x, 0) / targets.length;
    const avgY = targets.reduce((sum, r) => sum + r.pose.y, 0) / targets.length;
    this.target.set(avgX, avgY, 0.4);
    this.updateCamera();
  }

  public fitMap() {
    const b = this.terrain.bounds;
    this.target.set((b.minX + b.maxX) / 2, (b.minY + b.maxY) / 2, 0.4);
    const span = Math.hypot(b.maxX - b.minX, b.maxY - b.minY);
    this.distance = Math.max(12, Math.min(120, span * 1.15));
    this.updateCamera();
  }

  public fitCloud() {
    this.fitMap();
  }

  public dispose() {
    this.isDisposed = true;
    if (this.raf) cancelAnimationFrame(this.raf);
    this.terrain.dispose();
    this.robotManager.dispose();
    this.layers.dispose();
    this.renderer.dispose();
  }
}
