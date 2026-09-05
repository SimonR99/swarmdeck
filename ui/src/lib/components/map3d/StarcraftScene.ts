import * as THREE from 'three';
import { VoxelTerrain } from './voxelTerrain';
import { Robot3DManager } from './robot3d';
import { StarcraftLayers } from './starcraftLayers';
import type { MapRobot } from '../map2d/mapLayers';

export class StarcraftScene {
  public canvas: HTMLCanvasElement;
  public renderer: THREE.WebGLRenderer;
  public scene: THREE.Scene;
  public camera: THREE.PerspectiveCamera;
  public raycaster = new THREE.Raycaster();

  public terrain: VoxelTerrain;
  public robotManager: Robot3DManager;
  public layers: StarcraftLayers;

  // RTS Orbit Camera: in world frame, Z is up!
  public target = new THREE.Vector3(0, 0, 0.5);
  public yaw = -0.75; // ~45 deg isometric
  public pitch = 0.95; // ~55 deg high RTS angle
  public distance = 26.0;

  private sunLight: THREE.DirectionalLight;
  private hemiLight: THREE.HemisphereLight;
  private cameraFillLight: THREE.PointLight;
  private groundPlane: THREE.Plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);

  private raf = 0;
  private isDisposed = false;

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;

    // Renderer with clipping enabled for the ceiling slider
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
    this.scene.background = new THREE.Color(0x0f131a); // StarCraft dark space/command center tone
    this.scene.fog = new THREE.FogExp2(0x0f131a, 0.008);

    const aspect = canvas.clientWidth / Math.max(1, canvas.clientHeight);
    this.camera = new THREE.PerspectiveCamera(42, aspect, 0.2, 500);
    // Crucial: Set camera UP to Z axis so Z is UP everywhere without axis rotation bugs!
    this.camera.up.set(0, 0, 1);

    // StarCraft Sci-Fi RTS Lighting
    this.hemiLight = new THREE.HemisphereLight(0x405570, 0x141820, 1.4);
    this.scene.add(this.hemiLight);

    this.sunLight = new THREE.DirectionalLight(0xfff5e4, 2.2);
    this.sunLight.position.set(20, -25, 40);
    this.sunLight.castShadow = true;
    this.sunLight.shadow.mapSize.width = 2048;
    this.sunLight.shadow.mapSize.height = 2048;
    this.sunLight.shadow.camera.near = 1;
    this.sunLight.shadow.camera.far = 120;
    const d = 30;
    this.sunLight.shadow.camera.left = -d;
    this.sunLight.shadow.camera.right = d;
    this.sunLight.shadow.camera.top = d;
    this.sunLight.shadow.camera.bottom = -d;
    this.sunLight.shadow.bias = -0.0005;
    this.scene.add(this.sunLight);

    this.cameraFillLight = new THREE.PointLight(0x7090b0, 0.8, 100);
    this.scene.add(this.cameraFillLight);

    // Submodules
    this.terrain = new VoxelTerrain();
    this.scene.add(this.terrain.group);

    this.robotManager = new Robot3DManager();
    this.scene.add(this.robotManager.group);

    this.layers = new StarcraftLayers();
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
  }

  public render() {
    if (this.isDisposed) return;
    this.updateCamera();
    this.renderer.render(this.scene, this.camera);
  }

  /**
   * Raycast from normalized screen pointer (x: -1..1, y: 1..-1) onto the z=0 ground plane.
   */
  public raycastGround(ndc: THREE.Vector2): THREE.Vector3 | null {
    this.raycaster.setFromCamera(ndc, this.camera);
    const target = new THREE.Vector3();
    const hit = this.raycaster.ray.intersectPlane(this.groundPlane, target);
    return hit;
  }

  /**
   * Raycast against scene objects (robots, detections).
   */
  public raycastInteractive(ndc: THREE.Vector2): {
    robotId?: string;
    detectionId?: string;
    groundPoint?: THREE.Vector3;
  } {
    this.raycaster.setFromCamera(ndc, this.camera);

    // 1. Check robots first
    const robotHits = this.raycaster.intersectObjects(this.robotManager.group.children, true);
    for (const hit of robotHits) {
      let cur: THREE.Object3D | null = hit.object;
      while (cur && cur !== this.robotManager.group) {
        if (cur.userData?.robotId) {
          return { robotId: cur.userData.robotId };
        }
        cur = cur.parent;
      }
    }

    // 2. Check detection crystals
    const detHits = this.raycaster.intersectObjects(this.layers.group.children, true);
    for (const hit of detHits) {
      let cur: THREE.Object3D | null = hit.object;
      while (cur && cur !== this.layers.group) {
        if (cur.userData?.detectionId) {
          return { detectionId: cur.userData.detectionId };
        }
        cur = cur.parent;
      }
    }

    // 3. Fallback to ground
    const groundPoint = this.raycastGround(ndc);
    return { groundPoint: groundPoint ?? undefined };
  }

  /**
   * Convert a 3D world coordinate to 2D screen coordinates on the canvas.
   */
  public worldToScreen(worldPos: THREE.Vector3): { sx: number; sy: number; visible: boolean } {
    const v = worldPos.clone().project(this.camera);
    const sx = ((v.x + 1) / 2) * this.canvas.clientWidth;
    const sy = ((-v.y + 1) / 2) * this.canvas.clientHeight;
    return {
      sx,
      sy,
      visible: v.z < 1.0 && sx >= 0 && sx <= this.canvas.clientWidth && sy >= 0 && sy <= this.canvas.clientHeight
    };
  }

  // Camera Actions
  public centreRobots(robots: MapRobot[], ids?: Set<string>): boolean {
    const targets = robots.filter((r) => !ids || ids.has(r.robot_id));
    if (!targets.length) return false;
    const cx = targets.reduce((sum, r) => sum + r.pose.x, 0) / targets.length;
    const cy = targets.reduce((sum, r) => sum + r.pose.y, 0) / targets.length;
    this.target.set(cx, cy, 0.5);
    return true;
  }

  public zoomBy(factor: number) {
    this.distance = Math.max(3.0, Math.min(140.0, this.distance / factor));
  }

  public rotateBy(angleDelta: number) {
    this.yaw += angleDelta;
  }

  public resetRotation() {
    this.yaw = -0.75;
    this.pitch = 0.95;
  }

  public setCeiling(height: number) {
    this.terrain.setCeilingCutoff(height);
  }

  public panBy(deltaScreenX: number, deltaScreenY: number) {
    // Convert screen drag deltas to world pan deltas in the camera's ground plane
    const factor = (this.distance * 0.0018);
    const forward = new THREE.Vector2(-Math.cos(this.yaw), -Math.sin(this.yaw));
    const right = new THREE.Vector2(-Math.sin(this.yaw), Math.cos(this.yaw));

    this.target.x += (right.x * deltaScreenX + forward.x * deltaScreenY) * factor;
    this.target.y += (right.y * deltaScreenX + forward.y * deltaScreenY) * factor;
  }

  public fitCloud() {
    const b = this.terrain.bounds;
    this.target.set((b.minX + b.maxX) / 2, (b.minY + b.maxY) / 2, (b.minZ + b.maxZ) / 2);
    const diameter = Math.hypot(b.maxX - b.minX, b.maxY - b.minY, b.maxZ - b.minZ);
    this.distance = Math.max(8.0, Math.min(140.0, diameter * 1.15));
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
