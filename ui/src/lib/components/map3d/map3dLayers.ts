import * as THREE from 'three';
import { decalPose, mapFrameZ, routePositions, type MapPoint3D } from './mapFrames';
import { Line2 } from 'three/addons/lines/Line2.js';
import { LineGeometry } from 'three/addons/lines/LineGeometry.js';
import { LineMaterial } from 'three/addons/lines/LineMaterial.js';
import { fleet } from '$lib/stores/fleet.svelte';
import { mapStore } from '$lib/stores/mapstore.svelte';
import { review } from '$lib/stores/review.svelte';
import { detectionCatalog } from '$lib/stores/detection.svelte';
import type { MapRobot } from '../map2d/mapLayers';

export class Map3DLayers {
  public group = new THREE.Group();

  // Sub-groups
  private gridGroup = new THREE.Group();
  private pathsGroup = new THREE.Group();
  private goalsGroup = new THREE.Group();
  private trailsGroup = new THREE.Group();
  private detectionsGroup = new THREE.Group();
  private loopClosuresGroup = new THREE.Group();
  private costmapMesh: THREE.Mesh | null = null;
  private networkMesh: THREE.Mesh | null = null;
  private signatures = new Map<string, string>();
  public cursorReticle: THREE.Group;

  constructor() {
    this.group.add(this.gridGroup);
    this.group.add(this.pathsGroup);
    this.group.add(this.goalsGroup);
    this.group.add(this.trailsGroup);
    this.group.add(this.detectionsGroup);
    this.group.add(this.loopClosuresGroup);

    // Goal cursor reticle
    this.cursorReticle = this.createCursorReticle();
    this.group.add(this.cursorReticle);

    this.buildTacticalGrid();
  }

  /**
   * Tactical terrain grid at z = 0.
   */
  public buildTacticalGrid(size = 50, majorStep = 5, minorStep = 1) {
    while (this.gridGroup.children.length) {
      const child = this.gridGroup.children[0];
      this.gridGroup.remove(child);
      if (child instanceof THREE.LineSegments) {
        child.geometry.dispose();
        (child.material as THREE.Material).dispose();
      }
    }

    const minorPts: number[] = [];
    const majorPts: number[] = [];
    const half = size / 2;

    for (let x = -half; x <= half; x += minorStep) {
      const isMajor = Math.round(x) % majorStep === 0;
      const target = isMajor ? majorPts : minorPts;
      target.push(x, -half, 0.005, x, half, 0.005);
    }
    for (let y = -half; y <= half; y += minorStep) {
      const isMajor = Math.round(y) % majorStep === 0;
      const target = isMajor ? majorPts : minorPts;
      target.push(-half, y, 0.005, half, y, 0.005);
    }

    // Minor lines: clean visible grid
    const minorGeo = new THREE.BufferGeometry();
    minorGeo.setAttribute('position', new THREE.Float32BufferAttribute(minorPts, 3));
    const minorMat = new THREE.LineBasicMaterial({
      color: 0x475569,
      transparent: true,
      opacity: 0.65
    });
    this.gridGroup.add(new THREE.LineSegments(minorGeo, minorMat));

    // Major lines: bright tactical lines
    const majorGeo = new THREE.BufferGeometry();
    majorGeo.setAttribute('position', new THREE.Float32BufferAttribute(majorPts, 3));
    const majorMat = new THREE.LineBasicMaterial({
      color: 0x64748b,
      transparent: true,
      opacity: 0.9
    });
    this.gridGroup.add(new THREE.LineSegments(majorGeo, majorMat));

    // Axis crosshairs at (0, 0)
    const axisGeo = new THREE.BufferGeometry();
    axisGeo.setAttribute(
      'position',
      new THREE.Float32BufferAttribute(
        [-half, 0, 0.008, half, 0, 0.008, 0, -half, 0.008, 0, half, 0.008],
        3
      )
    );
    const axisMat = new THREE.LineBasicMaterial({
      color: 0x38bdf8,
      transparent: true,
      opacity: 0.9
    });
    this.gridGroup.add(new THREE.LineSegments(axisGeo, axisMat));
  }

  /**
   * 3D Animated Rally Point Reticle under cursor in Goal Mode.
   */
  private createCursorReticle(): THREE.Group {
    const group = new THREE.Group();
    group.visible = false;

    // Glowing circle
    const ringGeo = new THREE.RingGeometry(0.4, 0.46, 32);
    const ringMat = new THREE.MeshBasicMaterial({
      color: 0x22c55e,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.85
    });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    group.add(ring);

    // Crosshairs
    const crossGeo = new THREE.BufferGeometry();
    crossGeo.setAttribute(
      'position',
      new THREE.Float32BufferAttribute(
        [-0.6, 0, 0.01, 0.6, 0, 0.01, 0, -0.6, 0.01, 0, 0.6, 0.01],
        3
      )
    );
    const crossMat = new THREE.LineBasicMaterial({ color: 0x00ffaa, linewidth: 2 });
    group.add(new THREE.LineSegments(crossGeo, crossMat));

    return group;
  }

  public update(options: {
    robots: MapRobot[];
    trails: Map<string, { x: number; y: number }[]>;
    showGrid: boolean;
    showTrails: boolean;
    showPlans: boolean;
    showSensors: boolean;
    showCostmap: boolean;
    showNetwork: boolean;
    costmapKind: 'local';
    time: number;
    getGroundZ?: (x: number, y: number) => number;
  }) {
    this.gridGroup.visible = options.showGrid;

    // Rebuild only when source data changes. Animation never recreates geometry.
    const changed = (key: string, value: unknown) => {
      const signature = JSON.stringify(value);
      if (this.signatures.get(key) === signature) return false;
      this.signatures.set(key, signature);
      return true;
    };
    const robots = options.robots;
    if (
      changed('goals', [
        options.showPlans,
        robots.map((r) => [r.robot_id, r.pose, r.goal, r.nav_status, r.mode])
      ])
    )
      this.updateGoals(robots, options.showPlans, options.time, options.getGroundZ);
    if (
      changed('paths', [
        options.showPlans,
        fleet.selected,
        robots.map((r) => [
          r.robot_id,
          r.nav_status,
          r.mode,
          r.goal,
          r.planned_path,
          r.global_planned_path,
          r.local_planned_path
        ])
      ])
    )
      this.updatePaths(robots, options.showPlans, options.getGroundZ);
    if (
      changed('trails', [
        options.showTrails,
        Array.from(options.trails).filter(([id]) => robots.some((r) => r.robot_id === id))
      ])
    )
      this.updateTrails(
        new Map(Array.from(options.trails).filter(([id]) => robots.some((r) => r.robot_id === id))),
        options.showTrails,
        options.getGroundZ
      );
    if (
      changed('detections', [
        review.proposals,
        review.entities,
        review.selected,
        review.highlighted
      ])
    )
      this.updateDetections(options.time, options.getGroundZ);
    if (
      changed('loops', [
        options.showPlans,
        mapStore.slamGraphs,
        robots.map((r) => [r.robot_id, r.pose])
      ])
    )
      this.updateLoopClosures(options.showPlans);
    this.updateDecals(options.showCostmap, options.showNetwork, options.costmapKind);
  }

  public invalidate() {
    this.signatures.clear();
  }

  private updateGoals(
    robots: MapRobot[],
    showPlans: boolean,
    time: number,
    getGroundZ?: (x: number, y: number) => number
  ) {
    this.clearGroup(this.goalsGroup);
    if (!showPlans) return;

    for (const robot of robots) {
      const isNavActive =
        robot.nav_status === 'active' || robot.mode === 'nav' || Boolean(robot.goal);
      if (!isNavActive || !robot.goal) continue;

      const color = new THREE.Color(fleet.colorOf(robot.robot_id));
      const goalX = robot.goal.x;
      const goalY = robot.goal.y;
      const goalZ = mapFrameZ(robot.goal, getGroundZ);
      // Robot pose Z may name the base/navigation origin rather than contact
      // ground. Keep this non-planner guide attached to the rendered surface.
      const robotGroundZ = getGroundZ?.(robot.pose.x, robot.pose.y) ?? 0;

      const beaconGroup = new THREE.Group();
      beaconGroup.position.set(goalX, goalY, goalZ);

      // Rotating waypoint rally beacon rings
      const outerRingGeo = new THREE.RingGeometry(0.38, 0.44, 24);
      const ringMat = new THREE.MeshBasicMaterial({
        color,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.85
      });
      const outerRing = new THREE.Mesh(outerRingGeo, ringMat);
      outerRing.rotation.z = time * 2;
      beaconGroup.add(outerRing);

      const innerRingGeo = new THREE.RingGeometry(0.18, 0.22, 16);
      const innerRing = new THREE.Mesh(innerRingGeo, ringMat);
      innerRing.rotation.z = -time * 3;
      beaconGroup.add(innerRing);

      // Vertical holographic light pillar
      const beamGeo = new THREE.CylinderGeometry(0.04, 0.04, 2.5, 12, 1, true);
      beamGeo.rotateX(Math.PI / 2);
      beamGeo.translate(0, 0, 1.25);
      const beamMat = new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity: 0.35,
        side: THREE.DoubleSide
      });
      const beam = new THREE.Mesh(beamGeo, beamMat);
      beaconGroup.add(beam);

      this.goalsGroup.add(beaconGroup);

      // Dashed route line from robot to goal
      const linePts = [
        new THREE.Vector3(robot.pose.x, robot.pose.y, robotGroundZ + 0.025),
        new THREE.Vector3(goalX, goalY, goalZ)
      ];
      const lineGeo = new THREE.BufferGeometry().setFromPoints(linePts);
      const lineMat = new THREE.LineDashedMaterial({
        color,
        dashSize: 0.35,
        gapSize: 0.2,
        transparent: true,
        opacity: 0.8
      });
      const line = new THREE.Line(lineGeo, lineMat);
      line.computeLineDistances();
      this.goalsGroup.add(line);
    }
  }

  private updatePaths(
    robots: MapRobot[],
    showPlans: boolean,
    getGroundZ?: (x: number, y: number) => number
  ) {
    this.clearGroup(this.pathsGroup);
    if (!showPlans) return;

    for (const robot of robots) {
      const isNavActive =
        robot.nav_status === 'active' || robot.mode === 'nav' || Boolean(robot.goal);
      if (!isNavActive) continue;

      const color = new THREE.Color(fleet.colorOf(robot.robot_id));
      const hasSplitPaths = Boolean(
        (robot.global_planned_path && robot.global_planned_path.length > 0) ||
        (robot.local_planned_path && robot.local_planned_path.length > 0)
      );
      const globalPath = hasSplitPaths
        ? robot.global_planned_path && robot.global_planned_path.length > 0
          ? robot.global_planned_path
          : robot.planned_path
        : robot.planned_path;
      const localPath =
        robot.local_planned_path && robot.local_planned_path.length > 0
          ? robot.local_planned_path
          : undefined;

      const selected = fleet.isSelected(robot.robot_id);
      if (globalPath && globalPath.length >= 2)
        this.addRoute(globalPath, color, selected ? 6 : 4, true, getGroundZ);
      if (localPath && localPath.length >= 2)
        this.addRoute(localPath, new THREE.Color(0xffffff), selected ? 5 : 3, false, getGroundZ);

    }
  }

  private addRoute(
    path: MapPoint3D[], color: THREE.Color, width: number,
    dashed: boolean, getGroundZ?: (x: number, y: number) => number
  ) {
    // Triangle-backed screen-space lines: WebGL's native linewidth is not portable.
    const positions = routePositions(path, getGroundZ);
    const geometry = new LineGeometry().setPositions(positions);
    for (const outline of [true, false]) {
      const material = new LineMaterial({
        color: outline ? 0x101827 : color.getHex(),
        linewidth: width + (outline ? 4 : 0),
        dashed, dashSize: 0.65, gapSize: 0.3,
        depthTest: false, depthWrite: false, toneMapped: false,
        transparent: true, opacity: outline ? 0.95 : 1
      });
      const line = new Line2(geometry, material);
      line.computeLineDistances();
      line.renderOrder = (dashed ? 40 : 42) + (outline ? 0 : 1);
      this.pathsGroup.add(line);
    }
  }

  private updateTrails(
    trails: Map<string, { x: number; y: number }[]>,
    showTrails: boolean,
    getGroundZ?: (x: number, y: number) => number
  ) {
    this.clearGroup(this.trailsGroup);
    if (!showTrails) return;

    for (const [robotId, pts] of trails.entries()) {
      if (pts.length < 2) continue;
      const color = new THREE.Color(fleet.colorOf(robotId));
      const v3s = pts.map((p) => {
        // Shared 2D trail history has no vertical pose authority. Draw it just
        // above measured ground; unlike planner routes, no physical Z is lost.
        const z = getGroundZ ? getGroundZ(p.x, p.y) : 0.0;
        return new THREE.Vector3(p.x, p.y, z + 0.025);
      });
      const geo = new THREE.BufferGeometry().setFromPoints(v3s);
      const mat = new THREE.LineBasicMaterial({
        color,
        transparent: true,
        opacity: 0.55
      });
      this.trailsGroup.add(new THREE.Line(geo, mat));
    }
  }

  private updateDetections(time: number, getGroundZ?: (x: number, y: number) => number) {
    this.clearGroup(this.detectionsGroup);

    const items = [...review.proposals, ...review.entities];
    for (const item of items) {
      const isProposal = 'provisional' in item || review.proposals.some((p) => p.id === item.id);
      const color = new THREE.Color(detectionCatalog.colorOf(item.class));
      const isSelected = review.selected === item.id || review.highlighted === item.id;
      const z = getGroundZ ? getGroundZ(item.position.x, item.position.y) : 0.0;

      const group = new THREE.Group();
      group.position.set(item.position.x, item.position.y, z + 0.35);
      group.userData = { detectionId: item.id, isDetection: true };

      // 3D Target crystal beacon (Octahedron)
      const crystalGeo = new THREE.OctahedronGeometry(isSelected ? 0.28 : 0.2);
      const crystalMat = new THREE.MeshStandardMaterial({
        color,
        emissive: color,
        emissiveIntensity: isSelected ? 0.6 : 0.25,
        roughness: 0.2,
        metalness: 0.8
      });
      const crystal = new THREE.Mesh(crystalGeo, crystalMat);
      crystal.rotation.y = time * 1.5;
      crystal.rotation.z = time * 0.8;
      crystal.userData = { detectionId: item.id, isDetection: true };
      group.add(crystal);

      // Base ground ring
      const ringGeo = new THREE.RingGeometry(0.25, 0.32, 16);
      const ringMat = new THREE.MeshBasicMaterial({
        color,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: isSelected ? 0.8 : 0.4
      });
      const ring = new THREE.Mesh(ringGeo, ringMat);
      ring.position.set(0, 0, -0.33);
      group.add(ring);

      this.detectionsGroup.add(group);
    }
  }

  public raycastDetection(ndc: THREE.Vector2, camera: THREE.Camera): string | null {
    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(ndc, camera);
    const hits = raycaster.intersectObjects(this.detectionsGroup.children, true);
    for (const hit of hits) {
      let cur: THREE.Object3D | null = hit.object;
      while (cur && cur !== this.detectionsGroup) {
        if (cur.userData?.detectionId) {
          return cur.userData.detectionId as string;
        }
        cur = cur.parent;
      }
    }
    return null;
  }

  private updateLoopClosures(showPlans: boolean) {
    this.clearGroup(this.loopClosuresGroup);
    if (!showPlans) return;

    const seen = new Set<string>();
    for (const [robotId, graph] of Object.entries(mapStore.slamGraphs)) {
      const a = fleet.get(robotId);
      if (!a) continue;
      for (const link of graph.inter_robot) {
        const key = [robotId, link.other].sort().join('|');
        if (seen.has(key)) continue;
        seen.add(key);
        const b = fleet.get(link.other);
        if (!b) continue;

        const pts = [
          new THREE.Vector3(a.pose.x, a.pose.y, 0.18),
          new THREE.Vector3(b.pose.x, b.pose.y, 0.18)
        ];
        const geo = new THREE.BufferGeometry().setFromPoints(pts);
        const mat = new THREE.LineDashedMaterial({
          color: 0x00d4ff,
          dashSize: 0.4,
          gapSize: 0.2,
          transparent: true,
          opacity: 0.85
        });
        const l = new THREE.Line(geo, mat);
        l.computeLineDistances();
        this.loopClosuresGroup.add(l);
      }
    }
  }

  private updateDecals(
    showCostmap: boolean,
    showNetwork: boolean,
    costmapKind: 'local'
  ) {
    // 3D Costmap ground decal
    const viewedCostmapRobotId =
      mapStore.viewMode === 'local' && mapStore.viewRobot
        ? mapStore.viewRobot
        : (fleet.selected[0] ?? fleet.robots[0]?.robot_id ?? null);
    const costmapLayer = viewedCostmapRobotId
      ? mapStore.costmapLayer(viewedCostmapRobotId, costmapKind)
      : null;

    if (showCostmap && costmapLayer && costmapLayer.canvas) {
      if (!this.costmapMesh) {
        const geo = new THREE.PlaneGeometry(1, 1);
        const mat = new THREE.MeshBasicMaterial({
          transparent: true,
          opacity: 0.65,
          side: THREE.DoubleSide,
          depthWrite: false, depthTest: false
        });
        this.costmapMesh = new THREE.Mesh(geo, mat);
        this.costmapMesh.renderOrder = 20;
        this.group.add(this.costmapMesh);
      }
      this.costmapMesh.visible = true;
      const material = this.costmapMesh.material as THREE.MeshBasicMaterial;
      if (!material.map || material.map.image !== costmapLayer.canvas) {
        material.map?.dispose();
        material.map = new THREE.CanvasTexture(costmapLayer.canvas);
        material.needsUpdate = true;
      }
      if (this.costmapMesh.userData.seq !== costmapLayer.seq) material.map.needsUpdate = true;
      this.costmapMesh.userData.seq = costmapLayer.seq;

      const pose = decalPose(costmapLayer.info, mapStore.info?.transforms?.[costmapLayer.robotId]);
      this.costmapMesh.scale.set(pose.width, pose.height, 1);
      this.costmapMesh.rotation.z = pose.yaw;
      this.costmapMesh.position.set(pose.x, pose.y, 0.2);
    } else if (this.costmapMesh) {
      this.costmapMesh.visible = false;
    }

    // 3D Network heatmap decal
    const networkLayer = mapStore.networkLayer;
    if (showNetwork && networkLayer && networkLayer.canvas) {
      if (!this.networkMesh) {
        const geo = new THREE.PlaneGeometry(1, 1);
        const mat = new THREE.MeshBasicMaterial({
          transparent: true,
          opacity: 0.6,
          side: THREE.DoubleSide,
          depthWrite: false
        });
        this.networkMesh = new THREE.Mesh(geo, mat);
        this.networkMesh.position.z = 0.014;
        this.group.add(this.networkMesh);
      }
      this.networkMesh.visible = true;
      const material = this.networkMesh.material as THREE.MeshBasicMaterial;
      if (!material.map || material.map.image !== networkLayer.canvas) {
        material.map?.dispose();
        material.map = new THREE.CanvasTexture(networkLayer.canvas);
        material.needsUpdate = true;
      }
      material.map.needsUpdate = true;

      const pose = decalPose(networkLayer.info, mapStore.info?.transforms?.[networkLayer.robotId]);
      this.networkMesh.scale.set(pose.width, pose.height, 1);
      this.networkMesh.rotation.z = pose.yaw;
      this.networkMesh.position.set(pose.x, pose.y, 0.21);
    } else if (this.networkMesh) {
      this.networkMesh.visible = false;
    }
  }

  private clearGroup(g: THREE.Group) {
    const geometries = new Set<THREE.BufferGeometry>(),
      materials = new Set<THREE.Material>();
    g.traverse((c) => {
      if (c instanceof THREE.Mesh || c instanceof THREE.Line) {
        geometries.add(c.geometry);
        for (const m of Array.isArray(c.material) ? c.material : [c.material]) materials.add(m);
      }
    });
    geometries.forEach((g) => g.dispose());
    materials.forEach((m) => {
      (m as THREE.MeshBasicMaterial).map?.dispose();
      m.dispose();
    });
    g.clear();
  }
  public dispose() {
    this.clearGroup(this.group);
    this.signatures.clear();
  }
}
