import * as THREE from 'three';
import { fleet } from '$lib/stores/fleet.svelte';
import { mapStore } from '$lib/stores/mapstore.svelte';
import { robotDisplayName } from '$lib/robotDisplayName';
import type { MapRobot } from '../map2d/mapLayers';

export interface Robot3DEntry {
  id: string;
  group: THREE.Group;
  chevronMesh: THREE.Mesh;
  edges: THREE.LineSegments;
  selectionRing: THREE.Group;
  selectedRingMesh: THREE.LineSegments;
  sensorCone: THREE.Mesh;
  footprintMesh: THREE.LineSegments;
  labelSprite: THREE.Sprite | null;
}

export class Robot3DManager {
  public group = new THREE.Group();
  private entries = new Map<string, Robot3DEntry>();

  /**
   * Updates all 3D robots from the current fleet state.
   */
  public update(
    robots: MapRobot[],
    options: {
      showSensors: boolean;
      showLabels: boolean;
      time: number;
      getGroundZ?: (x: number, y: number) => number;
    }
  ) {
    const activeIds = new Set<string>();

    for (const robot of robots) {
      activeIds.add(robot.robot_id);
      let entry = this.entries.get(robot.robot_id);
      if (!entry) {
        entry = this.createRobot3D(robot);
        this.entries.set(robot.robot_id, entry);
        this.group.add(entry.group);
      }

      const colorHex = fleet.colorOf(robot.robot_id);
      const isSelected = fleet.isSelected(robot.robot_id);

      // Position in world coordinates: x, y, floor clearance
      // Snaps to real terrain or cave ground elevation if available
      const groundZ = options.getGroundZ ? options.getGroundZ(robot.pose.x, robot.pose.y) : 0.0;
      entry.group.position.set(robot.pose.x, robot.pose.y, groundZ);
      entry.group.rotation.set(0, 0, robot.pose.yaw);

      // 3D Selection Reticle
      entry.selectionRing.visible = true;
      if (isSelected) {
        entry.selectedRingMesh.visible = true;
        // Rotate the tactical corner reticle slowly
        entry.selectedRingMesh.rotation.z = options.time * 0.8;
        // Pulse scale slightly
        const pulse = 1.0 + 0.05 * Math.sin(options.time * 4);
        entry.selectedRingMesh.scale.set(pulse, pulse, 1);
      } else {
        entry.selectedRingMesh.visible = false;
      }

      // Sensor cone
      entry.sensorCone.visible = options.showSensors;
      entry.footprintMesh.visible = options.showSensors;

      // Label
      if (entry.labelSprite) {
        entry.labelSprite.visible = options.showLabels;
      }
    }

    // Remove any robots that have departed
    for (const [id, entry] of this.entries.entries()) {
      if (!activeIds.has(id)) {
        this.group.remove(entry.group);
        this.disposeEntry(entry);
        this.entries.delete(id);
      }
    }
  }

  private createRobot3D(robot: MapRobot): Robot3DEntry {
    const id = robot.robot_id;
    const colorHex = fleet.colorOf(id);
    const threeColor = new THREE.Color(colorHex);

    const robotGroup = new THREE.Group();
    robotGroup.name = `robot_${id}`;
    robotGroup.userData = { robotId: id, isRobot: true };

    // 1. Extruded Chevron Symbol (identical to 2D shape, extruded to 3D)
    // 2D shape points: forward tip at (0.50, 0), wings at (-0.32, 0.30) & (-0.32, -0.30), notch at (-0.18, 0)
    const chevronShape = new THREE.Shape();
    chevronShape.moveTo(0.52, 0);
    chevronShape.lineTo(-0.34, 0.31);
    chevronShape.lineTo(-0.19, 0);
    chevronShape.lineTo(-0.34, -0.31);
    chevronShape.closePath();

    const extrudeSettings: THREE.ExtrudeGeometryOptions = {
      depth: 0.16,
      bevelEnabled: true,
      bevelSegments: 2,
      steps: 1,
      bevelSize: 0.02,
      bevelThickness: 0.02
    };

    const chevronGeo = new THREE.ExtrudeGeometry(chevronShape, extrudeSettings);
    // Raise chevron above the chassis
    chevronGeo.translate(0, 0, 0.12);

    const chevronMat = new THREE.MeshStandardMaterial({
      color: threeColor,
      metalness: 0.45,
      roughness: 0.28,
      emissive: threeColor,
      emissiveIntensity: 0.25
    });

    const chevronMesh = new THREE.Mesh(chevronGeo, chevronMat);
    chevronMesh.castShadow = true;
    chevronMesh.receiveShadow = true;
    chevronMesh.userData = { robotId: id, isRobot: true };
    robotGroup.add(chevronMesh);

    // 2. Crisp White Edge Bevel (matching 2D strokeStyle = '#ffffff')
    const edgesGeo = new THREE.EdgesGeometry(chevronGeo, 24);
    const edgesMat = new THREE.LineBasicMaterial({
      color: 0xffffff,
      transparent: true,
      opacity: 0.95
    });
    const edges = new THREE.LineSegments(edgesGeo, edgesMat);
    robotGroup.add(edges);

    // 3. Glowing cockpit visor / headlights at tip
    const visorGeo = new THREE.SphereGeometry(0.05, 8, 8);
    const visorMat = new THREE.MeshBasicMaterial({
      color: 0x70ffff
    });
    const visorMesh = new THREE.Mesh(visorGeo, visorMat);
    visorMesh.position.set(0.42, 0, 0.22);
    robotGroup.add(visorMesh);

    // 4. Extruded chassis / base platform
    const baseShape = new THREE.Shape();
    baseShape.moveTo(0.38, 0.28);
    baseShape.lineTo(-0.38, 0.28);
    baseShape.lineTo(-0.38, -0.28);
    baseShape.lineTo(0.38, -0.28);
    baseShape.closePath();

    const baseGeo = new THREE.ExtrudeGeometry(baseShape, {
      depth: 0.1,
      bevelEnabled: true,
      bevelSegments: 1,
      bevelSize: 0.015,
      bevelThickness: 0.015
    });
    baseGeo.translate(0, 0, 0.01);
    const baseMat = new THREE.MeshStandardMaterial({
      color: 0x334155, // Clean slate 700 chassis (not pitch dark)
      metalness: 0.5,
      roughness: 0.35
    });
    const baseMesh = new THREE.Mesh(baseGeo, baseMat);
    baseMesh.castShadow = true;
    baseMesh.userData = { robotId: id, isRobot: true };
    robotGroup.add(baseMesh);

    // 5. 3D Tactical Selection Reticle on the floor
    const selectionRing = new THREE.Group();
    selectionRing.position.set(0, 0, 0.01);

    // Base subtle ground ring
    const groundRingGeo = new THREE.RingGeometry(0.55, 0.6, 32);
    const groundRingMat = new THREE.MeshBasicMaterial({
      color: threeColor,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.35
    });
    const groundRingMesh = new THREE.Mesh(groundRingGeo, groundRingMat);
    selectionRing.add(groundRingMesh);

    // 3D Tactical Corner Reticle (pulsing green/team brackets when selected)
    const bracketGeo = new THREE.BufferGeometry();
    const bracketPts: number[] = [];
    const R = 0.72;
    const cornerSize = 0.2;

    // 4 corner L-brackets
    const corners = [
      [R, R],
      [-R, R],
      [-R, -R],
      [R, -R]
    ];
    for (const [cx, cy] of corners) {
      const sx = Math.sign(cx);
      const sy = Math.sign(cy);
      bracketPts.push(cx, cy - sy * cornerSize, 0, cx, cy, 0);
      bracketPts.push(cx, cy, 0, cx - sx * cornerSize, cy, 0);
    }
    bracketGeo.setAttribute('position', new THREE.Float32BufferAttribute(bracketPts, 3));
    const bracketMat = new THREE.LineBasicMaterial({
      color: 0x00ff88, // Tactical selection green
      linewidth: 2,
      transparent: true,
      opacity: 0.95
    });
    const selectedRingMesh = new THREE.LineSegments(bracketGeo, bracketMat);
    selectedRingMesh.visible = false;
    selectionRing.add(selectedRingMesh);

    robotGroup.add(selectionRing);

    // 6. Sensor FOV Arc (projected onto ground)
    const sensorGeo = new THREE.RingGeometry(0.1, 2.0, 24, 1, -0.6, 1.2);
    const sensorMat = new THREE.MeshBasicMaterial({
      color: threeColor,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.12
    });
    const sensorCone = new THREE.Mesh(sensorGeo, sensorMat);
    sensorCone.position.set(0, 0, 0.015);
    sensorCone.visible = false;
    robotGroup.add(sensorCone);

    // 7. Footprint Boundary Outline
    const fpGeo = new THREE.BufferGeometry();
    const fpPts = [
      0.38, 0.28, 0.02, -0.38, 0.28, 0.02, -0.38, 0.28, 0.02, -0.38, -0.28, 0.02, -0.38, -0.28,
      0.02, 0.38, -0.28, 0.02, 0.38, -0.28, 0.02, 0.38, 0.28, 0.02
    ];
    fpGeo.setAttribute('position', new THREE.Float32BufferAttribute(fpPts, 3));
    const fpMat = new THREE.LineBasicMaterial({
      color: threeColor,
      transparent: true,
      opacity: 0.6
    });
    const footprintMesh = new THREE.LineSegments(fpGeo, fpMat);
    footprintMesh.visible = false;
    robotGroup.add(footprintMesh);

    // 8. 3D Floating Nameplate Label
    const labelSprite = this.createNameplateSprite(id, colorHex);
    labelSprite.position.set(0, 0, 0.72);
    robotGroup.add(labelSprite);

    return {
      id,
      group: robotGroup,
      chevronMesh,
      edges,
      selectionRing,
      selectedRingMesh,
      sensorCone,
      footprintMesh,
      labelSprite
    };
  }

  private createNameplateSprite(id: string, colorHex: string): THREE.Sprite {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 64;
    const ctx = canvas.getContext('2d');
    if (ctx) {
      ctx.fillStyle = 'rgba(15, 18, 24, 0.82)';
      ctx.roundRect(16, 12, 224, 40, 8);
      ctx.fill();
      ctx.strokeStyle = colorHex;
      ctx.lineWidth = 2;
      ctx.stroke();

      ctx.font = 'bold 20px ui-sans-serif, system-ui, sans-serif';
      ctx.fillStyle = colorHex;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(robotDisplayName(id), 128, 32);
    }

    const texture = new THREE.CanvasTexture(canvas);
    texture.minFilter = THREE.LinearFilter;
    const spriteMat = new THREE.SpriteMaterial({
      map: texture,
      transparent: true,
      depthTest: false
    });
    const sprite = new THREE.Sprite(spriteMat);
    sprite.scale.set(1.4, 0.35, 1.0);
    return sprite;
  }

  private disposeEntry(entry: Robot3DEntry) {
    const geometries = new Set<THREE.BufferGeometry>(),
      materials = new Set<THREE.Material>();
    entry.group.traverse((c) => {
      if (c instanceof THREE.Mesh || c instanceof THREE.Line || c instanceof THREE.Sprite) {
        if ('geometry' in c) geometries.add(c.geometry);
        for (const m of Array.isArray(c.material) ? c.material : [c.material]) materials.add(m);
      }
    });
    geometries.forEach((g) => g.dispose());
    materials.forEach((m) => {
      (m as THREE.MeshBasicMaterial).map?.dispose();
      m.dispose();
    });
    entry.group.clear();
  }

  public dispose() {
    for (const entry of this.entries.values()) {
      this.group.remove(entry.group);
      this.disposeEntry(entry);
    }
    this.entries.clear();
  }
}
