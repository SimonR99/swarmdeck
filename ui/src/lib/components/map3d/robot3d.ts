import * as THREE from 'three';
import { fleet } from '$lib/stores/fleet.svelte';
import { robotDisplayName } from '$lib/robotDisplayName';
import type { MapRobot } from '../map2d/mapLayers';
import { RobotPresenceTracker } from './robotPresence';

export interface Robot3DEntry {
  id: string;
  group: THREE.Group;
  marker: THREE.Group;
  chevronMesh: THREE.Mesh;
  selectionRing: THREE.Group;
  selectedRingMesh: THREE.Mesh;
  sensorCone: THREE.Mesh;
  footprintMesh: THREE.LineSegments;
  labelSprite: THREE.Sprite | null;
}

export class Robot3DManager {
  public group = new THREE.Group();
  /**
   * A selected robot's reticle pulses with the clock, so the scene has to keep
   * being drawn for it. It is the only marker here that animates on its own.
   */
  public animating = false;
  private entries = new Map<string, Robot3DEntry>();
  private presence = new RobotPresenceTracker(3);

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
      markerScale?: (position: THREE.Vector3, selected: boolean) => number;
      sourceIdentity?: string;
    }
  ) {
    const presence = this.presence.update(
      robots.map((robot) => robot.robot_id), options.time, options.sourceIdentity ?? 'live'
    );
    if (presence.reset) {
      for (const entry of this.entries.values()) {
        this.group.remove(entry.group);
        this.disposeEntry(entry);
      }
      this.entries.clear();
    }

    this.animating = false;
    for (const robot of robots) {
      let entry = this.entries.get(robot.robot_id);
      if (!entry) {
        entry = this.createRobot3D(robot);
        this.entries.set(robot.robot_id, entry);
        this.group.add(entry.group);
      }

      const isSelected = fleet.isSelected(robot.robot_id);
      if (isSelected) this.animating = true;

      // Position in world coordinates: x, y, floor clearance
      // Snaps to real terrain or cave ground elevation if available
      const groundZ = options.getGroundZ ? options.getGroundZ(robot.pose.x, robot.pose.y) : 0.0;
      entry.group.position.set(robot.pose.x, robot.pose.y, groundZ);
      entry.group.rotation.set(0, 0, robot.pose.yaw);

      const scale = options.markerScale?.(entry.group.position, isSelected) ?? 1;
      entry.marker.scale.setScalar(scale);

      // 3D Selection Reticle
      entry.selectionRing.visible = isSelected;
      if (isSelected) {
        entry.selectedRingMesh.visible = true;
        entry.selectedRingMesh.rotation.z = 0;
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
        entry.labelSprite.visible = options.showLabels || isSelected;
        entry.labelSprite.scale.set(1.8 * scale, 0.45 * scale, 1);
        entry.labelSprite.position.z = 0.85 * scale;
        (entry.labelSprite.material as THREE.SpriteMaterial).color.set(isSelected ? 0xffffff : 0xbac6d6);
      }
    }

    // Membership and live replica snapshots can briefly miss a robot while a
    // new frame is published. Retain the last pose through those short gaps.
    for (const [id, entry] of this.entries.entries()) {
      if (!presence.visible.has(id)) {
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

    const marker = new THREE.Group();
    robotGroup.add(marker);

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

    // Use the transparent render queue at full opacity so terrain and splats
    // cannot cover the fill while leaving only its outline visible.
    const chevronMat = new THREE.MeshBasicMaterial({
      color: threeColor,
      transparent: true, opacity: 1,
      toneMapped: false, fog: false,
      depthTest: false, depthWrite: false
    });

    const chevronMesh = new THREE.Mesh(chevronGeo, chevronMat);
    chevronMesh.renderOrder = 91;
    chevronMesh.castShadow = false;
    chevronMesh.userData = { robotId: id, isRobot: true };
    marker.add(chevronMesh);

    // 3. Glowing cockpit visor / headlights at tip
    const visorGeo = new THREE.SphereGeometry(0.05, 8, 8);
    const visorMat = new THREE.MeshBasicMaterial({
      color: 0xffffff, transparent: true, opacity: 1,
      toneMapped: false, fog: false, depthTest: false, depthWrite: false
    });
    const visorMesh = new THREE.Mesh(visorGeo, visorMat);
    visorMesh.position.set(0.42, 0, 0.22);
    visorMesh.renderOrder = 93;
    marker.add(visorMesh);

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
    const baseMat = new THREE.MeshBasicMaterial({
      color: 0x101827,
      transparent: true, opacity: 1, toneMapped: false, fog: false,
      depthTest: false, depthWrite: false
    });
    const baseMesh = new THREE.Mesh(baseGeo, baseMat);
    baseMesh.renderOrder = 90;
    baseMesh.castShadow = false;
    baseMesh.userData = { robotId: id, isRobot: true };
    marker.add(baseMesh);

    // 5. 3D Tactical Selection Reticle on the floor
    const selectionRing = new THREE.Group();
    selectionRing.position.set(0, 0, 0.15);

    // Base subtle ground ring
    const groundRingGeo = new THREE.RingGeometry(0.64, 0.86, 32);
    const groundRingMat = new THREE.MeshBasicMaterial({
      color: threeColor,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 1, depthTest: false, depthWrite: false, toneMapped: false, fog: false
    });
    const groundRingMesh = new THREE.Mesh(groundRingGeo, groundRingMat);
    groundRingMesh.renderOrder = 95;
    const backing = new THREE.Mesh(
      new THREE.RingGeometry(0.58, 0.92, 32),
      new THREE.MeshBasicMaterial({
        color: 0x101827, side: THREE.DoubleSide, transparent: true, opacity: 1,
        depthTest: false, depthWrite: false, toneMapped: false, fog: false
      })
    );
    backing.renderOrder = 94;
    selectionRing.add(backing, groundRingMesh);

    // 3D Tactical Corner Reticle (pulsing green/team brackets when selected)
    const bracketPts: number[] = [];
    const rect = (x: number, y: number, w: number, h: number) => {
      bracketPts.push(x,y,0, x+w,y,0, x+w,y+h,0, x,y,0, x+w,y+h,0, x,y+h,0);
    };
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
      rect(sx > 0 ? 0.76 : -1.06, sy > 0 ? 0.97 : -1.06, 0.3, 0.09);
      rect(sx > 0 ? 0.97 : -1.06, sy > 0 ? 0.76 : -1.06, 0.09, 0.3);
    }
    const bracketGeo = new THREE.BufferGeometry();
    bracketGeo.setAttribute('position', new THREE.Float32BufferAttribute(bracketPts, 3));
    const selectedRingMesh = new THREE.Mesh(bracketGeo, new THREE.MeshBasicMaterial({
      color: 0xffffff, side: THREE.DoubleSide, transparent: true, opacity: 1,
      depthTest: false, depthWrite: false, toneMapped: false, fog: false
    }));
    selectedRingMesh.renderOrder = 96;
    selectedRingMesh.visible = false;
    selectionRing.add(selectedRingMesh);

    marker.add(selectionRing);

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
      marker,
      chevronMesh,
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
      depthTest: false, depthWrite: false
    });
    const sprite = new THREE.Sprite(spriteMat);
    sprite.renderOrder = 100;
    sprite.scale.set(1.8, 0.45, 1.0);
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
    this.presence.clear();
    this.animating = false;
  }
}
