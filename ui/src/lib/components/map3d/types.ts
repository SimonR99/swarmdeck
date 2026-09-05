import type { MapRobot } from '../map2d/mapLayers';

export interface VoxelPoint {
  x: number;
  y: number;
  z: number;
  ownerIndex: number;
}

export interface CloudBounds {
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
  minZ: number;
  maxZ: number;
}

export interface Starcraft3DOptions {
  showGrid: boolean;
  showTrails: boolean;
  showLabels: boolean;
  showSensors: boolean;
  showPlans: boolean;
  showNetwork: boolean;
  showCostmap: boolean;
  costmapKind: 'global' | 'local';
  ceilingCutoff: number;
}
