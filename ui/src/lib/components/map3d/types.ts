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

export type Map3DRenderMode = 'points' | 'mesh' | 'both';
export type Map3DColorMode = 'robot' | 'elevation';

export interface Map3DOptions {
  showGrid: boolean;
  showTrails: boolean;
  showLabels: boolean;
  showSensors: boolean;
  showPlans: boolean;
  showNetwork: boolean;
  showCostmap: boolean;
  costmapKind: 'global' | 'local';
  ceilingCutoff: number;
  renderMode: Map3DRenderMode;
  colorMode: Map3DColorMode;
  pointSize: number;
}
