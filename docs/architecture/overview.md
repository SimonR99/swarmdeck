# SwarmDeck Architecture

SwarmDeck connects heterogeneous multi-robot fleets to a responsive web dashboard without requiring ROS dependencies on the host server or browser.

```text
┌────────────────────────────────────────────────────────┐
│                   SwarmDeck UI (Browser)               │
│   Svelte 5 · Canvas 2D / Three.js 3D · WebRTC / WHEP   │
└───────────────────────────▲────────────────────────────┘
                            │ WebSocket / REST / WebRTC
┌───────────────────────────▼────────────────────────────┐
│               SwarmDeck Server (FastAPI)               │
│   Fleet Manager · MapService (2D Merge) · Bus · API    │
└───────────────────────────▲────────────────────────────┘
                            │ SwarmDeck Wire Protocol (WS)
       ┌────────────────────┼────────────────────┐
       │                    │                    │
┌──────▼──────┐      ┌──────▼──────┐      ┌──────▼──────┐
│ ROS 2 Bridge│      │ ROS 1 Bridge│      │ Mock / Sim  │
│  (Botman /  │      │(Scout Mini) │      │  (Gazebo /  │
│ Spot / Aslan)      │             │      │  Synthetic) │
└──────┬──────┘      └──────┬──────┘      └─────────────┘
       │                    │
┌──────▼──────┐      ┌──────▼──────┐
│  Perception │      │ Media Stream│
│   Sidecar   │      │(RTSP/Media- │
│ (YOLOE-26n) │      │    MTX)     │
└─────────────┘      └─────────────┘
```

---

## 1. System Components

### A. SwarmDeck Server (`server/`)
- **Language & Runtime**: Python 3.11+ using FastAPI, Starlette WebSockets, and NumPy.
- **ROS Independence**: Pure Python with no `rclpy` or `rospy` dependencies.
- **Core Services**:
  - `FleetManager`: Tracks robot registrations, telemetry, heartbeats, and status.
  - `MapService`: Accumulates 2D lidar scans via raytracing, dynamically expands map boundaries, and merges multi-robot grids using voting or static priors.
  - `EventBus`: Decoupled publish/subscribe bus for robot telemetry, detections, and operator commands.

### B. User Interface (`ui/`)
- **Framework**: Svelte 5 (Runes mode), Vite, Tailwind CSS 4, Lucide icons.
- **Visualizations**:
  - 2D Canvas: High-performance occupancy grid renderer with dynamic canvas reallocation and sub-pixel delta patching.
  - 3D Viewer: Three.js point cloud renderer for robot lidar scans and detection bounding volumes.
- **Media**: Low-latency video via WebRTC (WHEP) with fallback JPEG polling, real-time FPS and ping diagnostic HUD overlay.

### C. Protocol Adapters (`adapters/`)
- **ROS 1 Adapter** (`adapters/adapter_ros1/`): Connects to ROS 1 Noetic nodes (e.g., Scout Mini / LVI-SAM).
- **ROS 2 Adapter** (`adapters/adapter_ros2/`): Connects to ROS 2 Humble/Jazzy nodes (e.g., Botman, Aslan, Spot).
- **Simulation Adapter** (`adapters/adapter_sim/`): Interfaces with Gazebo simulation environments.
- **Mock Adapter** (`adapters/adapter_mock/`): Pure synthetic multi-robot generator for UI testing without ROS or GPU requirements.

---

## 2. Coordinate Frames & Transforms

SwarmDeck standardizes coordinate frames across heterogeneous robots:

| Frame | Scope | Description |
| :--- | :--- | :--- |
| `world` | Global | Common reference frame anchored at the origin of the primary reference robot. |
| `map` | Robot Local | SLAM-estimated fixed map frame for a specific robot. |
| `odom` | Robot Local | Continuous wheel/inertial odometry frame. |
| `base_link` | Robot Body | Physical robot chassis geometric center. |
| `sensor` | Robot Sensor | Sensor optical or lidar frames (e.g., `os_lidar`, `camera_color_optical_frame`). |

### 2D Global Map Merging & Dynamic Expansion
1. **Raytracing Accumulator**: Converts 2D/3D point clouds into raytraced occupancy grids (`UNKNOWN`, `FREE`, `OCCUPIED`).
2. **Dynamic Bounds Expansion**: If a robot explores beyond the initial map window, the accumulator and `MapService` automatically expand their extents in chunk increments, broadcasting resized geometry (`width`, `height`, `origin`) to the frontend.
3. **Multi-Robot Alignment**:
   - `auto`: Computes pairwise SE(2) cross-correlations over occupied/free cells with coverage support gating.
   - `static`: Uses configured start poses from scenario/robot configuration files.
