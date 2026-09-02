"""Canonical odometry registry and configuration contract.

Defines standard odometry backends, their sensor requirements, default topics,
TF ownership rules, and configuration resolution for both simulated fleets
and physical robots.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping


@dataclass(frozen=True)
class OdometrySpec:
    """Specification of an odometry backend.

    Parameters
    ----------
    name:
        Canonical identifier (e.g. 'fast_livo2', 'superodometry', 'ekf', 'native', 'drift', 'icp').
    source_type:
        Algorithmic classification:
        - 'livo': LiDAR-Inertial-Visual Odometry (e.g. Fast-LIVO2)
        - 'lio': LiDAR-Inertial Odometry (e.g. SuperOdometry)
        - 'fused_wheels_imu': EKF state estimator fusing wheels and IMU
        - 'vendor': Manufacturer onboard kinematics/estimator (Spot, Unitree G1)
        - 'synthetic': Perturbed simulation odometry (ARGoS drift)
        - 'icp': Point cloud scan matcher (e.g. RTAB-Map icp_odometry)
    topic:
        Default ROS odometry topic name (e.g. 'odometry', 'laser_odometry', 'odom').
    publishes_tf:
        Whether the odometry provider broadcasts `odom -> base_link` transform.
        If False, an external bridge/sidecar (such as `odom_tf`) must publish it.
    requires_sensors:
        Tuple of sensor types required for this estimator.
    medium:
        Transport medium for simulation exchange (e.g. 'uf' for Unix domain socket).
    implementation:
        ARGoS controller implementation type (e.g. 'external' or 'drift').
    description:
        Human-readable summary of the estimator and its intended deployment.
    """

    name: str
    source_type: str
    topic: str
    publishes_tf: bool
    requires_sensors: tuple[str, ...] = field(default_factory=tuple)
    medium: str = ""
    implementation: str = "external"
    description: str = ""


# Canonical profiles supported across simulation and hardware.
ODOMETRY_PROFILES: dict[str, OdometrySpec] = {
    "fast_livo2": OdometrySpec(
        name="fast_livo2",
        source_type="livo",
        topic="odometry",
        publishes_tf=True,
        requires_sensors=("lidar", "imu", "camera"),
        medium="uf",
        implementation="external",
        description="Direct LiDAR-Inertial-Visual Odometry running in ROS 2 container over Unix socket",
    ),
    "superodometry": OdometrySpec(
        name="superodometry",
        source_type="lio",
        topic="laser_odometry",
        publishes_tf=False,  # publishes /laser_odometry; odom_tf sidecar handles TF
        requires_sensors=("lidar", "imu"),
        medium="",
        implementation="external",
        description="IMU-first LiDAR-inertial estimator deployed on physical Bunker platforms (Aslan, Botman)",
    ),
    "ekf": OdometrySpec(
        name="ekf",
        source_type="fused_wheels_imu",
        topic="odometry/filtered",
        publishes_tf=True,
        requires_sensors=("wheels", "imu"),
        medium="",
        implementation="external",
        description="robot_localization EKF fusing wheel encoders and IMU gyro/accel",
    ),
    "native": OdometrySpec(
        name="native",
        source_type="vendor",
        topic="odom",
        publishes_tf=True,
        requires_sensors=("wheels",),
        medium="",
        implementation="external",
        description="Proprietary manufacturer onboard locomotion estimator (Boston Dynamics Spot, Unitree G1)",
    ),
    "drift": OdometrySpec(
        name="drift",
        source_type="synthetic",
        topic="odometry",
        publishes_tf=True,
        requires_sensors=(),
        medium="",
        implementation="drift",
        description="ARGoS synthetic Gaussian drift model for rapid lightweight simulation",
    ),
    "icp": OdometrySpec(
        name="icp",
        source_type="icp",
        topic="icp_odometry",
        publishes_tf=True,
        requires_sensors=("lidar",),
        medium="",
        implementation="external",
        description="Scan matching point-cloud registration odometry (RTAB-Map 3D CSLAM)",
    ),
}

DEFAULT_SIM_ODOMETRY = "fast_livo2"
DEFAULT_HARDWARE_ODOMETRY = "superodometry"


def get_odometry_spec(name: str) -> OdometrySpec:
    """Look up an odometry specification by canonical name.

    Raises ValueError with list of available options if name is unknown.
    """
    spec = ODOMETRY_PROFILES.get(name)
    if spec is None:
        valid = ", ".join(sorted(ODOMETRY_PROFILES))
        raise ValueError(f"unknown odometry profile {name!r}; valid profiles: {valid}")
    return spec


def resolve_odometry_types(
    fleet_cfg: Mapping[str, Any] | None,
    count: int,
    prefix: str = "robot_",
    default_override: str | None = None,
) -> list[str]:
    """Resolve the odometry system for each robot in a fleet.

    Resolution precedence:
    1. If `default_override` is given (e.g. from CLI flag or env var):
       - If `fleet.odometry_types` explicitly overrides a specific robot, that override wins
         unless default_override was an explicit per-robot mandate.
       - The default fallback is `default_override`.
    2. Otherwise, `fleet.odometry` is the fleet default (fallback: DEFAULT_SIM_ODOMETRY).
    3. Any robot listed in `fleet.odometry_types` overrides the default for its ID.

    Example YAML:
    ```yaml
    fleet:
      odometry: fast_livo2
      odometry_types:
        robot_0: fast_livo2
        robot_1: drift
    ```
    """
    fleet_cfg = fleet_cfg or {}
    base_default = fleet_cfg.get("odometry") or DEFAULT_SIM_ODOMETRY
    default = default_override if default_override else base_default
    if default not in ODOMETRY_PROFILES:
        valid = ", ".join(sorted(ODOMETRY_PROFILES))
        raise ValueError(f"invalid default odometry profile {default!r}; valid: {valid}")

    per_robot = fleet_cfg.get("odometry_types") or {}
    expected_ids = {f"{prefix}{i}" for i in range(count)}
    unknown = set(per_robot) - expected_ids
    if unknown:
        raise ValueError(
            f"fleet.odometry_types names robots that are not in this fleet: {sorted(unknown)}"
        )

    resolved: list[str] = []
    for i in range(count):
        rid = f"{prefix}{i}"
        chosen = per_robot.get(rid, default)
        if chosen not in ODOMETRY_PROFILES:
            valid = ", ".join(sorted(ODOMETRY_PROFILES))
            raise ValueError(
                f"invalid odometry profile {chosen!r} for {rid}; valid: {valid}"
            )
        resolved.append(chosen)

    return resolved
