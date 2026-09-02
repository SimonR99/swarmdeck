#!/usr/bin/env python3
"""Bridges a live ARGoS simulation to Fast-LIVO2, in lockstep.

ARGoS connects to the Unix domain socket below, sends sensor data per simulation
tick (IMU, LiDAR scans, camera frames), and receives Fast-LIVO2's estimated
odometry pose in return.

Architecture:
    ARGoS ── socket ──► fast_livo_link ──► /{robot}/{imu,points,color/image_raw}
                             ▲                           │
                             │                  fast_livo2_node (one per robot)
                             └──── /{robot}/Odometry ────┘

Wire protocol (little-endian), matching CExternalEstimatorMedium::Update:
  "AEBR", u32 tick, u32 ticks_per_second, u8 lockstep, u32 robot_count
  per robot:
    u8 id_len, id bytes
    u8 has_frame; if set: u32 w, u32 h, f32 vertical_fov_deg,
                          u8 rgb[w*h*3],
                          u8 has_depth, if set f32 depth[w*h]
    u8 has_scan;  if set: u32 n_points, then per point
                          f32 x, f32 y, f32 z, f32 intensity,
                          f64 offset_ns, u8 tag, u8 line
    u8 has_wheels; if set: f64 wheel_pose[7]  (x y z qw qx qy qz)
    f64 imu[6]                                (wx wy wz ax ay az)

  reply: "ACK\\0", u32 pose_count, per pose:
    u8 id_len, id bytes, u32 tick, f64 pose[7], f64 twist[6], u8 valid
"""

from __future__ import annotations

import argparse
import math
import os
import socket
import struct
import sys
import time
import zlib

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image, CompressedImage, CameraInfo, Imu, PointCloud2, PointField
    from nav_msgs.msg import Odometry, Path
    from rosgraph_msgs.msg import Clock
except ImportError:
    rclpy = None
    Node = object
    QoSProfile = ReliabilityPolicy = HistoryPolicy = None
    Image = CompressedImage = CameraInfo = Imu = PointCloud2 = PointField = None
    Odometry = Path = Clock = None

MAGIC = b"AEBR"
ACK = b"ACK\0"

POINT_STEP = 26
if PointField is not None:
    POINT_FIELDS = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="timestamp", offset=16, datatype=PointField.FLOAT64, count=1),
        PointField(name="tag", offset=24, datatype=PointField.UINT8, count=1),
        PointField(name="line", offset=25, datatype=PointField.UINT8, count=1),
    ]
else:
    POINT_FIELDS = []


def recv_exactly(sock: socket.socket, n: int) -> bytes | None:
    """Reads exactly n bytes from socket, returning None if disconnected."""
    if n == 0:
        return b""
    chunks: list[bytes] = []
    got = 0
    while got < n:
        b = sock.recv(min(1 << 20, n - got))
        if not b:
            return None
        chunks.append(b)
        got += len(b)
    return b"".join(chunks)


def png_encode_rgb(w: int, h: int, rgb: bytes, bgr: bool = True) -> bytes:
    """Lossless PNG encoder for raw RGB/BGR frame buffers."""
    if bgr:
        buf = bytearray(rgb)
        buf[0::3], buf[2::3] = buf[2::3], buf[0::3]
        rgb = bytes(buf)

    raw = bytearray()
    stride = w * 3
    for y in range(h):
        raw.append(0)
        raw += rgb[y * stride : (y + 1) * stride]

    def chunk(tag: bytes, payload: bytes) -> bytes:
        out = struct.pack(">I", len(payload)) + tag + payload
        return out + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)

    header = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(bytes(raw), 1))
        + chunk(b"IEND", b"")
    )


class RobotIO:
    """Publishers and odometry subscriber for a single robot."""

    def __init__(self, node: Node, robot: str):
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.robot = robot
        self.frame_id = f"{robot}/base_link"

        self.color_compressed = node.create_publisher(
            CompressedImage, f"/{robot}/color/image_raw/compressed", qos
        )
        self.color_raw = node.create_publisher(
            Image, f"/{robot}/color/image_raw", qos
        )
        self.info = node.create_publisher(
            CameraInfo, f"/{robot}/color/camera_info", qos
        )
        self.points = node.create_publisher(
            PointCloud2, f"/{robot}/points", qos
        )
        self.imu = node.create_publisher(
            Imu, f"/{robot}/imu", qos
        )
        self.wheel = node.create_publisher(
            Odometry, f"/{robot}/odom", qos
        )

        estimate_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.estimate: tuple[int, tuple[float, float, float], tuple[float, float, float, float], tuple[float, ...]] | None = None

        # Subscribe to standard Fast-LIVO2 and Fast-LIO odometry output topics
        self.sub_odom1 = node.create_subscription(
            Odometry, f"/{robot}/Odometry", self.on_odometry, estimate_qos
        )
        self.sub_odom2 = node.create_subscription(
            Odometry, f"/{robot}/odom_lidar", self.on_odometry, estimate_qos
        )
        self.sub_odom3 = node.create_subscription(
            Odometry, f"/{robot}/aft_mapped_to_init", self.on_odometry, estimate_qos
        )
        self.sub_path = node.create_subscription(
            Path, f"/{robot}/path", self.on_path, estimate_qos
        )

    @staticmethod
    def _stamp_ns(header) -> int:
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    def _offer(self, stamp_ns: int, pos, quat, twist):
        if self.estimate is None or stamp_ns >= self.estimate[0]:
            self.estimate = (stamp_ns, pos, quat, twist)

    def on_odometry(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist
        self._offer(
            self._stamp_ns(msg.header),
            (p.x, p.y, p.z),
            (q.w, q.x, q.y, q.z),
            (t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z),
        )

    def on_path(self, msg: Path):
        if not msg.poses:
            return
        last = msg.poses[-1]
        p = last.pose.position
        q = last.pose.orientation
        stamp = self._stamp_ns(last.header) or self._stamp_ns(msg.header)
        self._offer(stamp, (p.x, p.y, p.z), (q.w, q.x, q.y, q.z), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))


class FastLivoLink(Node):
    """ROS 2 Node handling lockstep exchange with ARGoS."""

    def __init__(self, socket_path: str, lockstep_timeout: float = 5.0):
        super().__init__("argos_fast_livo_link")
        self.socket_path = socket_path
        self.lockstep_timeout = lockstep_timeout
        self.robots: dict[str, RobotIO] = {}
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.ticks_per_second = 100.0
        self.ticks = 0
        self.frames = 0
        self.scans = 0
        self.poses_returned = 0

    def serve(self):
        if os.path.exists(self.socket_path):
            os.unlink(self.socket_path)
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(self.socket_path)
        os.chmod(self.socket_path, 0o777)
        server.listen(1)
        self.get_logger().info(f"Fast-LIVO2 Link listening for ARGoS on {self.socket_path}")
        conn, _ = server.accept()
        self.get_logger().info("ARGoS connected to Fast-LIVO2 Link; running in lockstep")
        try:
            while rclpy.ok():
                if not self.handle_tick(conn):
                    break
        finally:
            conn.close()
            server.close()
            if os.path.exists(self.socket_path):
                os.unlink(self.socket_path)
        self.get_logger().info(
            f"ARGoS disconnected after {self.ticks} ticks: {self.frames} camera frames, "
            f"{self.scans} lidar scans published, {self.poses_returned} poses returned"
        )

    def handle_tick(self, conn: socket.socket) -> bool:
        head = recv_exactly(conn, 4 + 4 + 4 + 1 + 4)
        if head is None:
            return False
        magic, tick, tps, lockstep, n_robots = struct.unpack("<4sIIBI", head)
        if magic != MAGIC:
            raise RuntimeError(f"Protocol mismatch: bad magic {magic!r}")
        self.ticks_per_second = float(tps)

        stamp_ns = int(round(tick / float(tps) * 1e9))
        sec, nanosec = divmod(stamp_ns, 1_000_000_000)
        clock = Clock()
        clock.clock.sec = sec
        clock.clock.nanosec = nanosec
        self.clock_pub.publish(clock)

        names = [self.handle_robot(conn, sec, nanosec) for _ in range(n_robots)]
        self.ticks += 1

        rclpy.spin_once(self, timeout_sec=0.0)
        if lockstep:
            self.wait_for_poses(names, stamp_ns)
        conn.sendall(self.build_reply(names))
        return True

    def wait_for_poses(self, names: list[str], stamp_ns: int):
        deadline = time.monotonic() + self.lockstep_timeout
        while time.monotonic() < deadline:
            if all(self.estimate_is_current(name, stamp_ns) for name in names):
                return
            rclpy.spin_once(self, timeout_sec=0.005)
        self.get_logger().warning(
            f"Lockstep: pose timeout after {self.lockstep_timeout:.1f}s; releasing frame"
        )

    def estimate_is_current(self, name: str, stamp_ns: int) -> bool:
        est = self.robots[name].estimate
        return est is not None and est[0] >= stamp_ns

    def build_reply(self, names: list[str]) -> bytes:
        out = bytearray(ACK)
        out += struct.pack("<I", len(names))
        for name in names:
            pubs = self.robots[name]
            est = pubs.estimate
            raw = name.encode()
            out += struct.pack("<B", len(raw)) + raw
            if est is None:
                out += struct.pack("<I", 0)
                out += struct.pack("<7d", 0, 0, 0, 1, 0, 0, 0)
                out += struct.pack("<6d", 0, 0, 0, 0, 0, 0)
                out += struct.pack("<B", 0)
                continue
            est_ns, pos, quat, twist = est
            est_tick = int(round(est_ns * 1e-9 * self.ticks_per_second))
            out += struct.pack("<I", est_tick)
            out += struct.pack("<7d", pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3])
            out += struct.pack("<6d", *twist)
            out += struct.pack("<B", 1)
            self.poses_returned += 1
        return bytes(out)

    def handle_robot(self, conn: socket.socket, sec: int, nanosec: int) -> str:
        (id_len,) = struct.unpack("<B", recv_exactly(conn, 1))
        robot = recv_exactly(conn, id_len).decode()
        if robot not in self.robots:
            self.robots[robot] = RobotIO(self, robot)
            self.get_logger().info(f"Initialized Fast-LIVO2 IO for {robot}")
        pubs = self.robots[robot]

        (has_frame,) = struct.unpack("<B", recv_exactly(conn, 1))
        if has_frame:
            w, h, fov = struct.unpack("<IIf", recv_exactly(conn, 12))
            rgb = recv_exactly(conn, w * h * 3)
            (has_depth,) = struct.unpack("<B", recv_exactly(conn, 1))
            if has_depth:
                _ = recv_exactly(conn, w * h * 4)
            self.publish_frame(pubs, w, h, rgb, fov, sec, nanosec)
            self.frames += 1

        (has_scan,) = struct.unpack("<B", recv_exactly(conn, 1))
        if has_scan:
            (n_points,) = struct.unpack("<I", recv_exactly(conn, 4))
            blob = recv_exactly(conn, n_points * POINT_STEP)
            self.publish_scan(pubs, n_points, blob, sec, nanosec)
            self.scans += 1

        (has_wheels,) = struct.unpack("<B", recv_exactly(conn, 1))
        if has_wheels:
            wheel = struct.unpack("<7d", recv_exactly(conn, 56))
            self.publish_wheel(pubs, wheel, sec, nanosec)

        imu = struct.unpack("<6d", recv_exactly(conn, 48))
        self.publish_imu(pubs, imu, sec, nanosec)
        return robot

    def publish_frame(self, pubs: RobotIO, w: int, h: int, rgb: bytes, fov: float, sec: int, nanosec: int):
        # 1. Publish CompressedImage (bgr8; png compressed)
        img_c = CompressedImage()
        img_c.header.stamp.sec = sec
        img_c.header.stamp.nanosec = nanosec
        img_c.header.frame_id = pubs.frame_id
        img_c.format = "bgr8; png compressed bgr8"
        img_c.data = png_encode_rgb(w, h, rgb, bgr=True)
        pubs.color_compressed.publish(img_c)

        # 2. Publish raw Image
        img_raw = Image()
        img_raw.header = img_c.header
        img_raw.width = w
        img_raw.height = h
        img_raw.encoding = "bgr8"
        img_raw.is_bigendian = 0
        img_raw.step = w * 3
        # Swap RGB -> BGR
        bgr_buf = bytearray(rgb)
        bgr_buf[0::3], bgr_buf[2::3] = bgr_buf[2::3], bgr_buf[0::3]
        img_raw.data = bytes(bgr_buf)
        pubs.color_raw.publish(img_raw)

        # 3. Publish CameraInfo
        fy = h / (2.0 * math.tan(math.radians(fov) / 2.0))
        fx = fy
        info = CameraInfo()
        info.header = img_c.header
        info.width, info.height = w, h
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, w / 2.0, 0.0, fy, h / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, w / 2.0, 0.0, 0.0, fy, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        pubs.info.publish(info)

    def publish_scan(self, pubs: RobotIO, n_points: int, blob: bytes, sec: int, nanosec: int):
        cloud = PointCloud2()
        cloud.header.stamp.sec = sec
        cloud.header.stamp.nanosec = nanosec
        cloud.header.frame_id = pubs.frame_id
        cloud.height = 1
        cloud.width = n_points
        cloud.fields = POINT_FIELDS
        cloud.is_bigendian = False
        cloud.point_step = POINT_STEP
        cloud.row_step = POINT_STEP * n_points
        cloud.is_dense = True
        cloud.data = blob
        pubs.points.publish(cloud)

    def publish_wheel(self, pubs: RobotIO, o: tuple[float, ...], sec: int, nanosec: int):
        msg = Odometry()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec
        msg.header.frame_id = "odom"
        msg.child_frame_id = pubs.frame_id
        msg.pose.pose.position.x = o[0]
        msg.pose.pose.position.y = o[1]
        msg.pose.pose.position.z = o[2]
        msg.pose.pose.orientation.w = o[3]
        msg.pose.pose.orientation.x = o[4]
        msg.pose.pose.orientation.y = o[5]
        msg.pose.pose.orientation.z = o[6]
        pubs.wheel.publish(msg)

    def publish_imu(self, pubs: RobotIO, v: tuple[float, ...], sec: int, nanosec: int):
        msg = Imu()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec
        msg.header.frame_id = pubs.frame_id
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = v[0], v[1], v[2]
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = v[3], v[4], v[5]
        pubs.imu.publish(msg)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--socket", default="/tmp/argos_uf.sock")
    ap.add_argument("--lockstep-timeout", type=float, default=5.0)
    args = ap.parse_args()

    rclpy.init()
    node = FastLivoLink(args.socket, args.lockstep_timeout)
    try:
        node.serve()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
