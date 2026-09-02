#!/usr/bin/env python3
"""
Bridges a live ARGoS simulation to Ultra-Fusion, in lockstep.

ARGoS connects to the Unix socket below, sends one frame per simulation
tick, and blocks until this node replies. Nothing is dropped, because
the simulation cannot run ahead of the estimator, and nothing is written
to disk.

ARGoS itself never links ROS (see the <external_estimator> medium in
argos3/src/plugins/simulator/external_estimator/); this process is the
only place that does.

    ARGoS ── socket ──► uf_link ──► /{robot}/{imu,points,odom,color,depth}
                            ▲                    │
                            │                 uf_node  (one per robot)
                            └──── /{robot}/odom_lidar ─┘

The pose that comes back is reported by <odometry implementation=
"external"> inside ARGoS, so any controller, and the existing
Swarm-SLAM bridge, sees Ultra-Fusion's estimate in place of the
synthetic drift model.

Time
----
/clock is published from the simulation tick and every uf_node runs with
use_sim_time:=true, so the estimator's notion of time is the
simulation's. Verified: uf_node subscribes /clock when that parameter is
set.

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

The point layout is byte-identical to what Ultra-Fusion's own converter
produces (scripts/convert_m3dgr_ros1_to_ros2_common.py, point_step 26),
so the cloud is forwarded as an opaque blob with no repacking.

Usage:
  uf_link.py --socket /tmp/argos_uf.sock [--lockstep-timeout 5.0]
"""
import argparse
import math
import os
import socket
import struct
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Clock

MAGIC = b"AEBR"
ACK = b"ACK\0"

# One lidar point on the wire, and in the PointCloud2 that carries it.
# Ultra-Fusion's Livox PointCloud2 path (preprocess.lidar_type: 7) reads
# exactly these fields, and reads "timestamp" as nanoseconds.
POINT_STEP = 26
POINT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="timestamp", offset=16, datatype=PointField.FLOAT64, count=1),
    PointField(name="tag", offset=24, datatype=PointField.UINT8, count=1),
    PointField(name="line", offset=25, datatype=PointField.UINT8, count=1),
]


def recv_exactly(sock, n):
    """Reads exactly n bytes, or returns None if the peer hung up."""
    if n == 0:
        return b""
    chunks = []
    got = 0
    while got < n:
        b = sock.recv(min(1 << 20, n - got))
        if not b:
            return None
        chunks.append(b)
        got += len(b)
    return b"".join(chunks)


class RobotIO:
    """One robot's publishers, plus the newest pose uf_node reported.

    Topic names match what run_uf.sh remaps uf_node's absolute names to.
    Ultra-Fusion declares its topics absolute, so a namespace argument
    would not move them; explicit "-r /a:=/b" rules do, and are verified
    to work.
    """

    def __init__(self, node, robot):
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.robot = robot
        self.frame_id = "%s/base_link" % robot
        self.color = node.create_publisher(
            CompressedImage, "/%s/color/image_raw/compressed" % robot, qos)
        self.info = node.create_publisher(
            CameraInfo, "/%s/color/camera_info" % robot, qos)
        self.depth = node.create_publisher(
            Image, "/%s/aligned_depth_to_color/image_raw" % robot, qos)
        self.points = node.create_publisher(
            PointCloud2, "/%s/points" % robot, qos)
        self.imu = node.create_publisher(Imu, "/%s/imu" % robot, qos)
        self.wheel = node.create_publisher(Odometry, "/%s/odom" % robot, qos)
        # The fused estimate coming back out of uf_node.
        #
        # BEST_EFFORT on purpose. A RELIABLE subscription only matches a
        # RELIABLE publisher, so if uf_node ever advertises with a
        # sensor-data profile the two would silently never connect and
        # this would look exactly like an estimator that never
        # converged. BEST_EFFORT matches either. Losing the odd pose
        # costs nothing here: only the newest one is ever used.
        estimate_qos = QoSProfile(depth=10,
                                  reliability=ReliabilityPolicy.BEST_EFFORT,
                                  history=HistoryPolicy.KEEP_LAST)
        # Where the pose comes back depends on the FUSION MODE, and
        # nothing documents it:
        #
        #   lio/lwio/lvwio  -> /odom_lidar          (nav_msgs/Odometry)
        #   vio/viwo        -> NO Odometry at all; the trajectory only
        #                      appears as /result_path (nav_msgs/Path)
        #
        # So both are taken and the fresher one wins, which makes the
        # link mode-agnostic. Subscribing only to /odom_lidar in a
        # visual run looks exactly like an estimator that never
        # converged, even while it is happily writing poses to disk.
        self.estimate = None          # (stamp_ns, pos, quat, twist)
        self.sub_odom = node.create_subscription(
            Odometry, "/%s/odom_lidar" % robot, self.on_odometry, estimate_qos)
        self.sub_path = node.create_subscription(
            Path, "/%s/result_path" % robot, self.on_path, estimate_qos)

    @staticmethod
    def _stamp_ns(header):
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    def _offer(self, stamp_ns, pos, quat, twist):
        """Keeps the newest estimate, whichever topic it arrived on."""
        if self.estimate is None or stamp_ns >= self.estimate[0]:
            self.estimate = (stamp_ns, pos, quat, twist)

    def on_odometry(self, msg):
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        self._offer(self._stamp_ns(msg.header),
                    (p.x, p.y, p.z), (q.w, q.x, q.y, q.z),
                    (t.linear.x, t.linear.y, t.linear.z,
                     t.angular.x, t.angular.y, t.angular.z))

    def on_path(self, msg):
        if not msg.poses:
            return
        # The last entry is the current pose. A Path carries no twist,
        # so the velocities are reported as zero rather than invented.
        last = msg.poses[-1]
        p, q = last.pose.position, last.pose.orientation
        stamp = self._stamp_ns(last.header) or self._stamp_ns(msg.header)
        self._offer(stamp, (p.x, p.y, p.z), (q.w, q.x, q.y, q.z),
                    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))


class Link(Node):

    def __init__(self, socket_path, lockstep_timeout, depth_encoding="16UC1"):
        super().__init__("argos_ultra_fusion_link")
        self.socket_path = socket_path
        self.lockstep_timeout = lockstep_timeout
        self.depth_encoding = depth_encoding
        self.robots = {}
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        # Replaced by the value ARGoS sends in every frame header
        self.ticks_per_second = 100.0
        self.ticks = 0
        self.frames = 0
        self.scans = 0
        self.poses_returned = 0

    # ---------------------------------------------------------------- io

    def serve(self):
        if os.path.exists(self.socket_path):
            os.unlink(self.socket_path)
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(self.socket_path)
        os.chmod(self.socket_path, 0o777)
        server.listen(1)
        self.get_logger().info("waiting for ARGoS on %s" % self.socket_path)
        conn, _ = server.accept()
        self.get_logger().info("ARGoS connected; running in lockstep")
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
            "ARGoS disconnected after %d ticks: %d camera frames, %d lidar "
            "scans published, %d poses returned"
            % (self.ticks, self.frames, self.scans, self.poses_returned))

    def handle_tick(self, conn):
        head = recv_exactly(conn, 4 + 4 + 4 + 1 + 4)
        if head is None:
            return False
        magic, tick, tps, lockstep, n_robots = struct.unpack("<4sIIBI", head)
        if magic != MAGIC:
            raise RuntimeError("bad magic %r: protocol mismatch" % (magic,))
        self.ticks_per_second = float(tps)

        stamp_ns = int(round(tick / float(tps) * 1e9))
        sec, nanosec = divmod(stamp_ns, 1_000_000_000)
        clock = Clock()
        clock.clock.sec = sec
        clock.clock.nanosec = nanosec
        self.clock_pub.publish(clock)

        names = [self.handle_robot(conn, sec, nanosec) for _ in range(n_robots)]
        self.ticks += 1

        # Let the publishers flush and any estimate callbacks run before
        # the reply is assembled
        rclpy.spin_once(self, timeout_sec=0.0)
        if lockstep:
            self.wait_for_poses(names, stamp_ns)
        conn.sendall(self.build_reply(names))
        return True

    def wait_for_poses(self, names, stamp_ns):
        """Blocks until every robot has an estimate at least as new as
        this tick, or the timeout expires.

        This is where the "lockstep_pose" mode actually waits. Keeping it
        here rather than in ARGoS means the waiting happens in the
        process that already owns the estimator's event loop.
        """
        deadline = time.monotonic() + self.lockstep_timeout
        while time.monotonic() < deadline:
            if all(self.estimate_is_current(name, stamp_ns) for name in names):
                return
            rclpy.spin_once(self, timeout_sec=0.005)
        self.get_logger().warning(
            "lockstep: no pose for every robot within %.1f s; releasing the "
            "simulation with what is available" % self.lockstep_timeout)

    def estimate_is_current(self, name, stamp_ns):
        est = self.robots[name].estimate
        return est is not None and est[0] >= stamp_ns

    def build_reply(self, names):
        out = bytearray(ACK)
        out += struct.pack("<I", len(names))
        for name in names:
            pubs = self.robots[name]
            est = pubs.estimate
            raw = name.encode()
            out += struct.pack("<B", len(raw)) + raw
            if est is None:
                # The estimator has not converged yet. ARGoS keeps the
                # sensor reading invalid, which is what a controller on a
                # real robot sees while SLAM initializes.
                out += struct.pack("<I", 0)
                out += struct.pack("<7d", 0, 0, 0, 1, 0, 0, 0)
                out += struct.pack("<6d", 0, 0, 0, 0, 0, 0)
                out += struct.pack("<B", 0)
                continue
            est_ns, pos, quat, twist = est
            # Back to a tick: the reverse of the stamp derivation above
            est_tick = int(round(est_ns * 1e-9 * self.ticks_per_second))
            out += struct.pack("<I", est_tick)
            out += struct.pack("<7d", pos[0], pos[1], pos[2],
                               quat[0], quat[1], quat[2], quat[3])
            out += struct.pack("<6d", *twist)
            out += struct.pack("<B", 1)
            self.poses_returned += 1
        return bytes(out)

    def handle_robot(self, conn, sec, nanosec):
        (id_len,) = struct.unpack("<B", recv_exactly(conn, 1))
        robot = recv_exactly(conn, id_len).decode()
        if robot not in self.robots:
            self.robots[robot] = RobotIO(self, robot)
            self.get_logger().info("publishing topics for %s" % robot)
        pubs = self.robots[robot]

        (has_frame,) = struct.unpack("<B", recv_exactly(conn, 1))
        if has_frame:
            w, h, fov = struct.unpack("<IIf", recv_exactly(conn, 12))
            rgb = recv_exactly(conn, w * h * 3)
            (has_depth,) = struct.unpack("<B", recv_exactly(conn, 1))
            depth = recv_exactly(conn, w * h * 4) if has_depth else None
            self.publish_frame(pubs, w, h, rgb, depth, fov, sec, nanosec)
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

    # --------------------------------------------------------- messages

    def publish_frame(self, pubs, w, h, rgb, depth, fov, sec, nanosec):
        # Ultra-Fusion's visual path reads a CompressedImage
        # (image0_topic ends in /compressed). PNG keeps the frame
        # lossless, which matters when the depth channel beside it is
        # exact.
        img = CompressedImage()
        img.header.stamp.sec = sec
        img.header.stamp.nanosec = nanosec
        img.header.frame_id = pubs.frame_id
        # The format string is not decoration: cv_bridge parses it to
        # decide the destination encoding, and a bare "png" is not the
        # ROS convention. image_transport emits
        # "<encoding>; <codec> compressed <encoding>", and a consumer
        # that cannot parse it simply produces no image -- which shows
        # up downstream as an estimator reporting zero features rather
        # than as a decode error.
        #
        # OpenCV is BGR, so the bytes are written in that order to match
        # what the string promises.
        img.format = "bgr8; png compressed bgr8"
        img.data = png_encode_rgb(w, h, rgb, bgr=True)
        pubs.color.publish(img)

        # Pinhole intrinsics from the camera's VERTICAL field of view.
        # ARGoS renders with filament::Camera::Fov::VERTICAL, so the
        # HEIGHT is what the tangent relates to; dividing the width by it
        # inflates both focal lengths by the aspect ratio.
        fy = h / (2.0 * math.tan(math.radians(fov) / 2.0))
        fx = fy
        info = CameraInfo()
        info.header = img.header
        info.width, info.height = w, h
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, w / 2.0, 0.0, fy, h / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, w / 2.0, 0.0, 0.0, fy, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        pubs.info.publish(info)

        if depth is not None:
            d = Image()
            d.header = img.header
            d.width, d.height = w, h
            d.is_bigendian = 0
            if self.depth_encoding == "16UC1":
                # The ROS convention for aligned_depth_to_color/image_raw
                # is 16UC1 in MILLIMETRES, which is what a RealSense
                # publishes and therefore what the released profiles were
                # tuned against. Sending 32FC1 metres instead is not
                # rejected: the consumer reads the buffer with its own
                # assumption and every depth comes out ~1000x wrong, so
                # each feature fails its depth validity check and the
                # estimator reports "not enough features or parallax
                # (depth)" -- a message about geometry, for a units bug.
                metres = np.frombuffer(depth, dtype=np.float32)
                mm = np.clip(metres * 1000.0, 0, 65535).astype(np.uint16)
                d.encoding = "16UC1"
                d.step = w * 2
                d.data = mm.tobytes()
            else:
                d.encoding = "32FC1"
                d.step = w * 4
                d.data = depth
            pubs.depth.publish(d)

    def publish_scan(self, pubs, n_points, blob, sec, nanosec):
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

    def publish_wheel(self, pubs, o, sec, nanosec):
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

    def publish_imu(self, pubs, v, sec, nanosec):
        msg = Imu()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec
        msg.header.frame_id = pubs.frame_id
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = v[0], v[1], v[2]
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = v[3], v[4], v[5]
        pubs.imu.publish(msg)


def png_encode_rgb(w, h, rgb, bgr=False):
    """Minimal, dependency-free lossless PNG encoder.

    The runtime image has no cv_bridge Python bindings guaranteed, and
    pulling Pillow in just to serialize a frame is not worth a layer.
    Each row is written with filter type 0.

    With bgr=True the channel order is swapped on the way out, so the
    file matches the "bgr8" the format string declares.
    """
    import zlib

    if bgr:
        buf = bytearray(rgb)
        buf[0::3], buf[2::3] = buf[2::3], buf[0::3]
        rgb = bytes(buf)

    raw = bytearray()
    stride = w * 3
    for y in range(h):
        raw.append(0)
        raw += rgb[y * stride:(y + 1) * stride]

    def chunk(tag, payload):
        out = struct.pack(">I", len(payload)) + tag + payload
        return out + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)

    header = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", header)
            + chunk(b"IDAT", zlib.compress(bytes(raw), 1))
            + chunk(b"IEND", b""))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--socket", default="/tmp/argos_uf.sock")
    ap.add_argument("--depth-encoding", choices=("16UC1", "32FC1"),
                    default="16UC1",
                    help="16UC1 millimetres is the ROS convention for "
                         "aligned_depth_to_color/image_raw and what the "
                         "released profiles expect; 32FC1 metres is what the "
                         "Swarm-SLAM bridge uses")
    ap.add_argument("--lockstep-timeout", type=float, default=5.0,
                    help="seconds to wait for a current pose when ARGoS asks "
                         "for lockstep_pose before releasing it anyway")
    args = ap.parse_args()

    rclpy.init()
    node = Link(args.socket, args.lockstep_timeout, args.depth_encoding)
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
