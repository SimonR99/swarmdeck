#!/usr/bin/env python3
"""Publish Asimov's Unitree native topics as standard ROS odometry and state.

Asimov's installed G1 bridge has a correct control/state implementation, but
its generic odometry child subscribes to ``rt/sportmodestate`` and state child
to ``rt/lowstate``. On this G1 hardware:
- Live SportModeState is published on ``rt/odommodestate``
- Live LowState is published on ``rt/lf/lowstate``

Keep this bridge small and owned by SwarmDeck so deployment does not patch or
rebuild the external workspace.

This process only subscribes to native state and publishes ROS messages; it
does not call a locomotion API.
"""

# The SDK must be imported before rclpy; see g1_ros2_bridge.dds_init.
import time
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_, WirelessController_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import BmsState_, LowState_

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import Int32, String
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster

from g1_ros2_bridge.dds_init import finalize_dds, prepare_dds
from g1_ros2_bridge.joint_names import JOINT_INDICES, JOINT_NAMES


class AsimovOdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("g1_odom_bridge")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("pelvis_frame_id", "pelvis")
        self.declare_parameter("publish_odom_tf", True)
        self.declare_parameter("publish_imu_tf", True)
        self.declare_parameter("native_odom_topic", "rt/odommodestate")
        self.declare_parameter("native_state_topic", "rt/lf/lowstate")
        self.declare_parameter("native_bms_topic", "rt/lf/bmsstate")

        self.odom_frame = str(self.get_parameter("odom_frame_id").value)
        self.base_frame = str(self.get_parameter("base_frame_id").value)
        self.imu_frame = str(self.get_parameter("imu_frame_id").value)
        self.pelvis_frame = str(self.get_parameter("pelvis_frame_id").value)
        self.native_odom_topic = str(self.get_parameter("native_odom_topic").value)
        self.native_state_topic = str(self.get_parameter("native_state_topic").value)
        self.native_bms_topic = str(self.get_parameter("native_bms_topic").value)
        self.publish_odom_tf = bool(self.get_parameter("publish_odom_tf").value)
        self.publish_imu_tf = bool(self.get_parameter("publish_imu_tf").value)

        qos = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", qos)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", qos)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", qos)
        self.battery_pub = self.create_publisher(BatteryState, "/battery_state", qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._js = JointState()
        self._js.name = JOINT_NAMES

        self.loco = None
        self.wireless_pub = None
        self.odom_subscriber = None
        self.state_subscriber = None
        self.bms_subscriber = None

        self.create_subscription(String, "/cmd_body", self.on_body_command, qos)
        self.create_subscription(Int32, "/g1_loco_bridge/set_fsm_id", self.on_fsm_id, qos)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, qos)

    def send_keys(self, keys: int, duration: float = 0.4) -> None:
        if self.wireless_pub is None:
            return
        msg = WirelessController_(0.0, 0.0, 0.0, 0.0, keys)
        t0 = time.time()
        while time.time() - t0 < duration:
            self.wireless_pub.Write(msg)
            time.sleep(0.03)
        self.wireless_pub.Write(WirelessController_(0.0, 0.0, 0.0, 0.0, 0))

    def on_cmd_vel(self, msg: Twist) -> None:
        vx = max(-1.0, min(1.0, float(msg.linear.x)))
        vy = max(-0.6, min(0.6, float(msg.linear.y)))
        wz = max(-1.5, min(1.5, float(msg.angular.z)))
        self.get_logger().info(f"[asimov_bridge] cmd_vel: vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}")
        try:
            if self.wireless_pub is not None:
                c_msg = WirelessController_(float(vy), float(vx), -float(wz), 0.0, 0)
                self.wireless_pub.Write(c_msg)
            if self.loco is not None:
                if abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(wz) < 1e-3:
                    self.loco.StopMove()
                else:
                    self.loco.SetVelocity(vx, vy, wz, 1.0)
        except Exception as e:
            self.get_logger().error(f"[asimov_bridge] Move failed: {e}")

    def on_fsm_id(self, msg: Int32) -> None:
        fsm_id = int(msg.data)
        self.get_logger().info(f"[asimov_bridge] set_fsm_id: {fsm_id}")
        if self.loco is None:
            return
        try:
            self.loco.SetFsmId(fsm_id)
        except Exception as e:
            self.get_logger().error(f"[asimov_bridge] SetFsmId({fsm_id}) failed: {e}")

    def on_body_command(self, msg: String) -> None:
        action = msg.data.strip().lower()
        self.get_logger().info(f"[asimov_bridge] received body_command: {action}")
        if self.loco is None:
            self.get_logger().warn("[asimov_bridge] LocoClient not initialized")
            return
        try:
            if action in ("damping", "damp", "release"):
                self.loco.Damp()
            elif action in ("lock_stand", "squat_to_stand", "ready_mode"):
                # L2 + UP = 4128 -> FSM 4 (Locked Standing)
                self.send_keys((1 << 5) | (1 << 12))
            elif action in ("stand", "start", "walk_mode", "claim"):
                # R1 + Y = 2049 -> FSM 501 (Walk Mode)
                self.send_keys((1 << 0) | (1 << 11))
                self.loco.Start()
            elif action == "sit":
                self.loco.Sit()
            elif action == "lie_to_stand":
                self.loco.Lie2StandUp()
            elif action == "zero_torque":
                self.loco.ZeroTorque()
            elif action == "high_stand":
                self.loco.HighStand()
            elif action == "low_stand":
                self.loco.LowStand()
            elif action == "stop":
                self.loco.StopMove()
            else:
                self.get_logger().warn(f"[asimov_bridge] unknown body action: {action}")
        except Exception as e:
            self.get_logger().error(f"[asimov_bridge] error executing {action}: {e}")

    def on_sport_state(self, msg: SportModeState_) -> None:
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(msg.position[0])
        odom.pose.pose.position.y = float(msg.position[1])
        odom.pose.pose.position.z = float(msg.position[2])

        # Unitree's quaternion order is [w, x, y, z].
        q = msg.imu_state.quaternion
        odom.pose.pose.orientation.w = float(q[0])
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])
        odom.twist.twist.angular.z = float(msg.yaw_speed)
        self.odom_pub.publish(odom)

        if self.publish_odom_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = odom.pose.pose.position.x
            transform.transform.translation.y = odom.pose.pose.position.y
            transform.transform.translation.z = odom.pose.pose.position.z
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)

    def on_low_state(self, msg: LowState_) -> None:
        stamp = self.get_clock().now().to_msg()

        # IMU
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        q = msg.imu_state.quaternion  # [w, x, y, z]
        imu.orientation.w = float(q[0])
        imu.orientation.x = float(q[1])
        imu.orientation.y = float(q[2])
        imu.orientation.z = float(q[3])
        imu.angular_velocity.x = float(msg.imu_state.gyroscope[0])
        imu.angular_velocity.y = float(msg.imu_state.gyroscope[1])
        imu.angular_velocity.z = float(msg.imu_state.gyroscope[2])
        imu.linear_acceleration.x = float(msg.imu_state.accelerometer[0])
        imu.linear_acceleration.y = float(msg.imu_state.accelerometer[1])
        imu.linear_acceleration.z = float(msg.imu_state.accelerometer[2])
        self.imu_pub.publish(imu)

        if self.publish_imu_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.pelvis_frame
            t.child_frame_id = self.imu_frame
            t.transform.rotation = imu.orientation
            self.tf_broadcaster.sendTransform(t)

        # Joint states
        positions, velocities, efforts = [], [], []
        for idx in JOINT_INDICES:
            if idx < len(msg.motor_state):
                m = msg.motor_state[idx]
                positions.append(float(m.q))
                velocities.append(float(m.dq))
                efforts.append(float(m.tau_est))
            else:
                positions.append(0.0)
                velocities.append(0.0)
                efforts.append(0.0)
        self._js.header.stamp = stamp
        self._js.position = positions
        self._js.velocity = velocities
        self._js.effort = efforts
        self.joint_pub.publish(self._js)

    def on_bms_state(self, msg: BmsState_) -> None:
        stamp = self.get_clock().now().to_msg()
        b_msg = BatteryState()
        b_msg.header.stamp = stamp
        b_msg.header.frame_id = self.base_frame
        b_msg.percentage = float(msg.soc) / 100.0
        if len(msg.bmsvoltage) > 0 and msg.bmsvoltage[0] > 0:
            b_msg.voltage = float(msg.bmsvoltage[0]) / 1000.0
        if msg.current != 0:
            b_msg.current = float(msg.current) / 1000.0
        b_msg.present = True
        self.battery_pub.publish(b_msg)


def main() -> None:
    domain, interface = prepare_dds()
    rclpy.init()
    node = AsimovOdomBridge()
    finalize_dds(domain, interface)
    try:
        node.loco = LocoClient()
        node.loco.SetTimeout(1.5)
        node.loco.Init()
    except Exception as exc:
        node.get_logger().warn(f"LocoClient initialization failed: {exc}")
    try:
        node.wireless_pub = ChannelPublisher("rt/wirelesscontroller", WirelessController_)
        node.wireless_pub.Init()
    except Exception as exc:
        node.get_logger().warn(f"Wireless publisher initialization failed: {exc}")
    node.odom_subscriber = ChannelSubscriber(node.native_odom_topic, SportModeState_)
    node.odom_subscriber.Init(node.on_sport_state)
    node.state_subscriber = ChannelSubscriber(node.native_state_topic, LowState_)
    node.state_subscriber.Init(node.on_low_state)
    node.bms_subscriber = ChannelSubscriber(node.native_bms_topic, BmsState_)
    node.bms_subscriber.Init(node.on_bms_state)
    node.get_logger().info(
        f"DDS up (domain={domain}, interface='{interface or '<default>'}'); "
        f"subscribed to {node.native_odom_topic} -> /odom, {node.native_state_topic} -> /joint_states, {node.native_bms_topic} -> /battery_state, /cmd_body -> locomotion"
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
