// ROS 1 -> ROS 2 compatibility shim for the vendored FAST-LIVO2 sources.
//
// WHY A SHIM RATHER THAN A REWRITE
//   The algorithm here is ~7400 lines and only 73 of them touch ROS. Rewriting
//   the call sites would make every future sync with upstream a manual merge.
//   This header instead provides the small slice of the ROS 1 C++ API that
//   FAST-LIVO2 actually uses, implemented on rclcpp, so LIVMapper.cpp, vio.cpp,
//   voxel_map.cpp and the rest stay as upstream wrote them.
//
//   The whole surface, counted from the sources:
//     ros::NodeHandle (+ 64 nh.param calls)   ros::Publisher (28) / Subscriber (3)
//     ros::Time / Time::now (16)              ros::Rate, Duration, Timer, TimerEvent
//     ros::init / ok / spinOnce               tf:: quaternion + broadcaster helpers
//
// PARAMETER NAMES
//   ROS 1 nests parameters with '/', ROS 2 with '.', and ROS 2 rejects '/' in a
//   parameter name outright. param() translates, so upstream's "common/lid_topic"
//   reads ROS 2 parameter "common.lid_topic". The YAML must therefore be the
//   ROS 2 shape (/**: ros__parameters: common: lid_topic: ...). Getting that
//   wrong is what killed the previous attempt here: the node aborted with
//   "Cannot have a value before ros__parameters at line 2".

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <algorithm>
#include <cassert>
#include <fstream>   // ROS 1's ros/ros.h pulled this in transitively
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace ros {

inline rclcpp::Node *g_node = nullptr;

inline std::string to_ros2_param(std::string name) {
  std::replace(name.begin(), name.end(), '/', '.');
  return name;
}

// ---------------------------------------------------------------- Time -----
class Time {
public:
  Time() = default;
  explicit Time(double t) : sec_(t) {}
  Time(int32_t s, uint32_t ns) : sec_(static_cast<double>(s) + ns * 1e-9) {}
  static Time now() {
    return Time(g_node ? g_node->now().seconds()
                       : rclcpp::Clock(RCL_ROS_TIME).now().seconds());
  }
  double toSec() const { return sec_; }
  Time &fromSec(double s) { sec_ = s; return *this; }
  operator rclcpp::Time() const { return rclcpp::Time(static_cast<int64_t>(sec_ * 1e9)); }
  // Upstream assigns straight into header.stamp, which is a
  // builtin_interfaces::msg::Time in ROS 2.
  operator builtin_interfaces::msg::Time() const {
    builtin_interfaces::msg::Time m;
    m.sec = static_cast<int32_t>(sec_);
    m.nanosec = static_cast<uint32_t>((sec_ - m.sec) * 1e9);
    return m;
  }
private:
  double sec_ = 0.0;
};

class Duration {
public:
  Duration() = default;
  explicit Duration(double s) : sec_(s) {}
  void sleep() const {
    if (sec_ > 0) rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(sec_ * 1e9)));
  }
  double toSec() const { return sec_; }
  // voxel_map.cpp assigns straight into Marker::lifetime.
  operator builtin_interfaces::msg::Duration() const {
    builtin_interfaces::msg::Duration d;
    d.sec = static_cast<int32_t>(sec_);
    d.nanosec = static_cast<uint32_t>((sec_ - d.sec) * 1e9);
    return d;
  }
private:
  double sec_ = 0.0;
};

class Rate {
public:
  explicit Rate(double hz) : period_(hz > 0 ? 1.0 / hz : 0.0) {}
  void sleep() { Duration(period_).sleep(); }
private:
  double period_;
};

struct TimerEvent {};
using Timer = rclcpp::TimerBase::SharedPtr;

// ----------------------------------------------------------- Publisher -----
// ROS 1's ros::Publisher is type-erased and FAST-LIVO2 keeps 28 of them in one
// struct, so the shim erases the type too and recovers it at publish().
class Publisher {
public:
  Publisher() = default;
  template <typename T>
  Publisher(typename rclcpp::Publisher<T>::SharedPtr p)
      : pub_(std::static_pointer_cast<void>(p)) {}

  template <typename T> void publish(const T &msg) const {
    if (!pub_) return;
    std::static_pointer_cast<rclcpp::Publisher<T>>(pub_)->publish(msg);
  }
  size_t getNumSubscribers() const {
    return base_ ? base_->get_subscription_count() : 0;
  }
  explicit operator bool() const { return static_cast<bool>(pub_); }

  std::shared_ptr<void> pub_;
  rclcpp::PublisherBase::SharedPtr base_;
};

using Subscriber = rclcpp::SubscriptionBase::SharedPtr;

// ---------------------------------------------------------- NodeHandle -----
class NodeHandle {
public:
  NodeHandle() = default;
  explicit NodeHandle(rclcpp::Node *n) : node_(n) {}

  template <typename T>
  void param(const std::string &name, T &out, const T &def) const {
    const std::string key = to_ros2_param(name);
    if (!node_->has_parameter(key)) node_->declare_parameter<T>(key, def);
    node_->get_parameter(key, out);
  }

  template <typename T>
  Publisher advertise(const std::string &topic, int depth) const {
    auto p = node_->create_publisher<T>(topic, rclcpp::QoS(rclcpp::KeepLast(depth)));
    Publisher out;
    out.pub_ = std::static_pointer_cast<void>(p);
    out.base_ = p;
    return out;
  }

  template <typename M, typename T>
  Subscriber subscribe(const std::string &topic, int depth,
                       void (T::*fn)(const std::shared_ptr<const M> &), T *obj) const {
    return node_->create_subscription<M>(
        topic, rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort(),
        [obj, fn](const std::shared_ptr<const M> msg) { (obj->*fn)(msg); });
  }

  template <typename T>
  ::ros::Timer createTimer(Duration period, void (T::*fn)(const TimerEvent &),
                           T *obj) const {
    return node_->create_wall_timer(
        std::chrono::nanoseconds(static_cast<int64_t>(period.toSec() * 1e9)),
        [obj, fn]() { TimerEvent e; (obj->*fn)(e); });
  }

  rclcpp::Node *node_ = nullptr;
};

// ROS 1 message stamps carried .toSec(); builtin_interfaces::msg::Time does not.
inline double toSec(const builtin_interfaces::msg::Time &t) {
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

// ROS 2 has no vector<int> parameter type: its integer array is vector<int64_t>.
// Upstream declares several extrinsics as vector<int> and calls param with an
// explicit template argument, so this must be a specialisation, not an overload.
template <>
inline void NodeHandle::param<std::vector<int>>(const std::string &name,
                                                std::vector<int> &out,
                                                const std::vector<int> &def) const {
  const std::string key = to_ros2_param(name);
  std::vector<int64_t> wide(def.begin(), def.end());
  if (!node_->has_parameter(key)) node_->declare_parameter<std::vector<int64_t>>(key, wide);
  node_->get_parameter(key, wide);
  out.assign(wide.begin(), wide.end());
}

inline bool ok() { return rclcpp::ok(); }
inline void spinOnce() { if (g_node) rclcpp::spin_some(g_node->get_node_base_interface()); }

} // namespace ros

// ------------------------------------------------------------------ tf -----
namespace tf {
inline geometry_msgs::msg::Quaternion
createQuaternionMsgFromRollPitchYaw(double r, double p, double y) {
  tf2::Quaternion q;
  q.setRPY(r, p, y);
  geometry_msgs::msg::Quaternion m;
  m.x = q.x(); m.y = q.y(); m.z = q.z(); m.w = q.w();
  return m;
}

// Upstream broadcasts one map->body transform. tf2_ros needs a node, so the
// shim binds to the global node the way ROS 1's implicit NodeHandle did.
class Transform;
class Quaternion {
public:
  Quaternion() = default;
  Quaternion(double x, double y, double z, double w) : q_(x, y, z, w) {}
  void setW(double w) { q_.setW(w); } void setX(double x) { q_.setX(x); }
  void setY(double y) { q_.setY(y); } void setZ(double z) { q_.setZ(z); }
  tf2::Quaternion q_;
};
class Vector3 {
public:
  Vector3() = default;
  Vector3(double x, double y, double z) : x_(x), y_(y), z_(z) {}
  double x_ = 0, y_ = 0, z_ = 0;
};
class Transform {
public:
  void setOrigin(const Vector3 &v) { v_ = v; }
  void setRotation(const Quaternion &q) { q_ = q; }
  Vector3 v_; Quaternion q_;
};
class StampedTransform {
public:
  StampedTransform() = default;
  StampedTransform(const Transform &t, const ros::Time &stamp,
                   const std::string &parent, const std::string &child)
      : t_(t), stamp_(stamp), parent_(parent), child_(child) {}
  StampedTransform(const Transform &t, const builtin_interfaces::msg::Time &stamp,
                   const std::string &parent, const std::string &child)
      : t_(t), stamp_(ros::Time(ros::toSec(stamp))), parent_(parent), child_(child) {}
  Transform t_; ros::Time stamp_; std::string parent_, child_;
};
class TransformBroadcaster {
public:
  TransformBroadcaster() {
    if (ros::g_node) impl_ = std::make_shared<tf2_ros::TransformBroadcaster>(*ros::g_node);
  }
  void sendTransform(const StampedTransform &st) {
    if (!impl_) return;
    geometry_msgs::msg::TransformStamped m;
    m.header.stamp = st.stamp_;
    m.header.frame_id = st.parent_;
    m.child_frame_id = st.child_;
    m.transform.translation.x = st.t_.v_.x_;
    m.transform.translation.y = st.t_.v_.y_;
    m.transform.translation.z = st.t_.v_.z_;
    m.transform.rotation.x = st.t_.q_.q_.x();
    m.transform.rotation.y = st.t_.q_.q_.y();
    m.transform.rotation.z = st.t_.q_.q_.z();
    m.transform.rotation.w = st.t_.q_.q_.w();
    impl_->sendTransform(m);
  }
private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> impl_;
};
} // namespace tf

// ROS 1 wrote sensor_msgs::Imu where ROS 2 writes sensor_msgs::msg::Imu, and
// spelled the pointer typedefs ConstPtr / ImuConstPtr. Alias them back into the
// message namespaces so the vendored sources compile unchanged.
namespace sensor_msgs {
using Imu = msg::Imu;
using Image = msg::Image;
using CompressedImage = msg::CompressedImage;
using PointCloud2 = msg::PointCloud2;
using ImuConstPtr = msg::Imu::ConstSharedPtr;
using ImageConstPtr = msg::Image::ConstSharedPtr;
} // namespace sensor_msgs
namespace nav_msgs {
using Odometry = msg::Odometry;
using Path = msg::Path;
} // namespace nav_msgs
namespace geometry_msgs {
using Quaternion = msg::Quaternion;
using PoseStamped = msg::PoseStamped;
} // namespace geometry_msgs
namespace visualization_msgs {
using Marker = msg::Marker;
using MarkerArray = msg::MarkerArray;
} // namespace visualization_msgs

// Logging macros. Upstream uses only the four unformatted levels.
#define ROS_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("fast_livo"), __VA_ARGS__)
#define ROS_INFO(...)  RCLCPP_INFO(rclcpp::get_logger("fast_livo"), __VA_ARGS__)
#define ROS_WARN(...)  RCLCPP_WARN(rclcpp::get_logger("fast_livo"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("fast_livo"), __VA_ARGS__)
#define ROS_ASSERT(cond) assert(cond)
