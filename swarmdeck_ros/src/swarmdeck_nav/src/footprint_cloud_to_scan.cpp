#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}
}  // namespace

class FootprintCloudToScan final : public rclcpp::Node
{
public:
  FootprintCloudToScan()
  : Node("footprint_cloud_to_scan")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/ouster/points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/nav_scan");
    output_frame_ = declare_parameter<std::string>("output_frame", "nav_lidar");

    min_height_ = declare_parameter<double>("min_height", -0.37);
    max_height_ = declare_parameter<double>("max_height", 1.28);
    range_min_ = declare_parameter<double>("range_min", 0.05);
    range_max_ = declare_parameter<double>("range_max", 10.0);
    angle_min_ = declare_parameter<double>("angle_min", -kPi);
    angle_increment_ = declare_parameter<double>("angle_increment", 2.0 * kPi / 512.0);
    scan_time_ = declare_parameter<double>("scan_time", 0.1);
    use_inf_ = declare_parameter<bool>("use_inf", true);
    inf_epsilon_ = declare_parameter<double>("inf_epsilon", 1.0);

    sensor_yaw_in_base_ = declare_parameter<double>("sensor_yaw_in_base", kPi);
    sensor_cos_yaw_ = std::cos(sensor_yaw_in_base_);
    sensor_sin_yaw_ = std::sin(sensor_yaw_in_base_);
    footprint_front_ = declare_parameter<double>("footprint_front", 0.361);
    footprint_rear_ = declare_parameter<double>("footprint_rear", -0.661);
    footprint_half_width_ = declare_parameter<double>("footprint_half_width", 0.389);
    footprint_padding_ = declare_parameter<double>("footprint_padding", 0.05);

    if (
      min_height_ >= max_height_ || range_min_ < 0.0 || range_min_ >= range_max_ ||
      angle_increment_ <= 0.0 || footprint_front_ <= 0.0 || footprint_rear_ >= 0.0 ||
      footprint_half_width_ <= 0.0 || footprint_padding_ < 0.0)
    {
      throw std::invalid_argument("invalid cloud projection or footprint parameters");
    }

    const double requested_span = 2.0 * kPi;
    beam_count_ = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(requested_span / angle_increment_)));
    angle_increment_ = requested_span / static_cast<double>(beam_count_);
    angle_max_ = angle_min_ + angle_increment_ * static_cast<double>(beam_count_ - 1);

    // The live Ouster publisher is RELIABLE/TRANSIENT_LOCAL. Matching its
    // reliability prevents whole 10 Hz clouds from being discarded during
    // brief scheduler contention. A reliable scan publisher is compatible
    // with Nav2's best-effort sensor subscribers and avoids another lossy hop.
    const auto cloud_qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable().durability_volatile();
    const auto scan_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, scan_qos);
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, cloud_qos,
      std::bind(&FootprintCloudToScan::on_cloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Projecting %s over z %.2f..%.2f m into %zu beams; rejecting padded footprint "
      "x %.3f..%.3f m, |y| <= %.3f m",
      input_topic_.c_str(), min_height_, max_height_, beam_count_,
      footprint_rear_ - footprint_padding_, footprint_front_ + footprint_padding_,
      footprint_half_width_ + footprint_padding_);
  }

private:
  void on_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header = cloud->header;
    scan.header.frame_id = output_frame_;
    scan.angle_min = static_cast<float>(angle_min_);
    scan.angle_max = static_cast<float>(angle_max_);
    scan.angle_increment = static_cast<float>(angle_increment_);
    scan.time_increment = 0.0F;
    scan.scan_time = static_cast<float>(scan_time_);
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    const float empty_range = use_inf_ ? std::numeric_limits<float>::infinity() :
      static_cast<float>(range_max_ + inf_epsilon_);
    scan.ranges.assign(beam_count_, empty_range);

    const double front = footprint_front_ + footprint_padding_;
    const double rear = footprint_rear_ - footprint_padding_;
    const double half_width = footprint_half_width_ + footprint_padding_;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        const double px = *x;
        const double py = *y;
        const double pz = *z;
        if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) {
          continue;
        }
        if (pz < min_height_ || pz > max_height_) {
          continue;
        }

        const double range_squared = px * px + py * py;
        if (
          range_squared < range_min_ * range_min_ ||
          range_squared > range_max_ * range_max_)
        {
          continue;
        }

        // Rotate Cartesian coordinates directly. Computing atan2 + sin + cos
        // for every point made the full-height projector needlessly expensive
        // on the Jetson; this transform uses four multiplies and two adds.
        const double base_x = sensor_cos_yaw_ * px - sensor_sin_yaw_ * py;
        const double base_y = sensor_sin_yaw_ * px + sensor_cos_yaw_ * py;
        if (rear <= base_x && base_x <= front && std::abs(base_y) <= half_width) {
          continue;
        }

        const double range = std::sqrt(range_squared);
        const double raw_angle = std::atan2(py, px);
        // The output scan keeps raw Ouster angular coordinates. Its explicit
        // output frame has the measured pi-yaw static edge to the physical
        // base frame, avoiding SuperOdometry's misleading os_lidar child name.
        const double wrapped = normalize_angle(raw_angle - angle_min_);
        const double positive = wrapped < 0.0 ? wrapped + 2.0 * kPi : wrapped;
        const std::size_t index = std::min<std::size_t>(
          beam_count_ - 1, static_cast<std::size_t>(positive / angle_increment_));
        scan.ranges[index] = std::min(scan.ranges[index], static_cast<float>(range));
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Cannot read PointCloud2 x/y/z fields: %s", error.what());
      return;
    }

    scan_publisher_->publish(std::move(scan));
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  double min_height_;
  double max_height_;
  double range_min_;
  double range_max_;
  double angle_min_;
  double angle_max_;
  double angle_increment_;
  double scan_time_;
  bool use_inf_;
  double inf_epsilon_;
  double sensor_yaw_in_base_;
  double sensor_cos_yaw_;
  double sensor_sin_yaw_;
  double footprint_front_;
  double footprint_rear_;
  double footprint_half_width_;
  double footprint_padding_;
  std::size_t beam_count_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FootprintCloudToScan>());
  rclcpp::shutdown();
  return 0;
}
