#include "LIVMapper.h"

// ROS 2 entry point. Upstream's ROS 1 main created a NodeHandle and an
// ImageTransport and handed both to LIVMapper; here the rclcpp node is created
// first and published through the shim's global pointer, which is what
// ros::Time::now, the tf broadcaster and the parameter helpers bind to.
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("laserMapping");
  ros::g_node = node.get();

  ros::NodeHandle nh(node.get());
  image_transport::ImageTransport it(node);

  LIVMapper mapper(nh);
  mapper.initializeSubscribersAndPublishers(nh, it);
  mapper.run();

  rclcpp::shutdown();
  return 0;
}
