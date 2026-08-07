cmake_minimum_required(VERSION 3.8)
project(swarmdeck_nav)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/odom_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odom_to_tf
)
ament_package()
