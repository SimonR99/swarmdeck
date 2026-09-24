cmake_minimum_required(VERSION 3.8)
project(swarmdeck_peer)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# The `swarmdeck_peer` Python package carries the Swarm-SLAM bridge and the
# MOLA worker; the bridge is also installed as the launch file's executable.
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS swarmdeck_peer/cslam_bridge.py
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
