cmake_minimum_required(VERSION 3.8)
project(swarmdeck_sim)
find_package(ament_cmake REQUIRED)

# `nodes` carries the ARGoS bridge, which session.launch.py runs by absolute
# path out of the source tree; installing it keeps `ros2 run` working too.
install(DIRECTORY worlds scenario nodes DESTINATION share/${PROJECT_NAME})
install(PROGRAMS nodes/swarmdeck_argos_bridge.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
