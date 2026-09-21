cmake_minimum_required(VERSION 3.8)
project(swarmdeck_sim)
find_package(ament_cmake REQUIRED)

# `nodes` carries the ARGoS bridge; the generated scenario helpers are installed
# alongside it for the session launcher.
install(DIRECTORY scenario nodes DESTINATION share/${PROJECT_NAME})
install(PROGRAMS nodes/swarmdeck_argos_bridge.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
