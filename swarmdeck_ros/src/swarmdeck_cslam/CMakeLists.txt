cmake_minimum_required(VERSION 3.8)
project(swarmdeck_cslam)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS nodes/graph_reporter.py nodes/cslam_grid.py
        DESTINATION lib/${PROJECT_NAME})
ament_package()
