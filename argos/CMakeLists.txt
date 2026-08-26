#
# SwarmDeck's ARGoS plugins: the robot controller and the bridge loop function.
#
# This is a standalone CMake project rather than part of the swarmdeck_ros
# colcon workspace, because the two never share a container: the `argos`
# service has ARGoS and no ROS, and the `sim` service has ROS and no ARGoS.
# The only thing crossing between them is the Unix socket the loop function
# below writes to.
#
#   cmake -B build -DCMAKE_BUILD_TYPE=Release .
#   cmake --build build -j
#
# The resulting libraries are loaded through ARGOS_PLUGIN_PATH; nothing is
# installed into the ARGoS prefix.
#
cmake_minimum_required(VERSION 3.8.0)
project(swarmdeck_argos)

if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_MACOSX_RPATH 0)
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH} ${CMAKE_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ARGoS REQUIRED)
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})

# ONE module, holding both the controller and the loop function.
#
# They were two, as argos3-examples keeps them, and that does not work here:
# the loop function resolves each robot's controller with
# `dynamic_cast<CSwarmdeckRobotController*>`, which needs that class's
# typeinfo symbol, and a separately dlopen'd module does not export it to its
# sibling. The failure is a load-time `undefined symbol:
# _ZTI25CSwarmdeckRobotController`, and the alternative fixes are all worse:
# compiling the controller into both modules registers the controller twice,
# and dropping the cast means the bridge cannot reach the sensors at all.
#
# Both <controllers library="..."> and <loop_functions library="..."> in the
# generated experiment therefore name this one library.
add_library(swarmdeck_argos MODULE
  controllers/swarmdeck_robot_controller.h
  controllers/swarmdeck_robot_controller.cpp
  loop_functions/swarmdeck_bridge_loop_functions.h
  loop_functions/swarmdeck_bridge_loop_functions.cpp)

target_include_directories(swarmdeck_argos PRIVATE
  ${CMAKE_SOURCE_DIR}/controllers)

target_link_libraries(swarmdeck_argos
  argos3core_${ARGOS_BUILD_FOR}
  argos3plugin_${ARGOS_BUILD_FOR}_genericrobot
  argos3plugin_${ARGOS_BUILD_FOR}_photorealism)
