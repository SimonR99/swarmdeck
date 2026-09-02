# FAST-LIVO2 for SwarmDeck, built from source for both arm64 and amd64.
#
# Source is vendored under vendor/ from sjtuyinjie/M3DGR's Fast_LIVO2_M3DGR
# baseline (which carries hku-mars FAST-LIVO2 plus rpg_vikit). Upstream is
# catkin/ROS 1; the ament port is include/ros1_shim.h plus the two build files.
#
# NOT vendored, and deliberately:
#   livox_ros_driver  - only the Avia path needs CustomMsg, and Botman is Ouster
#   vikit_ros         - one call site (camera_loader), replaced with direct
#                       PinholeCamera construction from parameters
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-perception

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git ca-certificates \
        libomp-dev libeigen3-dev libboost-thread-dev libboost-system-dev \
        ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-pcl-ros ros-${ROS_DISTRO}-pcl-conversions \
        ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-geometry-msgs ros-${ROS_DISTRO}-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

# Sophus, pinned to the NON-TEMPLATED double-only API.
#
# This is not a version preference, it is a hard requirement: vikit_common and
# FAST-LIVO2 both include <sophus/se3.h>, which only exists before the
# templated rewrite. Release 1.22.10 ships <sophus/se3.hpp> instead and fails
# with "fatal error: sophus/se3.h: No such file or directory". Commit a621ff is
# the one upstream's own build instructions pin, for exactly this reason.
#
# That commit predates C++17 std::complex, whose real()/imag() setters are no
# longer assignable, so so2.cpp needs the two-line fix below to compile at all
# on any modern toolchain.
ARG SOPHUS_COMMIT=a621ff
# Partial clone: a full-history clone of Sophus took over 15 minutes on the
# Jetson's link and was still in index-pack. blob:none fetches trees and
# commits only, filling blobs in on checkout, which is what the pinned commit
# actually needs.
RUN git clone --filter=blob:none --no-checkout https://github.com/strasdat/Sophus.git /tmp/Sophus \
    && cd /tmp/Sophus \
    && git checkout ${SOPHUS_COMMIT} \
    && sed -i 's/unit_complex_\.real() = 1\./unit_complex_.real(1.);/' sophus/so2.cpp \
    && sed -i 's/unit_complex_\.imag() = 0\./unit_complex_.imag(0.);/' sophus/so2.cpp \
    # -Wno-error is ARM-specific and load-bearing. Sophus a621ff compiles with
    # -Werror, and on arm64 Eigen's NEON path
    # (Eigen/src/Core/arch/NEON/PacketMath.h) memcpy's a non-trivial
    # Packet4c, which trips -Wclass-memaccess and so fails the build. The
    # equivalent x86 SSE path does not, which is why this only appears when
    # building on the Jetson.
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
         -DCMAKE_CXX_FLAGS="-Wno-error -Wno-class-memaccess" \
    && cmake --build build --target install -j"$(nproc)" \
    && ldconfig \
    && rm -rf /tmp/Sophus

WORKDIR /fast_livo_ws/src
COPY vendor/vikit_common /fast_livo_ws/src/vikit_common
COPY vendor/fast_livo    /fast_livo_ws/src/fast_livo

WORKDIR /fast_livo_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --merge-install \
         --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY config /fast_livo_ws/config
WORKDIR /fast_livo_ws
