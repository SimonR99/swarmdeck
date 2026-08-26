# Ultra-Fusion (ROS 2 Humble) as an ARGoS odometry sidecar.
#
# Ultra-Fusion ships as prebuilt binaries, so nothing is compiled here:
# the runtime image carries ROS/PCL/OpenCV/Ceres and the .deb carries
# uf_node itself.
#
# TWO PUBLISHED-ARTEFACT PROBLEMS ARE WORKED AROUND BELOW, both verified
# on 2026-08-17. Do not "simplify" either away without re-checking.
#
# 1. docs/ros2_humble_m3dgr.md tells you to pull
#    maotiandocker/ultrafusion-ros2:0.2.1. That tag does not exist:
#    Docker Hub has exactly one tag for this repository, 0.2.0.
# 2. The v0.2.2 .deb declares dependencies the 0.2.0 runtime image does
#    not carry (ros-humble-std-srvs, ros-humble-ros2service), so
#    `dpkg -i` leaves the package unconfigured and uf_node cannot
#    resolve libstd_srvs__rosidl_typesupport_cpp.so. std_srvs is what
#    the optional map-PCD Trigger service needs, so this is not
#    cosmetic. ros2topic is added too: the runtime image ships only the
#    bag/daemon/service CLI verbs, which makes the node impossible to
#    inspect from inside the container.
FROM maotiandocker/ultrafusion-ros2:0.2.0

ARG UF_VERSION=0.2.2
ARG UF_SHA256=243f88fa5e3d87fcd96a2b02c8561fca4d5e56419ab72ec0a3a731b0ea34cccc

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-std-srvs \
        ros-humble-ros2service \
        ros-humble-ros2topic \
        ca-certificates curl \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL -o /tmp/ultrafusion-ros2.deb \
      "https://github.com/sjtuyinjie/Ultra-Fusion/releases/download/v${UF_VERSION}/ultrafusion-ros2_${UF_VERSION}_amd64.deb" \
    && echo "${UF_SHA256}  /tmp/ultrafusion-ros2.deb" | sha256sum -c - \
    && dpkg -i /tmp/ultrafusion-ros2.deb \
    && ldconfig \
    && rm -f /tmp/ultrafusion-ros2.deb

# Fail the build rather than a run if the estimator cannot even link
RUN ldd /opt/ultrafusion/bin/uf_node | grep "not found" && exit 1 || true

WORKDIR /work
CMD ["bash"]
