# ROS 2 Lyrical underlay for the CUDA rosidl::Buffer interop fixtures, on a
# Jetson (L4T R36, CUDA 12.6) host.
#
# The image carries Lyrical binaries (Ubuntu 26.04 "resolute", arm64), both
# buffer-aware rmws (rmw_fastrtps_cpp, rmw_zenoh_cpp), the build tools for an
# overlay, and the upstream backend sources at one pinned commit. It carries
# no CUDA: the L4T nvidia runtime mounts only the driver (libcuda), so
# container/run.sh bind-mounts the host toolkit /usr/local/cuda-12.6
# read-only, and the overlay is built at run time (container/build-overlay.sh).
#
# Build on the box from this directory:
#   docker build --network host -t cerulion/lyrical-cuda:s2 .
FROM ros@sha256:f79cd486fa218596747c45d00ca42101b0303fc6d94ed5adb72145cd6f54b86c

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git strace python3-pytest util-linux \
      ros-dev-tools \
      ros-lyrical-rmw-fastrtps-cpp \
      ros-lyrical-rmw-zenoh-cpp \
      ros-lyrical-rclcpp-components \
      ros-lyrical-sensor-msgs \
      ros-lyrical-std-msgs \
      ros-lyrical-pluginlib \
      ros-lyrical-rosidl-buffer \
      ros-lyrical-rosidl-buffer-backend \
      ros-lyrical-rosidl-buffer-backend-registry \
      ros-lyrical-rosidl-buffer-py \
      ros-lyrical-rosidl-default-generators \
      ros-lyrical-ament-cmake-auto \
      ros-lyrical-ament-cmake-gtest \
      ros-lyrical-ament-cmake-pytest \
      ros-lyrical-ament-lint-auto \
      ros-lyrical-ament-lint-common \
      ros-lyrical-launch-testing-ament-cmake \
      ros-lyrical-launch-testing-ros \
    && rm -rf /var/lib/apt/lists/*

# Upstream backend sources, pinned. The checkout is verified against the
# full commit id so a moved ref fails the build.
ARG RBB_COMMIT=4f3b257db3a5cc4c8e62d526f11f344c1e5801dc
RUN git clone https://github.com/ros2/rosidl_buffer_backends.git /opt/s2/src/rosidl_buffer_backends \
 && git -C /opt/s2/src/rosidl_buffer_backends checkout --detach "${RBB_COMMIT}" \
 && test "$(git -C /opt/s2/src/rosidl_buffer_backends rev-parse HEAD)" = "${RBB_COMMIT}"

# The toolkit path the overlay build and the runtime expect; run.sh mounts it.
# The L4T runtime injects the driver only when NVIDIA_VISIBLE_DEVICES is set,
# and mounts it under /usr/lib/aarch64-linux-gnu/nvidia, which the loader
# must search.
RUN ln -s /usr/local/cuda-12.6 /usr/local/cuda \
 && echo /usr/lib/aarch64-linux-gnu/nvidia > /etc/ld.so.conf.d/nvidia-tegra.conf
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    CUDA_HOME=/usr/local/cuda-12.6 \
    PATH=/usr/local/cuda-12.6/bin:${PATH} \
    LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:/usr/lib/aarch64-linux-gnu/nvidia

# A non-root user with a second uid, for the cross-uid CPU-fallback fixture.
RUN useradd -m -u 1001 s2other

RUN dpkg-query -W -f '${Package}\t${Version}\n' 'ros-lyrical-*' > /opt/s2/ros-lyrical-packages.tsv
WORKDIR /opt/s2
