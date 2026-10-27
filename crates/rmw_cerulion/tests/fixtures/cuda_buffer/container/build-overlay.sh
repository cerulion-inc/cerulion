#!/usr/bin/env bash
# Inside the container: build cuda_buffer_backend (pinned sources in the
# image) and the capture probes into /opt/s2/ws. Tests are built too: the
# upstream sample nodes are test targets.
set -eo pipefail
# The ROS setup scripts read unset variables; -u stays off.
source /opt/ros/lyrical/setup.bash
mkdir -p /opt/s2/ws/src
ln -sfn /opt/s2/src/rosidl_buffer_backends /opt/s2/ws/src/rosidl_buffer_backends
ln -sfn /opt/s2/fixtures/probe /opt/s2/ws/src/cuda_buffer_s2_probe
cd /opt/s2/ws
colcon build --merge-install --event-handlers console_direct- \
  --packages-up-to cuda_buffer_backend cuda_buffer_s2_probe \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCUDAToolkit_ROOT=/usr/local/cuda-12.6
