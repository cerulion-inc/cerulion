# Dockerfile — benches/latency ROS 2 comparison image.
#
# One image per distro; one container per cell (bench.py ros2).
#
# ┌─────────────────────────────────────────────────────────────────────┐
# │ BUILD CONTEXT = THE REPO ROOT, NOT THIS DIRECTORY.                  │
# │                                                                     │
# │   DOCKER_BUILDKIT=1 docker build \                                  │
# │       --build-arg ROS_DISTRO=jazzy \                                │
# │       -t latency_bench:jazzy \                                      │
# │       -f benches/latency/ros2/docker/Dockerfile \                   │
# │       .                                # <- repo root               │
# │                                                                     │
# │ The COPY paths below are repo-root-relative. BuildKit is REQUIRED: │
# │ the co-located Dockerfile.dockerignore (BuildKit's per-Dockerfile  │
# │ ignore file) keeps .git/ and target/ out of the context. Building  │
# │ with the legacy builder would tar tens of GB of target/ into the   │
# │ context — if the build context upload is huge, you forgot BuildKit.│
# └─────────────────────────────────────────────────────────────────────┘
#
# Distros: ARG ROS_DISTRO ∈ {humble, jazzy, lyrical}   (kilted dropped)
#   jazzy   — the proven pattern (ros2-bench container, May-2026 campaign).
#   humble  — stock-RMW cells expected to work (May campaign ran humble
#             on a host; container path verified separately). rmw_zenoh
#             has NO humble apt package: the defensive install below
#             warns, and zenoh cells FAIL LOUDLY at run time.
#   lyrical — verified 2026-08-12.
#
# rmw_cerulion (removed from this suite once, then re-admitted) ships
# as a TOOLCHAIN, not a binary: the image carries rust + libclang, and
# run_bench.sh's ensure_rmw_cerulion builds librmw_cerulion.so inside the
# cell's own container from the repo bind-mounted at /work. The old
# rmw_builder stage is NOT coming back, and the reason is the same one
# that killed it: a .so baked at image-build time is bound to whichever
# distro's headers were present then, while the crate's bindgen step must
# match the distro it will actually run against (introspection struct
# layouts drift between them). Building per-cell also makes freshness
# structural: the binary measured is the one built from the tree this
# run was launched from, not whatever an older image happened to carry.
#
# What the final image contains:
#   - ROS 2 base (`ros:${ROS_DISTRO}`)
#   - stock RMWs: rmw_cyclonedds_cpp, rmw_fastrtps_cpp, rmw_zenoh_cpp
#     (zenoh + iceoryx-posh installed defensively — absent packages
#     warn at build and fail loudly at run time)
#   - iceoryx RouDi (for CycloneDDS SHM transport)
#   - colcon-built ros2_rtt_msgs + ros2_rtt_bench packages
#   - a Rust stable toolchain + libclang, for the rmw_cerulion cdylib
#     built at run time (cerulion cells only; inert for every other lane)
#
# The image runs as root so chrt -f 80 works without sudo. Mount the
# host's /dev/cpu_dma_latency in (--device) for the CPU DMA lock;
# without it the bench warns and continues (p99 may degrade).

ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}
ARG ROS_DISTRO

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# Stock RMW backends + build/tooling.
#   - rmw_zenoh_cpp ships in apt for jazzy+; humble has none (host
#     overlays were the old path). Defensive install: warn at build,
#     fail loudly at run time (run_bench.sh checks `ros2 pkg prefix`).
#   - iceoryx-posh (iox-roudi, for CycloneDDS SHM) is defensive for
#     the same reason on lyrical (its apt package name is unverified);
#     run_bench.sh hard-fails a cyclonedds+shm cell if iox-roudi is
#     absent, so a missing package cannot silently mislabel a cell.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
        ros-${ROS_DISTRO}-std-msgs \
        ros-${ROS_DISTRO}-sensor-msgs \
        ros-${ROS_DISTRO}-rosidl-default-generators \
        ros-${ROS_DISTRO}-rosidl-default-runtime \
        cmake build-essential pkg-config \
        python3-colcon-common-extensions python3-colcon-mixin \
        procps psmisc util-linux iproute2 \
        && (apt-get install -y --no-install-recommends \
              ros-${ROS_DISTRO}-rmw-zenoh-cpp \
              || echo "WARN: ros-${ROS_DISTRO}-rmw-zenoh-cpp not in apt — zenoh cells will FAIL LOUDLY at run time") \
        && (apt-get install -y --no-install-recommends \
              ros-${ROS_DISTRO}-iceoryx-posh \
              || echo "WARN: ros-${ROS_DISTRO}-iceoryx-posh not in apt — cyclonedds+shm cells will FAIL LOUDLY at run time (no iox-roudi)") \
        && rm -rf /var/lib/apt/lists/*

# iceoryx-iox-roudi binary lives under iceoryx-posh in newer distros;
# create a stable name on PATH when present.
RUN if [ -x "/opt/ros/${ROS_DISTRO}/bin/iox-roudi" ]; then \
        ln -sf "/opt/ros/${ROS_DISTRO}/bin/iox-roudi" /usr/local/bin/iox-roudi ; \
    else \
        echo "WARN: iox-roudi not found in /opt/ros/${ROS_DISTRO}/bin — CycloneDDS+SHM will not be available" ; \
    fi

# Rust toolchain for the rmw_cerulion cdylib that cerulion cells build at
# run time (see the banner above). Unconditional rather than behind a
# build ARG on purpose: a gated install would produce two DIFFERENT
# images under one `latency_bench:<distro>` tag, and nothing downstream
# (the run manifest records an image id and a repo sha, not a feature set)
# could tell an operator which of the two a cell ran against. The cost
# is carried by every image so that the tag means one thing.
#
# `stable`, matching tools/ros2_toolchain/Dockerfile: the workspace pins no
# toolchain file, and the pinned iceoryx2 0.10.0 does not build on current
# nightlies.
#   clang + libclang-dev: bindgen (build-dependency) needs libclang to
#     parse the distro's rmw/rosidl headers.
#   g++ + pkg-config: already present via build-essential above; the `cc`
#     build-dependency compiles shim/cppstring_shim.cpp with them.
#   curl + ca-certificates: fetching rustup itself.
RUN apt-get update && apt-get install -y --no-install-recommends \
        clang libclang-dev llvm-dev curl ca-certificates git \
        && rm -rf /var/lib/apt/lists/*
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
        | sh -s -- -y --no-modify-path --default-toolchain stable
ENV PATH="/root/.cargo/bin:${PATH}"

WORKDIR /bench

# Bench sources + configs + drivers (paths are REPO-ROOT-relative —
# see the build-context banner at the top).
COPY benches/latency/ros2/ros2_rtt_msgs  /bench/ros2_ws/src/ros2_rtt_msgs
COPY benches/latency/ros2/ros2_rtt_bench /bench/ros2_ws/src/ros2_rtt_bench
COPY benches/latency/ros2/configs        /bench/configs
COPY benches/latency/ros2/run_bench.sh   /bench/run_bench.sh
COPY benches/latency/ros2/verify_shm.sh  /bench/verify_shm.sh

# Build the colcon workspace inside the container.
RUN /bin/bash -c "set -e; \
    source /opt/ros/${ROS_DISTRO}/setup.bash; \
    cd /bench/ros2_ws; \
    colcon build --packages-select ros2_rtt_msgs ros2_rtt_bench \
        --cmake-args -DCMAKE_BUILD_TYPE=Release"

ENV ROS_SETUP_BASH=/opt/ros/${ROS_DISTRO}/setup.bash
ENV ROS_DISTRO_NAME=${ROS_DISTRO}

# Default: run run_bench.sh with whatever cell envelope the container
# was given (bench.py passes RMWS/SHM_MODE/CHRT_MODE/RECV_PATH/
# CER_BENCH_* via -e). Override with `docker run ... bash` to debug.
#
# Per-netns UDP fragmentation reassembly cap (net.ipv4.ipfrag_high_
# thresh) is set via the docker run `--sysctl` flag at container-
# creation time for bridge-network cells; `--network host` cells
# inherit the host's value. An in-container `sysctl -w` would fail
# under the default read-only /proc/sys mount, so we don't attempt one.
CMD ["/bin/bash", "-lc", "source /opt/ros/${ROS_DISTRO_NAME}/setup.bash && /bench/run_bench.sh"]
