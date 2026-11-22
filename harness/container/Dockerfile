# SPDX-License-Identifier: AGPL-3.0-only
# A recipe for one image that runs a benchmark cell with and without the Cerulion harness: Linux x86-64 or
# arm64, CPU rendering, no GPU. No image is published; build it yourself, from the repository root:
#
#   docker build -f harness/container/Dockerfile -t cerulion-harness harness
#   docker run --rm -e OPENAI_BASE_URL -e OPENAI_API_KEY -v "$PWD/out:/out" cerulion-harness \
#       run-cell --model YOUR_MODEL_NAME
#
# harness/.dockerignore keeps what install.sh writes out of the build context. docker build picks the platform
# of the machine it runs on (on a Mac with Apple silicon: linux/arm64). The image runs the same install.sh a
# machine without a container runs; on a platform other than x86-64 that includes the robot's one-time
# self-test for the default start state's body (python -m cerulion_harness.body_record says why).
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl git python3 python3-venv tini \
        libosmesa6 libegl1 libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*
COPY --from=ghcr.io/astral-sh/uv:0.13.7 /uv /usr/local/bin/uv

ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python \
    UV_LINK_MODE=copy \
    UV_NO_CACHE=1 \
    PYTHONDONTWRITEBYTECODE=1
COPY . /opt/harness
RUN /opt/harness/install.sh

# Results go to /out: mount a folder there. Each run writes a new folder cerulion-bench-UTCTIME in it.
WORKDIR /out
VOLUME /out
# tini reaps the simulator and server processes an episode leaves behind when it ends.
ENTRYPOINT ["/usr/bin/tini", "--", "/opt/harness/venv/bin/python", "-m", "cerulion_harness"]
CMD ["--help"]
