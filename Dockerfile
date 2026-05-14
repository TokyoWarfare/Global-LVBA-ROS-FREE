# Global-LVBA build environment.
#
# Pattern: container provides the build-time dependencies only. Source code
# lives on the host (so editor / git / search all work as normal) and is
# mounted into /opt/lvba at runtime. cmake + make run inside the container
# against the mounted source -- build artefacts land in host's build/ dir
# because it's the same filesystem.
#
# Build the image once (rebuild only if deps change):
#   cd /Host_home/pablo/lvba_ws/Global-LVBA
#   docker build -t lvba:dev .
#
# Use it (mount source + dataset, drop into a shell):
#   docker run --gpus all --rm -it \
#     -v /Host_home/pablo/lvba_ws/Global-LVBA:/opt/lvba \
#     -v /data:/data \
#     lvba:dev
#
# Inside the container, first time only (or whenever SiftGPU changes):
#   cd /opt/lvba/src/SiftGPU && mkdir -p build && cd build && cmake .. && make -j
#   cd /opt/lvba && mkdir -p build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j
#
# Then run:
#   ./build/lvba_run /data/lvba_dataset/CBD_Building_01/config.yaml
#
# Edit source on host, re-make inside container -- no image rebuild needed.

ARG CUDA_VERSION=12.2.0
FROM nvidia/cuda:${CUDA_VERSION}-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8

# Build toolchain + LVBA's listed prerequisites (Eigen, OpenCV, PCL, Ceres,
# Sophus, yaml-cpp, sqlite3) + SiftGPU's GL/GLEW/GLUT deps. Ubuntu 22.04's
# libceres-dev ships 2.1.0 which matches the README pin.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        pkg-config \
        ca-certificates \
        libeigen3-dev \
        libopencv-dev \
        libpcl-dev \
        libceres-dev \
        libsqlite3-dev \
        libyaml-cpp-dev \
        libglew-dev \
        libglu1-mesa-dev \
        freeglut3-dev \
        libopengl-dev \
        libgl1-mesa-dev \
        libomp-dev \
        gdb \
        vim \
    && rm -rf /var/lib/apt/lists/*

# Sophus: LVBA uses the OLD non-templated API (`#include "sophus/se3.h"`,
# `Sophus::SE3` without a template parameter). The current strasdat/Sophus
# repo is header-only with the templated API, and Ubuntu 22.04 has no
# libsophus-dev package. So we build the legacy version from source -- the
# specific commit below is the last one with the libSophus.so + se3.h API.
#
# Two patches applied before build:
#   1. C++11 deprecated assigning to `std::complex<T>::real()` / `.imag()`;
#      switch to the setter form `.real(x)` / `.imag(x)` so the `-Werror`
#      build doesn't choke.
#   2. The old CMakeLists has `Target "Sophus" links to itself` (CMP0038);
#      not fatal at warning level but harmless to also drop the duplicate
#      target_link_libraries line.
RUN git clone https://github.com/strasdat/Sophus.git /tmp/Sophus \
    && cd /tmp/Sophus \
    && git checkout a621ff2 \
    && sed -i 's/unit_complex_\.real() = \(.*\);/unit_complex_.real(\1);/' sophus/so2.cpp \
    && sed -i 's/unit_complex_\.imag() = \(.*\);/unit_complex_.imag(\1);/' sophus/so2.cpp \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && rm -rf /tmp/Sophus

# Source is mounted in at /opt/lvba; no COPY. cwd defaults to it so the
# typical workflow (`cd build && cmake --build .`) needs no path juggling.
WORKDIR /opt/lvba

CMD ["/bin/bash"]
