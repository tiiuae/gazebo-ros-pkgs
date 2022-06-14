# BUILDER
#FROM ros:galactic-ros-base as builder
FROM ghcr.io/tiiuae/fog-ros-baseimage:builder-latest AS builder

# Install build dependencies
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    ros-galactic-camera-info-manager \
    nlohmann-json3-dev \
    pkg-config \
    libopencv-imgproc-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

COPY . .

RUN /bin/bash -c "source /opt/ros/galactic/setup.bash && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y && \
    colcon build && \
    mkdir /packages && cd install && \
    find . -name '*.so' -exec cp {} /packages \;"

FROM busybox
COPY --from=builder /packages/ /artifacts/plugins
