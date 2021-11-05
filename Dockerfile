# fog-sw BUILDER
FROM ros:galactic-ros-base as fog-sw-builder

# Install build dependencies
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    curl \
    build-essential \
    dh-make debhelper \
    cmake \
    git-core \
    ros-galactic-camera-info-manager \
    ros-galactic-fastrtps \
    ros-galactic-rmw-fastrtps-cpp \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

WORKDIR /build

COPY . .

RUN /bin/bash -c "source /opt/ros/galactic/setup.bash && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y && \
    colcon build && \
    mkdir /packages && cd install && \
    find . -name '*.so' -exec cp --parents \{\} /packages \;"

FROM scratch
COPY --from=fog-sw-builder /packages/ /packages/
