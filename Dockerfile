# fog-sw BUILDER
FROM ros:foxy-ros-base as fog-sw-builder

# workaround for ROS GPG Key Expiration Incident
RUN rm /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update && \
    apt-get install -y curl && \
    curl http://repo.ros2.org/repos.key | sudo apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update

# Install build dependencies
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    curl \
    build-essential \
    dh-make debhelper \
    cmake \
    git-core \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build

COPY . .

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -r -y && \
    colcon build && \
    mkdir /packages && cd install && \
    find . -name '*.so' -exec cp --parents \{\} /packages \;"

FROM scratch
COPY --from=fog-sw-builder /packages/ /packages/
