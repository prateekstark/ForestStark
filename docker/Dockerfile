# ForestStark: one image per role, built from the same workspace.
#
#   docker build -f docker/Dockerfile --target robot -t foreststark:robot .   # on the machine (headless)
#   docker build -f docker/Dockerfile --target sim   -t foreststark:sim   .   # Gazebo + RViz workstation
#   docker build -f docker/Dockerfile --target test  .                        # build and run every test
ARG ROS_DISTRO=lyrical

# ------------------------------------------------------------------ build: workspace and its deps
FROM ros:${ROS_DISTRO}-ros-base AS build
ARG ROS_DISTRO
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /ws
COPY ros2_ws/src src
RUN apt-get update && rosdep update --rosdistro ${ROS_DISTRO} \
 && rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} \
 && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build --event-handlers console_cohesion+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# ------------------------------------------------------------------ test: unit tests and the physics audit
FROM build AS test
RUN . install/setup.bash \
 && colcon test --event-handlers console_direct+ && colcon test-result --verbose \
 && ros2 run fs_bringup physics_audit > /ws/physics_audit.md && ! grep -q "FAIL" /ws/physics_audit.md

# ------------------------------------------------------------------ robot: runtime only, no compilers
FROM ros:${ROS_DISTRO}-ros-base AS robot
ARG ROS_DISTRO
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive ROS_DISTRO=${ROS_DISTRO} FASTDDS_BUILTIN_TRANSPORTS=UDPv4
WORKDIR /ws
COPY --from=build /ws/src /tmp/src
RUN apt-get update && rosdep update --rosdistro ${ROS_DISTRO} \
 && rosdep install -y -r --from-paths /tmp/src --ignore-src --rosdistro ${ROS_DISTRO} --dependency-types exec \
 && rm -rf /var/lib/apt/lists/* /tmp/src
COPY --from=build /ws/install /ws/install
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "fs_bringup", "robot.launch.py"]

# ------------------------------------------------------------------ sim: robot image + Gazebo GUI and RViz
FROM robot AS sim
RUN apt-get update && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rviz2 \
 && rm -rf /var/lib/apt/lists/*
CMD ["ros2", "launch", "fs_bringup", "katar.launch.py"]
