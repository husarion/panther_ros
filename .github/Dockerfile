ARG ROS_DISTRO=humble
ARG BUILD_TEST=ON

FROM husarnet/ros:${ROS_DISTRO}-ros-core

ENV HUSARION_ROS_BUILD_TYPE=hardware

STOPSIGNAL SIGINT

WORKDIR /ros2_ws

COPY ./ src/panther_ros

RUN apt-get update  && \
    apt-get install -y \
    ros-dev-tools && \
    # Setup workspace
    vcs import src < src/panther_ros/panther/panther_hardware.repos && \
    cp -r src/ros2_controllers/imu_sensor_broadcaster src && rm -rf src/ros2_controllers && \
    # Install dependencies
    rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --from-paths src -y -i && \
    # Build
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install --packages-up-to panther --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=$BUILD_TEST && \
    # Size optimalization
    export SUDO_FORCE_REMOVE=yes && \
    apt-get remove -y \
    ros-dev-tools && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
