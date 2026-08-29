FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV WORKSPACE=/workspace/ros2_ws
ENV FRANKA_WS=/opt/franka_ros2_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    nano \
    curl \
    usbutils \
    v4l-utils \
    python3-opencv \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-dev-tools \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-rqt-image-view \
    ros-jazzy-rviz2 \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-moveit \
    ros-jazzy-moveit-py \
    ros-jazzy-ros-gz \
    ros-jazzy-realsense2-camera \
    ros-jazzy-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true && rosdep update

RUN mkdir -p "${FRANKA_WS}/src" && \
    git clone \
      --branch jazzy \
      --depth 1 \
      https://github.com/frankarobotics/franka_ros2.git \
      "${FRANKA_WS}/src/franka_ros2"

RUN vcs import "${FRANKA_WS}/src" \
      < "${FRANKA_WS}/src/franka_ros2/dependency.repos" \
      --recursive \
      --skip-existing

RUN apt-get update && \
    source /opt/ros/jazzy/setup.bash && \
    rosdep install \
      --from-paths "${FRANKA_WS}/src" \
      --ignore-src \
      --rosdistro jazzy \
      -r -y \
      --skip-keys=zed_wrapper && \
    rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/jazzy/setup.bash && \
    cd "${FRANKA_WS}" && \
    colcon build \
      --symlink-install \
      --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_TESTS=OFF

RUN mkdir -p "${WORKSPACE}/src"

COPY start_container.sh /usr/local/bin/start_container.sh
RUN chmod +x /usr/local/bin/start_container.sh

WORKDIR /workspace

ENTRYPOINT ["/usr/local/bin/start_container.sh"]
CMD ["bash"]
