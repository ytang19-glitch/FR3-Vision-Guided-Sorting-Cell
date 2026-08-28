FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV WORKSPACE=/workspace/ros2_ws

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

RUN mkdir -p ${WORKSPACE}/src

COPY start_container.sh /usr/local/bin/start_container.sh
RUN chmod +x /usr/local/bin/start_container.sh

WORKDIR /workspace

ENTRYPOINT ["/usr/local/bin/start_container.sh"]
CMD ["bash"]
