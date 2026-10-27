# ROS 2 Humble + Pinocchio + ros2_control + RViz + gnuplot
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-humble-pinocchio \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-robot-state-publisher \
    ros-humble-realtime-tools \
    ros-humble-rviz2 \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    gnuplot-nox \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
 && echo "[ -f /sot/ros2/install/setup.bash ] && source /sot/ros2/install/setup.bash" >> ~/.bashrc

SHELL ["/bin/bash", "-c"]
WORKDIR /sot
CMD ["/bin/bash"]
