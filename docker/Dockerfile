# ROS 2 Humble (officiell image från Docker Hub) + Pinocchio
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-humble-pinocchio \
    ros-humble-xacro \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

SHELL ["/bin/bash", "-c"]
WORKDIR /sot
CMD ["/bin/bash"]
