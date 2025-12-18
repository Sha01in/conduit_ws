# Base Image: ROS 2 Jazzy (Native support for Apple Silicon/ARM64)
FROM ros:jazzy-ros-base

# Install python dependencies for the API bridge
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace
WORKDIR /root/conduit_ws

# Auto-source ROS 2 in every new shell
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Default command
CMD ["/bin/bash"]