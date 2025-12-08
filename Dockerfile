# Use ROS2 Humble base image
FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-serial \
    ros-humble-xacro \
    ros-humble-sensor-msgs \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    ros-humble-joint-state-broadcaster \
    ros-humble-diff-drive-controller \
    ros-humble-robot-state-publisher \
    libserial-dev \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /workspace

# Copy source files
COPY src/ /workspace/src/
COPY launch/ /workspace/launch/

# Install dependencies using rosdep (skip unavailable packages)
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source ROS2 and workspace in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Set entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && exec \"$@\"", "--"]
CMD ["/bin/bash"]
