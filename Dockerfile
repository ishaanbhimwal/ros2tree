FROM ros:humble-ros-base-jammy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install Python dependencies and utilities
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-ament-index-python \
    ros-humble-rclpy \
    ros-humble-rosidl-runtime-py \
    && rm -rf /var/lib/apt/lists/*

# Create a working directory
WORKDIR /opt/ros2tree

# Copy the ros2tree scripts
COPY ros2tree.py ros2tree_node.py ros2tree_topic.py ./

# Make the scripts executable
RUN chmod +x ros2tree.py ros2tree_node.py ros2tree_topic.py

# Install the scripts into /usr/local/bin (as per README instructions)
RUN install -m 0755 ros2tree.py /usr/local/bin/ros2tree && \
    install -m 0755 ros2tree_node.py /usr/local/bin/ros2tree_node && \
    install -m 0755 ros2tree_topic.py /usr/local/bin/ros2tree_topic

# Source ROS 2 environment in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the entrypoint to source ROS 2 and run bash
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec \"$@\"", "--"]
CMD ["/bin/bash"]
