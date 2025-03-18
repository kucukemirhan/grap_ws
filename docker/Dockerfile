# Use the NVIDIA ROS2 Humble base image for L4T (Jetson devices)
FROM dustynv/ros:humble-ros-base-l4t-r36.3.0

# Install additional build dependencies (adjust as necessary)
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Set the working directory for your workspace
WORKDIR /home/ros

# Copy your GRAP workspace into the container
# Assumes your local directory structure has a folder named "grap_ws"
COPY grap_ws ./grap_ws

# Build the ROS2 workspace using colcon
RUN /bin/bash -c "source /opt/ros/humble/setup.sh && cd grap_ws && colcon build --symlink-install"

# Copy an entrypoint script that sources ROS2 and the workspace setup
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set any required environment variables
ENV ROS_DOMAIN_ID=0
ENV CUDA_VISIBLE_DEVICES=0

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command can be overridden; here we start with an interactive shell
CMD ["bash"]
