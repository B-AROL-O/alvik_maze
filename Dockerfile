# ROS2 Jazzy with GUI support for macOS (ARM64 compatible)
FROM --platform=linux/arm64 arm64v8/ros:jazzy-ros-base

# Install desktop components and dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-default-plugins \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    mosquitto \
    mosquitto-clients \
    vim \
    wget \
    curl \
    net-tools \
    iputils-ping \
    libgl1-mesa-dri \
    libglx-mesa0 \
    libopengl0 \
    libosmesa6 \
    libosmesa6-dev \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages one at a time with --break-system-packages flag
RUN pip3 install --no-cache-dir --break-system-packages paho-mqtt==1.6.1 || \
    pip3 install --no-cache-dir --break-system-packages --no-deps paho-mqtt==1.6.1

RUN pip3 install --no-cache-dir --break-system-packages numpy || \
    apt-get update && apt-get install -y python3-numpy && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --break-system-packages transforms3d

RUN pip3 install --no-cache-dir --break-system-packages matplotlib; exit 0

# Set up ROS2 workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Configure Cyclone DDS for better performance
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Source ROS2 in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Expose ports for rviz2 and MQTT
EXPOSE 1883 11311

# Set up entrypoint
COPY scripts/start.sh /start.sh
RUN chmod +x /start.sh

ENTRYPOINT ["/start.sh"]
CMD ["bash"]
