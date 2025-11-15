#!/bin/bash

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

echo "====================================="
echo "Alvik SLAM & Navigation System"
echo "====================================="
echo "Starting all ROS2 nodes..."
echo ""

# Start nodes in background
echo "Starting ToF receiver..."
python3 /ros2_ws/src/alvik_navigation/alvik_navigation/tof_receiver.py --ros-args -p mqtt_host:=192.168.1.152 &

echo "Starting odometry publisher..."
python3 /ros2_ws/src/alvik_navigation/alvik_navigation/odom_publisher.py --ros-args -p mqtt_host:=192.168.1.152 &

echo "Starting SLAM toolbox..."
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /ros2_ws/src/alvik_navigation/config/mapper_params_online_async.yaml &

echo "Starting click-to-move navigation..."
python3 /ros2_ws/src/alvik_navigation/alvik_navigation/click_to_move.py --ros-args -p mqtt_host:=192.168.1.152 &

# Wait for SLAM to initialize
sleep 2

# Activate SLAM lifecycle node
echo "Activating SLAM toolbox..."
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate

echo ""
echo "All nodes started!"
echo "Foxglove Bridge: ws://localhost:8765"
echo "SLAM is active and building maps!"
echo "Click points in Foxglove 3D view to navigate!"
echo ""

# Wait for all background processes
wait
