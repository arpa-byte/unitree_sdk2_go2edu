#!/bin/bash

# This script performs a full, clean build of the ROS 2 workspace,
# then sources the environment and launches the SLAM system.
# Use this after changing any C++ or CMakeLists.txt files.

# --- SCRIPT CONFIGURATION ---
# Set your computer's network interface name here.
NETWORK_INTERFACE="enp7s0"

# --- DO NOT EDIT BELOW THIS LINE ---

# Step 1: Navigate to the script's directory (your workspace root)
cd "$(dirname "$0")"

# Step 2: Clean the old build artifacts
echo "Cleaning previous builds..."
rm -rf build/ install/ log/

# Step 3: Build all packages in the workspace
echo "Building ROS 2 packages..."
colcon build --symlink-install

# Check if the build was successful
if [ $? -eq 0 ]; then
    echo "Build successful."
else
    echo "Build failed. Please check the error messages above."
    exit 1
fi

# Step 4: Source the ROS 2 Humble environment
echo "Sourcing bash file"
source ~/.bashrc


# Step 7: Launch the SLAM system
echo "Launching SLAM system on interface: $NETWORK_INTERFACE"
ros2 launch utlidar_launcher slam.launch.py network_interface:=$NETWORK_INTERFACE
