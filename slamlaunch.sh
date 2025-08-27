#!/bin/bash

# This script is used to launch the SLAM system after it has been built.

# --- SCRIPT CONFIGURATION ---
# Set your computer's network interface name here.
# Find it by running 'ip addr' in a terminal. Common names are eno1, enp7s0, wlp2s0.
NETWORK_INTERFACE="enp7s0"

# Step 1: Navigate to the script's directory (your workspace root)
cd "$(dirname "$0")"

# Step to source all at once
echo "Sourcing bash file"
source ~/.bashrc

# Step 5: Launch the SLAM system
echo "Launching SLAM system on interface: $NETWORK_INTERFACE"
ros2 launch utlidar_launcher slam.launch.py network_interface:=$NETWORK_INTERFACE
