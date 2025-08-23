cmake_minimum_required(VERSION 3.8)
project(lidar_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)

# Add the executable from our source file
add_executable(lidar_bridge_node src/lidar_bridge_node.cpp)

# Link against the ROS 2 libraries (this uses the 'plain' style internally)
ament_target_dependencies(lidar_bridge_node
  "rclcpp"
  "sensor_msgs"
  "rosgraph_msgs"
)
# Manually specify the location of the Unitree SDK headers
target_include_directories(lidar_bridge_node PUBLIC
  "/home/arpan/unitree2/unitree_sdk2/include"
  "/home/arpan/unitree2/unitree_sdk2/thirdparty/include/ddscxx"
  "/home/arpan/unitree2/unitree_sdk2/thirdparty/include"
)

# --- THE CRITICAL FIX IS HERE ---
# Manually link the Unitree SDK libraries using the same 'plain' style.
# We have removed the 'PRIVATE' keyword to match the style of ament_target_dependencies.
target_link_libraries(lidar_bridge_node
  "/home/arpan/unitree2/unitree_sdk2/lib/x86_64/libunitree_sdk2.a"
  "/home/arpan/unitree2/unitree_sdk2/thirdparty/lib/x86_64/libddscxx.so"
  "/home/arpan/unitree2/unitree_sdk2/thirdparty/lib/x86_64/libddsc.so"
  Threads::Threads
)

# Apply the crucial compiler flag for the older SDK API
target_compile_definitions(lidar_bridge_node PRIVATE UNITREE_SDK_VERSION_2)

# Install the executable so ros2 launch can find it
install(
  TARGETS lidar_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
