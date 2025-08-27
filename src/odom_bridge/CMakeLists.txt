cmake_minimum_required(VERSION 3.8)
project(odom_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(odom_bridge_node src/odom_bridge_node.cpp)

ament_target_dependencies(odom_bridge_node
  "rclcpp"
  "nav_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"

)

# --- THE CRITICAL FIX IS HERE ---
# We must provide all the same include paths as the lidar_bridge
# to ensure the compiler can find all required DDS and SDK headers.
target_include_directories(odom_bridge_node PUBLIC
  "/home/arpan/unitree2/unitree_sdk2/include"
  "/home/arpan/unitree2/unitree_sdk2/thirdparty/include/ddscxx"
  "/home/arpan/unitree2/unitree_sdk2/thirdparty/include"
)

# The link libraries section is correct.
target_link_libraries(odom_bridge_node
  "/home/arpan/unitree2/unitree_sdk2/lib/x86_64/libunitree_sdk2.a"
  "/home/arpan/unitree2/unitree_sdk2/thirdparty/lib/x86_64/libddscxx.so"
  "/home/arpan/unitree2/unitree_sdk2/thirdparty/lib/x86_64/libddsc.so"
  Threads::Threads
)

target_compile_definitions(odom_bridge_node PRIVATE UNITREE_SDK_VERSION_2)

install(
  TARGETS odom_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()