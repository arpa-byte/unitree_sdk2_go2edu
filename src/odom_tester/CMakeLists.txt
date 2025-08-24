cmake_minimum_required(VERSION 3.8)
project(odom_tester)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

# Add our executable
add_executable(odom_tester_node src/odom_tester_node.cpp)

# Link against ROS 2 libraries
ament_target_dependencies(odom_tester_node
  "rclcpp"
  "nav_msgs"
)

# Manually specify the location of the Unitree SDK headers
target_include_directories(odom_tester_node PUBLIC
  "/home/arpan/unitree2/unitree_sdk2/include"
  "/home/arpan/unitree2/unitree_sdk2/thirdparty/include/ddscxx"
  "/home/arpan/unitree2/unitree_sdk2/thirdparty/include"
)

# Manually link against the pre-compiled Unitree SDK libraries
target_link_libraries(odom_tester_node
  "/home/arpan/unitree2/unitree_sdk2/lib/x86_64/libunitree_sdk2.a"
  "/home/arpan/unitree2/unitree_sdk2/thirdparty/lib/x86_64/libddscxx.so"
  "/home/arpan/unitree2/unitree_sdk2/thirdparty/lib/x86_64/libddsc.so"
  Threads::Threads
)

# Apply the compiler flag for the older SDK API
target_compile_definitions(odom_tester_node PRIVATE UNITREE_SDK_VERSION_2)

# Install the executable so ros2 run can find it
install(
  TARGETS odom_tester_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
