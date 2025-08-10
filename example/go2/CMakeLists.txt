
# add_executable(go2_trajectory_follow go2_trajectory_follow.cpp)
# target_link_libraries(go2_trajectory_follow unitree_sdk2)

add_executable(go2_sport_client go2_sport_client.cpp)
target_link_libraries(go2_sport_client unitree_sdk2)

add_executable(go2_low_level go2_low_level.cpp)
target_link_libraries(go2_low_level unitree_sdk2)

add_executable(go2_stand_example go2_stand_example.cpp)
target_link_libraries(go2_stand_example unitree_sdk2)


add_executable(go2_robot_state_client go2_robot_state_client.cpp)
target_link_libraries(go2_robot_state_client unitree_sdk2)

add_executable(go2_video_client go2_video_client.cpp)
target_link_libraries(go2_video_client unitree_sdk2)

add_executable(go2_vui_client go2_vui_client.cpp)
target_link_libraries(go2_vui_client unitree_sdk2)

# --- START OF MODIFIED BLOCK ---

# Find the required ROS2 packages. This is the crucial missing step.
# This command searches for the packages using the paths from your sourced environment.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add our new ROS node executable
add_executable(go2_lidar_ros go2_lidar_ros.cpp)

# Link it against the Unitree SDK and ROS 2 libraries
# The 'target_include_directories' is also added to ensure the compiler finds the headers.
target_include_directories(go2_lidar_ros PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
)

target_link_libraries(
    go2_lidar_ros
    PRIVATE
    unitree_sdk2
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
)

# --- END OF MODIFIED BLOCK ---