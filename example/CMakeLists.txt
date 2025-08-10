#NEW CMAKE FILE FOR KEYBOARD CONTROL EXAMPLE

add_subdirectory(helloworld)
add_subdirectory(wireless_controller)
add_subdirectory(jsonize)
add_subdirectory(state_machine)


add_subdirectory(go2)
add_subdirectory(b2)
add_subdirectory(h1)
add_subdirectory(g1)
add_subdirectory(go2w)
add_subdirectory(b2w)

# ------------------------------------------------------------------
# --- SECTION TO BUILD keyboard_control ---
# ------------------------------------------------------------------


add_executable(keyboard_control keyboard_control.cpp)

target_include_directories(keyboard_control PUBLIC
	${CMAKE_SOURCE_DIR}/include
	${CMAKE_SOURCE_DIR}/thirdparty/include/ddscxx
	${CMAKE_SOURCE_DIR}/thirdparty/include
)

find_package(Threads REQUIRED)
target_link_libraries(keyboard_control PRIVATE
	${CMAKE_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_sdk2.a
	${CMAKE_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}/libddscxx.so
	${CMAKE_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}/libddsc.so
	Threads::Threads
)		

get_target_property(KC_INCLUDES keyboard_control INCLUDE_DIRECTORIES)
message(STATUS "Include directories for keyboard_control: ${KC_INCLUDES}")

# ------------------------------------------------------------------
# --- SECTION TO BUILD another_keyboard_control) ---
# ------------------------------------------------------------------

add_executable(another_keyboard_control another_keyboard_control.cpp)
target_include_directories(another_keyboard_control PUBLIC
	${CMAKE_SOURCE_DIR}/include
	${CMAKE_SOURCE_DIR}/thirdparty/include/ddscxx
	${CMAKE_SOURCE_DIR}/thirdparty/include
)
target_link_libraries(another_keyboard_control PRIVATE
	${CMAKE_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_sdk2.a
	${CMAKE_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}/libddscxx.so
	${CMAKE_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}/libddsc.so
	Threads::Threads
)	

get_target_property(KC_INCLUDES another_keyboard_control INCLUDE_DIRECTORIES)
message(STATUS "Include directories for another_keyboard_control: ${KC_INCLUDES}")



# ------------------------------------------------------------------
# --- NEW, CORRECTED SECTION FOR hand_instruct EXAMPLE ---
# ------------------------------------------------------------------

# 1. Find dependencies
find_package(OpenCV REQUIRED)

# Define ONNX Runtime paths explicitly
set(ONNXRUNTIME_DIR "/home/arpan/unitree2/handmodel/onnxruntime-linux-x64-1.18.0")
set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_DIR}/include")
set(ONNXRUNTIME_LIBRARY "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so")

# 2. Add the executable with the consistent name 'hand_instruct'
add_executable(hand_instruct hand_instruct.cpp)

# 3. Add include directories for 'hand_instruct'
target_include_directories(hand_instruct PUBLIC
    # Unitree SDK
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/thirdparty/include/ddscxx
    ${CMAKE_SOURCE_DIR}/thirdparty/include
    # OpenCV
    ${OpenCV_INCLUDE_DIRS}
    # ONNX Runtime
    ${ONNXRUNTIME_INCLUDE_DIR}
)

# 4. Link libraries to 'hand_instruct'
target_link_libraries(hand_instruct PRIVATE
    # Unitree SDK
    ${CMAKE_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_sdk2.a
    ${CMAKE_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}/libddscxx.so
    ${CMAKE_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}/libddsc.so
    Threads::Threads
    # OpenCV
    ${OpenCV_LIBS}
    # ONNX Runtime
    ${ONNXRUNTIME_LIBRARY}
)

# ------------------------------------------------------------------
# --- SECTION TO BUILD lidar_defaultcode (ROS 2 VERSION) ---
# ------------------------------------------------------------------

# 1. Find all required ROS 2 packages (rclcpp and sensor_msgs)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

# 2. Add the executable
add_executable(lidar_defaultcode lidar_defaultcode.cpp)

# 3. Add all necessary include directories
target_include_directories(lidar_defaultcode PUBLIC
	# Unitree SDK paths
	${CMAKE_SOURCE_DIR}/include
	${CMAKE_SOURCE_DIR}/thirdparty/include/ddscxx
	${CMAKE_SOURCE_DIR}/thirdparty/include
	# ROS 2 package paths
	${rclcpp_INCLUDE_DIRS}
	${sensor_msgs_INCLUDE_DIRS}
)

# 4. Link all necessary libraries
target_link_libraries(lidar_defaultcode PRIVATE
	# Unitree SDK libs
	${CMAKE_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_sdk2.a
	${CMAKE_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}/libddscxx.so
	${CMAKE_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}/libddsc.so
	Threads::Threads
	# ROS 2 package libs
	${rclcpp_LIBRARIES}
	${sensor_msgs_LIBRARIES}
)

# ------------------------------------------------------------------
# --- SECTION TO BUILD lidar_wirelesstest (ROS 2 VERSION) ---
# ------------------------------------------------------------------

# 2. Add the executable
add_executable(lidar_wirelesstest lidar_wirelesstest.cpp)

# 3. Add all necessary include directories
target_include_directories(lidar_wirelesstest PUBLIC
	# Unitree SDK paths
	${CMAKE_SOURCE_DIR}/include
	${CMAKE_SOURCE_DIR}/thirdparty/include/ddscxx
	${CMAKE_SOURCE_DIR}/thirdparty/include
	# ROS 2 package paths
	${rclcpp_INCLUDE_DIRS}
	${sensor_msgs_INCLUDE_DIRS}
)

# 4. Link all necessary libraries
target_link_libraries(lidar_wirelesstest PRIVATE
	# Unitree SDK libs
	${CMAKE_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_sdk2.a
	${CMAKE_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}/libddscxx.so
	${CMAKE_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}/libddsc.so
	Threads::Threads
	# ROS 2 package libs
	${rclcpp_LIBRARIES}
	${sensor_msgs_LIBRARIES}
)