cmake_minimum_required(VERSION 3.5)
project(location)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common REQUIRED)
find_package(message REQUIRED)

##==============Add Interfaces========================================
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(location_msg_files
  "msg/Pose.msg"
  #etc
)

set(location_srv_files
  
  #etc
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${location_msg_files}
  ${location_srv_files}
  DEPENDENCIES builtin_interfaces std_msgs common
)
ament_export_dependencies(rosidl_default_runtime)

##==============End Interfaces========================================

link_directories(
  /workspaces/ECT_ROS2_WS/build/common
  # /home/benchu/workspace/nirvana_ros2/build/vehicle
)

##==============Add Libraries===================================
# library location_rtk
file(GLOB location_rtk_srcs 
  "rtk/*.cc" 
  "rtk/ins570d/*.cc"
  "rtk/m2/*.cc"  
)
# add_library(location_rtk SHARED
#   ${location_rtk_srcs}
# )
# target_include_directories(location_rtk PUBLIC
#   rtk
#   rtk/ins570d
#   rtk/m2
# )
# ament_target_dependencies(location_rtk 
#   # common_logger
#   geometry_msgs
#   rclcpp
#   message
# )
# target_link_libraries(location_rtk  
#   common_quaternion
#   proj
# )
# rosidl_target_interfaces(location_rtk
# ${PROJECT_NAME} "rosidl_typesupport_cpp")

##==============End Libraries===================================

# add_executable(location_node src/location_node.cpp)
# target_include_directories(location_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   src
# )
# ament_target_dependencies(location_node
#   "std_msgs"
#   "geometry_msgs"
#   "rclcpp"
# )
# target_link_libraries(location_node
#   common_communication
#   common_schedule_thread_manager
#   common_schedule_task_thread
#   common_logger
#   common_time
#   common_transform_broadcaster
#   common_transform_listener
#   # common_algorithm_pid
#   location_rtk
# )
# rosidl_target_interfaces(location_node
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

# install(TARGETS location_node
#   DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
