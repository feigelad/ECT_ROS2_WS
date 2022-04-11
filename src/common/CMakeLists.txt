cmake_minimum_required(VERSION 3.5)
project(common)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

##==============Add Interfaces========================================
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
set(common_msg_files
  "msg/Quaternion.msg"
  "msg/BoolTimeStamped.msg"
  "msg/DoubleTimeStamped.msg"
  "msg/FloatTimeStamped.msg"
  "msg/Int8TimeStamped.msg"
  "msg/Int16TimeStamped.msg"
  "msg/Int32TimeStamped.msg"
  "msg/Int64TimeStamped.msg"
  "msg/LogData.msg"
  "msg/Point3d.msg"
  "msg/ThreadManager.msg"
  "msg/Uint8TimeStamped.msg"
  "msg/Uint16TimeStamped.msg"
  "msg/Uint32TimeStamped.msg"
  "msg/Uint64TimeStamped.msg"
  #etc
)

set(common_srv_files
  
  #etc
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${common_msg_files}
  ${common_srv_files}

  DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

##==============End Interfaces========================================


##==============Add Libraries===================================
# library common_byte
file(GLOB common_byte_srcs "byte/*.cc")
add_library(common_byte SHARED
  ${common_byte_srcs}
)
target_include_directories(common_byte PUBLIC
  byte
)

# library common_quaternion
file(GLOB common_quaternion_srcs "quaternion/*.cc")
add_library(common_quaternion SHARED
  ${common_quaternion_srcs}
)
target_include_directories(common_quaternion PUBLIC
  quaternion
)
# target_link_libraries(common_quaternion rclcpp tutorial_interfaces)

# library common_geometry
file(GLOB common_geometry_srcs "geometry/polyn/*.cc")
add_library(common_geometry SHARED
  ${common_geometry_srcs}
)
target_include_directories(common_geometry PUBLIC
  geometry/polyn
)

# library common_time
file(GLOB common_time_srcs "systime/ros_systime/*.cc")
add_library(common_time SHARED
  ${common_time_srcs}
)
target_include_directories(common_time PUBLIC
  systime
  systime/ros_systime
)
ament_target_dependencies(common_time 
  rclcpp
)

# library common_logger
file(GLOB common_logger_srcs "syslog/*.cc" "syslog/roslog/*.cc")
add_library(common_logger SHARED
  ${common_logger_srcs}
)
target_include_directories(common_logger PUBLIC
  syslog
  syslog/roslog
)
ament_target_dependencies(common_logger 
  rclcpp
)
rosidl_target_interfaces(common_logger
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# library common_communication
file(GLOB common_communication_srcs "communication/ipc_message/*.cc" "communication/ipc_manager/*.cc")
add_library(common_communication SHARED
  ${common_communication_srcs}
)
target_include_directories(common_communication PUBLIC
  communication/publisher
  communication/publisher/ros_publisher
  communication/subscriber
  communication/subscriber/ros_subscriber
  communication/ipc_message
  communication/ipc_manager
)
ament_target_dependencies(common_communication 
  rclcpp
)
target_link_libraries(common_communication  
  common_logger  
)
rosidl_target_interfaces(common_communication
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


# library common_transform_broadcaster
file(GLOB common_transform_broadcaster_srcs "transform/broadcaster/*.cc" "transform/broadcaster/ros_transform_broadcaster/*.cc")
add_library(common_transform_broadcaster SHARED
  ${common_transform_broadcaster_srcs}
)
target_include_directories(common_transform_broadcaster PUBLIC
  transform/broadcaster
  transform/broadcaster/ros_transform_broadcaster
)
ament_target_dependencies(common_transform_broadcaster 
  rclcpp 
  tf2
  tf2_ros
  tf2_geometry_msgs 
  
)
target_link_libraries(common_transform_broadcaster  
  common_logger  
)

# library common_transform_listener
file(GLOB common_transform_listener_srcs "transform/listener/*.cc" "transform/listener/ros_transform_listener/*.cc")
add_library(common_transform_listener SHARED
  ${common_transform_listener_srcs}
)
target_include_directories(common_transform_listener PUBLIC
  transform/listener
  transform/listener/ros_transform_listener
)
ament_target_dependencies(common_transform_listener 
  rclcpp  
  
)
target_link_libraries(common_transform_listener  
  common_logger  
)

# library common_schedule_task_thread
file(GLOB common_schedule_task_thread_srcs "schedule/task_thread/*.cc")
add_library(common_schedule_task_thread SHARED
  ${common_schedule_task_thread_srcs}
)
target_include_directories(common_schedule_task_thread PUBLIC
  schedule/task_thread
)
ament_target_dependencies(common_schedule_task_thread 
  rclcpp
  
)
target_link_libraries(common_schedule_task_thread  
  common_logger  
)
rosidl_target_interfaces(common_schedule_task_thread
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# library common_schedule_task_thread
file(GLOB common_schedule_thread_manager_srcs "schedule/thread_manager/*.cc")
add_library(common_schedule_thread_manager SHARED
  ${common_schedule_thread_manager_srcs}
)
target_include_directories(common_schedule_thread_manager PUBLIC
  schedule/thread_manager
)
ament_target_dependencies(common_schedule_thread_manager 
  rclcpp
  
)
target_link_libraries(common_schedule_thread_manager  
  common_schedule_task_thread  
)
rosidl_target_interfaces(common_schedule_thread_manager
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# library common_algorithm_pid
file(GLOB common_algorithm_pid_srcs "algorithm/pid/*.cc" "algorithm/pid/pos_pid/*.cc")
add_library(common_algorithm_pid SHARED
  ${common_algorithm_pid_srcs}
)
target_include_directories(common_algorithm_pid PUBLIC
  algorithm/pid
  algorithm/pid/pos_pid
)

##==============End Libraries===================================

#install(TARGETS common_node
#  DESTINATION lib/${PROJECT_NAME})


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
