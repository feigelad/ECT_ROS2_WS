cmake_minimum_required(VERSION 3.5)
project(message)

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


##==============Add Interfaces========================================
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
set(message_msg_files
  "msg/AnticrshStat.msg"
  "msg/BatDiag.msg"
  "msg/BatStat.msg"
  "msg/BodyCmd.msg"
  "msg/BodyDiag.msg"
  "msg/BodyStat.msg"
  "msg/BrakeCmd.msg"
  "msg/BrakeDiag.msg"
  "msg/BrakeStat.msg"
  "msg/CloudBodyCmd.msg"
  "msg/CloudDriveCmd.msg"
  "msg/EcarChassis.msg"
  "msg/EcarChassisCmd.msg"
  "msg/EcarChassisStat.msg"
  "msg/ImuGyro1Stat.msg"
  "msg/ImuGyro2Stat.msg"
  "msg/ImuGyro3Stat.msg"
  "msg/ImuInputCmd.msg"
  "msg/ImuRtkAttCovStat.msg"
  "msg/ImuRtkAttitudeStat.msg"
  "msg/ImuRtkPos1Stat.msg"
  "msg/ImuRtkPos2Stat.msg"
  "msg/ImuRtkPosCovStat.msg"
  "msg/ImuRtkSpeedCovStat.msg"
  "msg/ImuRtkSpeedStat.msg"
  "msg/ImuRtkStatus.msg"
  "msg/ImuStat.msg"
  "msg/ImuUtcTimeStat.msg"
  "msg/ImuWeekTimeStat.msg"
  "msg/Ins570dAccStat.msg"
  "msg/Ins570dAttitudeStat.msg"
  "msg/Ins570dDatainfoStat.msg"
  "msg/Ins570dGyroStat.msg"
  "msg/Ins570dPos1Stat.msg"
  "msg/Ins570dPos2Stat.msg"
  "msg/Ins570dSpeedStat.msg"
  "msg/Ins570dStdcovStat.msg"
  "msg/ParkCmd.msg"
  "msg/ParkDiag.msg"
  "msg/ParkStat.msg"
  "msg/PowerCmd.msg"
  "msg/PowerStat.msg"
  "msg/RemoterStat1.msg"
  "msg/RemoterStat2.msg"
  "msg/RemoterStat3.msg"
  "msg/SteerCmd.msg"
  "msg/SteerDiag.msg"
  "msg/SteerStat.msg"
  "msg/ThrotCmd.msg"
  "msg/ThrotDiag.msg"
  "msg/ThrotStat.msg"
  "msg/TripClearCmd.msg"
  "msg/TripStat.msg"
  "msg/VehDynaStat.msg"
  "msg/VehInfo.msg"
  "msg/VehStat.msg"
  "msg/VehVinWrite.msg"
  "msg/WheelSpdStat.msg"
  #etc
)

set(message_srv_files
  
  #etc
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${message_msg_files}
  ${message_srv_files}
  DEPENDENCIES builtin_interfaces std_msgs common
)
ament_export_dependencies(rosidl_default_runtime)

##==============End Interfaces========================================

link_directories(
  /home/benchu/workspace/nirvana_ros2/build/common
  /home/benchu/workspace/nirvana_ros2/build/message
  canbus/third_party/zlg_can
)

##==============Add Libraries===================================
# library message_candev
file(GLOB message_candev_srcs 
  "canbus/can_dev/*.cc" 
  "canbus/can_dev/zlg_can/*.cc"  
)
add_library(message_candev SHARED
  ${message_candev_srcs}
)
target_include_directories(message_candev PUBLIC
  canbus/can_dev
  canbus/can_dev/zlg_can
)
ament_target_dependencies(message_candev 
  # common_logger
  rclcpp
)
target_link_libraries(message_candev  
  libusbcan.so
)
rosidl_target_interfaces(message_candev
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# library message_cancomm
file(GLOB message_cancomm_srcs
  "canbus/can_comm/*.cc"
)
add_library(message_cancomm SHARED
  ${message_cancomm_srcs}
)
target_include_directories(message_cancomm PUBLIC
  canbus/can_comm
)
# library message_can_protocol
file(GLOB message_can_protocol_srcs 
  "canbus/protocol/*.cc" 
  "canbus/protocol/ecar_standard/*.cc" 
  "canbus/protocol/imu_m2/*.cc" 
  "canbus/protocol/imu_ins570d/*.cc"
)
add_library(message_can_protocol SHARED
  ${message_can_protocol_srcs}  
)
target_include_directories(message_can_protocol PUBLIC
  canbus/protocol
  canbus/protocol/ecar_standard
  canbus/protocol/imu_m2
  canbus/protocol/imu_ins570d
)
ament_target_dependencies(message_can_protocol 
  rclcpp
  common
)
target_link_libraries(message_can_protocol  
  message_cancomm
)
rosidl_target_interfaces(message_can_protocol
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# library message_can_message_manager
file(GLOB message_can_message_manager_srcs "canbus/message_manager/*.cc")
add_library(message_can_message_manager SHARED
  ${message_can_message_manager_srcs}
)
ament_target_dependencies(message_can_message_manager 
  rclcpp
  
)
target_link_libraries(message_can_message_manager  
  message_candev
  message_can_protocol
)
rosidl_target_interfaces(message_can_message_manager
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

##==============End Libraries===================================


##==============Add Nodes===================================
#  executable canbus_node
file(GLOB canbus_node_srcs
  # "src/canbus_node.h"
  "src/canbus_node.cpp"
)
add_executable(canbus_node ${canbus_node_srcs})
target_include_directories(canbus_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(canbus_node
  std_msgs
  rclcpp
  common
)
target_link_libraries(canbus_node
  # std_msgs
  # rclcpp
  # common
  common_communication
  common_schedule_thread_manager
  common_schedule_task_thread
  common_logger
  common_time
  message_cancomm
  message_candev
  message_can_protocol
  message_can_message_manager
)
rosidl_target_interfaces(canbus_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS canbus_node
  DESTINATION lib/${PROJECT_NAME})

##==============End Nodes===================================

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
