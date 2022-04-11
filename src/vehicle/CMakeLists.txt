cmake_minimum_required(VERSION 3.5)
project(vehicle)

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
set(vehicle_msg_files
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory.msg"
  "msg/VehicleDebug.msg"
  #etc
)

set(vehicle_srv_files
  
  #etc
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${vehicle_msg_files}
  ${vehicle_srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

##==============End Interfaces========================================

link_directories(
  /workspaces/ECT_ROS2_WS/build/common
  /workspaces/ECT_ROS2_WS/build/vehicle
)

##==============Add Libraries===================================
# library vehicle_components
file(GLOB vehicle_components_srcs 
  "components/body/*.cc" 
  "components/body/standard_body/*.cc"  
  "components/brake/*.cc" 
  "components/brake/disc_brake/*.cc"  
  "components/gearbox/*.cc" 
  "components/gearbox/gearbox1/*.cc" 
  "components/motor/*.cc" 
  "components/motor/DBStar_2p2kw/*.cc" 
  "components/park/*.cc" 
  "components/park/magnetic/*.cc" 
  "components/reducer/*.cc" 
  "components/reducer/reducer1/*.cc" 
  "components/steer/*.cc" 
  "components/steer/ceps/*.cc" 
  "components/wheel/*.cc" 
  "components/wheel/wheel1/*.cc" 
)
add_library(vehicle_components SHARED
  ${vehicle_components_srcs}
)
target_include_directories(vehicle_components PUBLIC
  vehicle/components/body
  vehicle/components/body/standard_body  
  vehicle/components/brake 
  vehicle/components/brake/disc_brake  
  vehicle/components/gearbox 
  vehicle/components/gearbox/gearbox1 
  vehicle/components/motor 
  vehicle/components/motor/DBStar_2p2kw 
  vehicle/components/park 
  vehicle/components/park/magnetic 
  vehicle/components/reducer 
  vehicle/components/reducer/reducer1 
  vehicle/components/steer 
  vehicle/components/steer/ceps 
  vehicle/components/wheel 
  vehicle/components/wheel/wheel1 
)
ament_target_dependencies(vehicle_components 
  # common_logger
  rclcpp
  message
)
# target_link_libraries(vehicle_components  
#   # libusbcan.so
# )
rosidl_target_interfaces(vehicle_components
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# library vehicle_dynamic
file(GLOB vehicle_dynamic_srcs 
  "dynamic/*.cc" 
  "dynamic/monorail_model/*.cc"  
)
add_library(vehicle_dynamic SHARED
${vehicle_dynamic_srcs}
)
target_include_directories(vehicle_dynamic PUBLIC
  dynamic 
  dynamic/monorail_model
)
ament_target_dependencies(vehicle_dynamic 
  # common_logger
  rclcpp
  message
)
target_link_libraries(vehicle_dynamic  
  vehicle_components
)
rosidl_target_interfaces(vehicle_dynamic
${PROJECT_NAME} "rosidl_typesupport_cpp")

# library vehicle_control
file(GLOB vehicle_control_srcs 
  "control/driver/*.cc" 
  "control/driver/ecar_driver/*.cc" 
  "control/remoter/*.cc" 
  "control/remoter/wfly_et06/*.cc" 
  "control/controller/*.cc" 
  "control/controller/dynamic_controller/*.cc" 
)
add_library(vehicle_control SHARED
${vehicle_control_srcs}
)
target_include_directories(vehicle_control PUBLIC
  control/driver
  control/driver/ecar_driver
  control/remoter
  control/remoter/wfly_et06
  control/controller
  control/controller/dynamic_controller
)
ament_target_dependencies(vehicle_control 
  # common_logger
  rclcpp
  message
)
target_link_libraries(vehicle_control  
  vehicle_components
  vehicle_dynamic
)
rosidl_target_interfaces(vehicle_control
${PROJECT_NAME} "rosidl_typesupport_cpp")

# library vehicle_status
file(GLOB vehicle_status_srcs 
  "status/motion_status/*.cc" 
  "status/motion_status/ecar_motion/*.cc" 
)
add_library(vehicle_status SHARED
${vehicle_status_srcs}
)
target_include_directories(vehicle_status PUBLIC
  status/motion_status
  status/motion_status/ecar_motion
)
ament_target_dependencies(vehicle_status 
  # common_logger
  rclcpp
  message
)
target_link_libraries(vehicle_status  
  vehicle_components
  # vehicle_dynamic
)
rosidl_target_interfaces(vehicle_status
${PROJECT_NAME} "rosidl_typesupport_cpp")

# library vehicle_manager
file(GLOB vehicle_manager_srcs 
  "vehicle_manager/*.cc" 
)
add_library(vehicle_manager SHARED
${vehicle_manager_srcs}
)
target_include_directories(vehicle_manager PUBLIC
  vehicle_manager
)
ament_target_dependencies(vehicle_manager 
  # common_logger
  rclcpp
  # message
)
target_link_libraries(vehicle_manager  
  vehicle_components
  vehicle_dynamic
  vehicle_control
)
rosidl_target_interfaces(vehicle_manager
${PROJECT_NAME} "rosidl_typesupport_cpp")

##==============End Libraries===================================

add_executable(vehicle_node src/vehicle_node.cpp)
target_include_directories(vehicle_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  src
)
ament_target_dependencies(vehicle_node
  "std_msgs"
  "rclcpp"
)
target_link_libraries(vehicle_node
  common_communication
  common_schedule_thread_manager
  common_schedule_task_thread
  common_logger
  common_time
  common_algorithm_pid
  vehicle_components
  vehicle_dynamic
  vehicle_control
  vehicle_manager
)
rosidl_target_interfaces(vehicle_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS vehicle_node
  DESTINATION lib/${PROJECT_NAME})

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
