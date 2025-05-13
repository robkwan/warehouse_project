cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

include_directories(include)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#add_service_files(
#  FILES
#  GoToLoading.srv
#)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
)

#add_executable(pre_approach src/pre_approach.cpp)
#ament_target_dependencies(pre_approach rclcpp sensor_msgs geometry_msgs nav_msgs tf2)

add_executable(pre_approach_v2 src/pre_approach_v2.cpp)
ament_target_dependencies(pre_approach_v2 rclcpp sensor_msgs geometry_msgs nav_msgs tf2)

add_executable(approach_service_server src/approach_service_server.cpp)
ament_target_dependencies(approach_service_server rclcpp sensor_msgs geometry_msgs tf2_ros tf2_geometry_msgs std_msgs)

#add_executable(test_tf_cart src/test_tf_cart.cpp)
#ament_target_dependencies(test_tf_cart rclcpp tf2_ros)

#add_executable(shelf_tf_publisher src/shelf_tf_publisher.cpp)
#ament_target_dependencies(shelf_tf_publisher rclcpp sensor_msgs geometry_msgs tf2_ros)

# Link generated interface to this node
rosidl_target_interfaces(pre_approach_v2
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

rosidl_target_interfaces(approach_service_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS 
#		pre_approach
		pre_approach_v2
  		approach_service_server
#		test_tf_cart
#		shelf_tf_publisher
		DESTINATION 
		lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
#  rviz
  srv
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

