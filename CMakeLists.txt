cmake_minimum_required(VERSION 3.8)
project(mren203_new)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Arduino parser node
add_executable(arduino_parser arduino_parser.cpp)
ament_target_dependencies(arduino_parser
  rclcpp
  std_msgs
  sensor_msgs
)
target_link_libraries(arduino_parser Boost::system)

# Odom publisher node
add_executable(odom_publisher scripts/odom_publisher.cpp)
ament_target_dependencies(odom_publisher
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2
)

install(TARGETS
  arduino_parser
  odom_publisher
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
