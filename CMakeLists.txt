cmake_minimum_required(VERSION 3.8)
project(mren203_new)

# 1. Find Dependencies
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# # 2. Add the Executable
# # This assumes your file is named 'arduino_parser.cpp' inside the 'src' folder
# add_executable(arduino_parser arduino_parser.cpp)

# # 3. Link Dependencies to the Executable
# ament_target_dependencies(arduino_parser
#   rclcpp
#   std_msgs
#   sensor_msgs
# )

# # Manually link Boost (since it's a non-Ament library)
# target_link_libraries(arduino_parser Boost::system)

# # 4. Install the Executable (so 'ros2 run' can find it)
# install(TARGETS
#   arduino_parser
#   DESTINATION lib/${PROJECT_NAME}
# )

install(PROGRAMS 
  scripts/motor_test_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 5. Export the Package
ament_package()
