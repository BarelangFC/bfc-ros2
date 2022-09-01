cmake_minimum_required(VERSION 3.5)
project(coordination)

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
find_package(std_msgs REQUIRED)
find_package(bfc_msgs REQUIRED)

add_executable(coordination src/coordination.cpp)
ament_target_dependencies(coordination rclcpp std_msgs bfc_msgs)

install(TARGETS
  coordination
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
