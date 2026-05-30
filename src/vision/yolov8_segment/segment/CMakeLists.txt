cmake_minimum_required(VERSION 3.10)

project(yolov8_segment)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# CUDA and TensorRT
find_package(CUDA REQUIRED)
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    include_directories(/usr/include/aarch64-linux-gnu)
    link_directories(/usr/lib/aarch64-linux-gnu)
else()
    include_directories(/usr/include/x86_64-linux-gnu)
    link_directories(/usr/lib/x86_64-linux-gnu)
endif()

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add libraries and executable
file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.cu)
cuda_add_library(yolov8_segment SHARED ${SRCS})
target_link_libraries(yolov8_segment cudart nvinfer nvonnxparser ${OpenCV_LIBS})

add_executable(main ${PROJECT_SOURCE_DIR}/main.cpp)
ament_target_dependencies(main rclcpp yolo_msgs OpenCV)
target_link_libraries(main yolov8_segment)

# Install targets
install(TARGETS main yolov8_segment
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}")

# Export package
ament_package()

