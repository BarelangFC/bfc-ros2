cmake_minimum_required(VERSION 3.10)

project(yolov8_bytrseg)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories("/usr/include/eigen3")

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(detect_msgs REQUIRED)
find_package(OpenCV REQUIRED) 
find_package(ZLIB REQUIRED)

# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# ============= cuda ============
find_package(CUDA REQUIRED)
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)

# ============= tensorrt ============
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("Embed_platform on")
  include_directories(/usr/include/aarch64-linux-gnu)
  link_directories(/usr/lib/aarch64-linux-gnu)
else()
  message("Embed_platform off")
  include_directories(/usr/include/x86_64-linux-gnu)
  link_directories(/usr/lib/x86_64-linux-gnu)
endif()

# ============ opencv ============
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

# =========== bytetrack lib ===========
include_directories(${PROJECT_SOURCE_DIR}/bytetrack/include)
file(GLOB_RECURSE SRCS01 ${PROJECT_SOURCE_DIR}/bytetrack/src/*.cpp)
add_library(bytetrack SHARED ${SRCS01})
target_link_libraries(bytetrack cudart nvinfer ${OpenCV_LIBS})

# ============= yolov8 lib =============
include_directories(${PROJECT_SOURCE_DIR}/detect/include)
file(GLOB_RECURSE SRCS02 ${PROJECT_SOURCE_DIR}/detect/src/*.cpp ${PROJECT_SOURCE_DIR}/detect/src/*.cu)
cuda_add_library(yolov8_bytrseg SHARED ${SRCS02})
target_link_libraries(yolov8_bytrseg nvinfer cudart nvonnxparser ZLIB::ZLIB ${OpenCV_LIBS})

# ======== main execute file ========
add_executable(main ${PROJECT_SOURCE_DIR}/main.cpp ${PROJECT_SOURCE_DIR}/detect/src/cnpy.cpp)
target_include_directories(main PRIVATE ${PROJECT_SOURCE_DIR}/detect/include)
ament_target_dependencies(main rclcpp detect_msgs OpenCV)
target_link_libraries(main bytetrack yolov8_bytrseg ZLIB::ZLIB)

# Install targets
install(TARGETS main bytetrack yolov8_bytrseg
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME})

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}")

# Export package
ament_package()
