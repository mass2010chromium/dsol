cmake_minimum_required(VERSION 3.8.2)
project(dsol)

find_package(ament_cmake REQUIRED)

# The c++ version is also being set in CMakeHelpers.cmake
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 --verbose")
list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(BUILD_ROS "Build ros" ON)
option(BUILD_PLAY "Build play" OFF)
option(BUILD_TESTING "Build tests" OFF)
option(BUILD_BENCHMARK "Build benchmarks" OFF)
option(BUILD_VECTORIZE "Build with -march=native" ON)

set(CC_TARGET_PREFIX sv)
include(CMakeHelpers)

add_library(sv_options INTERFACE)
include(CompilerWarnings)
enable_warnings(sv_options)
include(Sanitizers)
enable_sanitizers(sv_options)

# find_package(
#   catkin QUIET
#   COMPONENTS roscpp
#             tf2_ros
#             tf2_eigen
#             cv_bridge
#             image_transport
#             nav_msgs
#             sensor_msgs
#             visualization_msgs)

find_package(TBB REQUIRED)
find_package(fmt REQUIRED)
message(ERROR "THINGS: ${fmt_DIR}")
find_package(glog REQUIRED)
#find_package(absl REQUIRED)
find_package(Boost REQUIRED) # circular buffer

find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(xtensor) # reading npy

find_package(GTest REQUIRED)
#find_package(benchmark REQUIRED)

if(BUILD_ROS)
    find_package(rclcpp REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_ros REQUIRED)
endif()
# if(NOT catkin_FOUND)
#   set(BUILD_ROS OFF)
#   message(WARN "BUILD_ROS set to OFF because catkin not found.")
# endif()

# if(BUILD_ROS)
#   catkin_package()
# endif()

enable_testing()
add_subdirectory(sv)

install(TARGETS sv_dsol_node_odom
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
