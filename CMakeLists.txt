cmake_minimum_required(VERSION 3.10)
project(Global-LVBA LANGUAGES CXX)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native -mtune=native -funroll-loops")

# ROOT_DIR is used by main.cpp to default the config path. Trailing slash
# preserved to match the original ros build.
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

# Pure-cmake build: no catkin / roscpp. The few ROS APIs the code used
# (NodeHandle::param, Publisher::publish, ros::Time, sensor_msgs::PointCloud2)
# are stubbed in include/ros_shim.h, which is a header-only drop-in.
find_package(OpenMP REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)

# Legacy Sophus (the non-templated `Sophus::SE3` API LVBA uses) is built from
# source in the Dockerfile and installed to /usr/local. That old version
# doesn't ship a SophusConfig.cmake, so find_package(Sophus) fails. We point
# at the .so directly (linker resolves it via /usr/local/lib which is in the
# default search path) and rely on /usr/local/include being implicit for the
# headers. Switch to find_package if/when we ever move to header-only Sophus.
set(Sophus_LIBRARIES libSophus.so)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SiftGPU/src/SiftGPU
  /usr/local/include
)

set(SIFTGPU_LIBS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/SiftGPU/build/src/SiftGPU/libsiftgpu.a
)

add_library(lvba_system src/lvba_system.cpp)
add_library(lvba_io src/dataset_io.cpp)

target_link_libraries(lvba_system
  PUBLIC
    Ceres::ceres
    Eigen3::Eigen
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Sophus_LIBRARIES}
    ${SIFTGPU_LIBS}
    yaml-cpp
    sqlite3
    GLEW::GLEW
    GLUT::GLUT
    OpenGL::GL
)

target_link_libraries(lvba_io
  PUBLIC
    ${Sophus_LIBRARIES}
    yaml-cpp
)

add_executable(lvba_run src/main.cpp)
target_link_libraries(lvba_run
  PRIVATE
    lvba_system
    lvba_io
)
