cmake_minimum_required(VERSION 3.10)
project(gmsl_camera_ros)

set(CMAKE_CXX_STANDARD 14)

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -g -O0 -ggdb")

find_package(catkin 
  REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  rospy  
  roscpp
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    /opt/miivii/include
    )

link_directories(/opt/miivii/lib
        /usr/lib/aarch64-linux-gnu/
        /usr/lib/aarch64-linux-gnu/tegra
        )

add_executable(
        gmsl_camera_ros
        src/gmsl_camera_ros.cpp
        )
target_link_libraries(
        gmsl_camera_ros
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        mvgmslcamera
        )
