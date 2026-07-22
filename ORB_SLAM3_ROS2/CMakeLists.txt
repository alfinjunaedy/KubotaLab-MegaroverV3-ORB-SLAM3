cmake_minimum_required(VERSION 3.5)
project(orbslam3)

set(CMAKE_CXX_STANDARD 14)

set(ORB_SLAM3_ROOT_DIR "/home/robot/ORB_SLAM3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${ORB_SLAM3_ROOT_DIR}
  ${ORB_SLAM3_ROOT_DIR}/include
  ${ORB_SLAM3_ROOT_DIR}/include/CameraModels
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2
  ${Pangolin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /opt/ros/humble/include
  /usr/include/eigen3
)

link_directories(
  ${ORB_SLAM3_ROOT_DIR}/lib
)

#
# MONO
#
add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)

ament_target_dependencies(mono
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(mono
  ${OpenCV_LIBS}
  ${ORB_SLAM3_ROOT_DIR}/lib/libORB_SLAM3.so
  pangolin
)

#
# RGBD
#
add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)

ament_target_dependencies(rgbd
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
)

target_link_libraries(rgbd
  ${OpenCV_LIBS}
  ${ORB_SLAM3_ROOT_DIR}/lib/libORB_SLAM3.so
  pangolin
)

#
# STEREO
#
add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)

ament_target_dependencies(stereo
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
)

target_link_libraries(stereo
  ${OpenCV_LIBS}
  ${ORB_SLAM3_ROOT_DIR}/lib/libORB_SLAM3.so
  pangolin
)

#
# STEREO-INERTIAL
#
add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)

ament_target_dependencies(stereo-inertial
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(stereo-inertial
  ${OpenCV_LIBS}
  ${ORB_SLAM3_ROOT_DIR}/lib/libORB_SLAM3.so
  pangolin
)

install(TARGETS
  mono
  rgbd
  stereo
  stereo-inertial
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
