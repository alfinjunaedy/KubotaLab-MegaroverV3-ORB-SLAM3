cmake_minimum_required(VERSION 3.8)
project(map_builder)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

add_executable(
    map_builder 
    src/map_builder.cpp
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

target_include_directories(map_builder PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

ament_target_dependencies(
    map_builder
    rclcpp
    geometry_msgs
    sensor_msgs
    visualization_msgs
    pcl_conversions
    PCL
)

target_link_libraries(map_builder
    ${Pangolin_LIBRARIES}
    Eigen3::Eigen
)

install(TARGETS map_builder
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()