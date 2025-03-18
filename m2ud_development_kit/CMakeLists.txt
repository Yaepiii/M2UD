cmake_minimum_required(VERSION 3.0.2)
project(m2ud_development_kit)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  pcl_conversions
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  message_filters
  cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rosbag roscpp rospy sensor_msgs std_msgs geometry_msgs nav_msgs message_filters cv_bridge
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
)

add_executable(groundtruth_create src/groundtruth_create.cpp)
target_link_libraries(groundtruth_create ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(time_align src/time_align.cpp)
target_link_libraries(time_align ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(bag2file src/bag2file.cpp)
target_link_libraries(bag2file ${catkin_LIBRARIES} ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})
