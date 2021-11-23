cmake_minimum_required(VERSION 3.0.2)
project(traversable_mapping)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(src)

add_executable(velodyne_filter src/velodyne_filter.cpp)
target_link_libraries(velodyne_filter ${catkin_LIBRARIES})

add_executable(astra_filter src/astra_filter.cpp)
target_link_libraries(astra_filter ${catkin_LIBRARIES})

add_executable(kinect_filter src/kinect_filter.cpp)
target_link_libraries(kinect_filter ${catkin_LIBRARIES})


