cmake_minimum_required(VERSION 3.0.2)
project(semantic_mapping)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  laser_geometry
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(semantic_people_lidar src/lidar_people.cpp)
target_link_libraries(semantic_people_lidar ${catkin_LIBRARIES})

add_executable(pc2csv src/pc2csv.cpp)
target_link_libraries(pc2csv ${catkin_LIBRARIES})

add_executable(lidar_people_sensor src/lidar_people_sensor.cpp)
target_link_libraries(lidar_people_sensor ${catkin_LIBRARIES})