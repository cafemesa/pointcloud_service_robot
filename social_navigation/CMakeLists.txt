cmake_minimum_required(VERSION 3.0.2)
project(social_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_msgs
  pcl_conversions
  pcl_ros
  geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


#add_executable(gazebo_variables src/gazebo.cpp)
#target_link_libraries(gazebo_variables ${catkin_LIBRARIES})

#add_executable(proxemics src/proxemics.cpp)
#target_link_libraries(proxemics ${catkin_LIBRARIES})