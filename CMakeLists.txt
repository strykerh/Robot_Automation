cmake_minimum_required(VERSION 2.8.3)
project(robot_sd)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS  
  #dynamic_reconfigure
  roscpp
  rospy
  #sensor_msgs
  std_msgs
  message_generation
  #cv_bridge
  #tf2_ros
 # pcl_conversions
 # pcl_ros
 # nearness_control_msgs
)

## Declare ROS messages and services (do we need?)
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services  (do we need?)
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker robot_sd_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener robot_sd_generate_messages_cpp)

# now we need to add a subscriber to the lidar scans, client.cpp seems to be the subscriber within the RPlidar folders. How can we pull that node into here without getting errors pertaining to file location?

add_executable(lidarlistener src/lidarlistener.cpp)
target_link_libraries(lidarlistener ${catkin_LIBRARIES})
add_dependencies(lidarlistener robot_sd_generate_messages_cpp)

