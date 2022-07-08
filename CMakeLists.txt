cmake_minimum_required(VERSION 3.0.2)
project(ntripbrowser_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ntripbrowser_ros
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)