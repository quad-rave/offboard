cmake_minimum_required(VERSION 2.8.3)
project(swarm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  catkin_simple REQUIRED
  message_generation
)

catkin_python_setup()
catkin_simple()


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

cs_export()
