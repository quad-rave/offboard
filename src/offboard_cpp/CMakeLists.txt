cmake_minimum_required(VERSION 3.0.2)
project(offboard_cpp)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)
include_directories(${catkin_INCLUDE_DIRS})

catkin_package()

add_executable(state_subscribe src/sqr_state_sub.cpp)
target_link_libraries(state_subscribe ${catkin_LIBRARIES})

add_executable(offboard_sqr src/offb_node_square.cpp)
target_link_libraries(offboard_sqr ${catkin_LIBRARIES})



