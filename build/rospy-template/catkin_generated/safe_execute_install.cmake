execute_process(COMMAND "/home/bargos/offboard/build/rospy-template/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/bargos/offboard/build/rospy-template/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
