execute_process(COMMAND "/home/huo/Downloads/ur3_ws/build/arm/handeye/easy_handeye/easy_handeye/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/huo/Downloads/ur3_ws/build/arm/handeye/easy_handeye/easy_handeye/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
