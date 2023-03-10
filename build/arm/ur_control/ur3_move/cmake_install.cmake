# Install script for directory: /home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/huo/Downloads/ur3_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur3_move/srv" TYPE FILE FILES
    "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/AddTwoInts.srv"
    "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/getObjectPosition.srv"
    "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/mulObjectsPosition.srv"
    "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/varObjectsPosition.srv"
    "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/movingObjectPosition.srv"
    "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/Stop.srv"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur3_move/cmake" TYPE FILE FILES "/home/huo/Downloads/ur3_ws/build/arm/ur_control/ur3_move/catkin_generated/installspace/ur3_move-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/huo/Downloads/ur3_ws/devel/include/ur3_move")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/huo/Downloads/ur3_ws/devel/share/roseus/ros/ur3_move")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/huo/Downloads/ur3_ws/devel/share/common-lisp/ros/ur3_move")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/huo/Downloads/ur3_ws/devel/share/gennodejs/ros/ur3_move")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/huo/Downloads/ur3_ws/devel/lib/python2.7/dist-packages/ur3_move")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/huo/Downloads/ur3_ws/devel/lib/python2.7/dist-packages/ur3_move")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/huo/Downloads/ur3_ws/build/arm/ur_control/ur3_move/catkin_generated/installspace/ur3_move.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur3_move/cmake" TYPE FILE FILES "/home/huo/Downloads/ur3_ws/build/arm/ur_control/ur3_move/catkin_generated/installspace/ur3_move-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur3_move/cmake" TYPE FILE FILES
    "/home/huo/Downloads/ur3_ws/build/arm/ur_control/ur3_move/catkin_generated/installspace/ur3_moveConfig.cmake"
    "/home/huo/Downloads/ur3_ws/build/arm/ur_control/ur3_move/catkin_generated/installspace/ur3_moveConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ur3_move" TYPE FILE FILES "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/package.xml")
endif()

