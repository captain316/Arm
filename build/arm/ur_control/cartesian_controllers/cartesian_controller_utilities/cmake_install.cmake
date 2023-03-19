# Install script for directory: /home/huo/Downloads/ur3_ws/src/arm/ur_control/cartesian_controllers/cartesian_controller_utilities

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
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/huo/Downloads/ur3_ws/build/arm/ur_control/cartesian_controllers/cartesian_controller_utilities/catkin_generated/installspace/cartesian_controller_utilities.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartesian_controller_utilities/cmake" TYPE FILE FILES
    "/home/huo/Downloads/ur3_ws/build/arm/ur_control/cartesian_controllers/cartesian_controller_utilities/catkin_generated/installspace/cartesian_controller_utilitiesConfig.cmake"
    "/home/huo/Downloads/ur3_ws/build/arm/ur_control/cartesian_controllers/cartesian_controller_utilities/catkin_generated/installspace/cartesian_controller_utilitiesConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartesian_controller_utilities" TYPE FILE FILES "/home/huo/Downloads/ur3_ws/src/arm/ur_control/cartesian_controllers/cartesian_controller_utilities/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cartesian_controller_utilities" TYPE PROGRAM FILES
    "/home/huo/Downloads/ur3_ws/src/arm/ur_control/cartesian_controllers/cartesian_controller_utilities/scripts/buttons.py"
    "/home/huo/Downloads/ur3_ws/src/arm/ur_control/cartesian_controllers/cartesian_controller_utilities/scripts/converter.py"
    "/home/huo/Downloads/ur3_ws/src/arm/ur_control/cartesian_controllers/cartesian_controller_utilities/scripts/pose.py"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartesian_controller_utilities" TYPE DIRECTORY FILES
    "/home/huo/Downloads/ur3_ws/src/arm/ur_control/cartesian_controllers/cartesian_controller_utilities/etc"
    "/home/huo/Downloads/ur3_ws/src/arm/ur_control/cartesian_controllers/cartesian_controller_utilities/launch"
    )
endif()

