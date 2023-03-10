# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "odom_pub: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iodom_pub:/home/huo/Downloads/ur3_ws/src/omnicar/odom_pub/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(odom_pub_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/omnicar/odom_pub/msg/motor_msg.msg" NAME_WE)
add_custom_target(_odom_pub_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "odom_pub" "/home/huo/Downloads/ur3_ws/src/omnicar/odom_pub/msg/motor_msg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(odom_pub
  "/home/huo/Downloads/ur3_ws/src/omnicar/odom_pub/msg/motor_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odom_pub
)

### Generating Services

### Generating Module File
_generate_module_cpp(odom_pub
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odom_pub
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(odom_pub_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(odom_pub_generate_messages odom_pub_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/omnicar/odom_pub/msg/motor_msg.msg" NAME_WE)
add_dependencies(odom_pub_generate_messages_cpp _odom_pub_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odom_pub_gencpp)
add_dependencies(odom_pub_gencpp odom_pub_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odom_pub_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(odom_pub
  "/home/huo/Downloads/ur3_ws/src/omnicar/odom_pub/msg/motor_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/odom_pub
)

### Generating Services

### Generating Module File
_generate_module_eus(odom_pub
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/odom_pub
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(odom_pub_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(odom_pub_generate_messages odom_pub_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/omnicar/odom_pub/msg/motor_msg.msg" NAME_WE)
add_dependencies(odom_pub_generate_messages_eus _odom_pub_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odom_pub_geneus)
add_dependencies(odom_pub_geneus odom_pub_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odom_pub_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(odom_pub
  "/home/huo/Downloads/ur3_ws/src/omnicar/odom_pub/msg/motor_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/odom_pub
)

### Generating Services

### Generating Module File
_generate_module_lisp(odom_pub
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/odom_pub
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(odom_pub_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(odom_pub_generate_messages odom_pub_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/omnicar/odom_pub/msg/motor_msg.msg" NAME_WE)
add_dependencies(odom_pub_generate_messages_lisp _odom_pub_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odom_pub_genlisp)
add_dependencies(odom_pub_genlisp odom_pub_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odom_pub_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(odom_pub
  "/home/huo/Downloads/ur3_ws/src/omnicar/odom_pub/msg/motor_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/odom_pub
)

### Generating Services

### Generating Module File
_generate_module_nodejs(odom_pub
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/odom_pub
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(odom_pub_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(odom_pub_generate_messages odom_pub_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/omnicar/odom_pub/msg/motor_msg.msg" NAME_WE)
add_dependencies(odom_pub_generate_messages_nodejs _odom_pub_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odom_pub_gennodejs)
add_dependencies(odom_pub_gennodejs odom_pub_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odom_pub_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(odom_pub
  "/home/huo/Downloads/ur3_ws/src/omnicar/odom_pub/msg/motor_msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odom_pub
)

### Generating Services

### Generating Module File
_generate_module_py(odom_pub
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odom_pub
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(odom_pub_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(odom_pub_generate_messages odom_pub_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/omnicar/odom_pub/msg/motor_msg.msg" NAME_WE)
add_dependencies(odom_pub_generate_messages_py _odom_pub_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(odom_pub_genpy)
add_dependencies(odom_pub_genpy odom_pub_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS odom_pub_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odom_pub)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/odom_pub
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(odom_pub_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(odom_pub_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/odom_pub)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/odom_pub
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(odom_pub_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(odom_pub_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/odom_pub)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/odom_pub
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(odom_pub_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(odom_pub_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/odom_pub)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/odom_pub
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(odom_pub_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(odom_pub_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odom_pub)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odom_pub\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/odom_pub
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(odom_pub_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(odom_pub_generate_messages_py nav_msgs_generate_messages_py)
endif()
