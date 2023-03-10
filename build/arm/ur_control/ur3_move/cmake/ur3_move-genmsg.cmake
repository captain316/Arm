# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ur3_move: 0 messages, 6 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ur3_move_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/AddTwoInts.srv" NAME_WE)
add_custom_target(_ur3_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ur3_move" "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/AddTwoInts.srv" ""
)

get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/mulObjectsPosition.srv" NAME_WE)
add_custom_target(_ur3_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ur3_move" "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/mulObjectsPosition.srv" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/varObjectsPosition.srv" NAME_WE)
add_custom_target(_ur3_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ur3_move" "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/varObjectsPosition.srv" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/getObjectPosition.srv" NAME_WE)
add_custom_target(_ur3_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ur3_move" "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/getObjectPosition.srv" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/Stop.srv" NAME_WE)
add_custom_target(_ur3_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ur3_move" "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/Stop.srv" ""
)

get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/movingObjectPosition.srv" NAME_WE)
add_custom_target(_ur3_move_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ur3_move" "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/movingObjectPosition.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur3_move
)
_generate_srv_cpp(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/mulObjectsPosition.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur3_move
)
_generate_srv_cpp(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/varObjectsPosition.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur3_move
)
_generate_srv_cpp(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/getObjectPosition.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur3_move
)
_generate_srv_cpp(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/Stop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur3_move
)
_generate_srv_cpp(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/movingObjectPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur3_move
)

### Generating Module File
_generate_module_cpp(ur3_move
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur3_move
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ur3_move_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ur3_move_generate_messages ur3_move_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_cpp _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/mulObjectsPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_cpp _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/varObjectsPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_cpp _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/getObjectPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_cpp _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/Stop.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_cpp _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/movingObjectPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_cpp _ur3_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur3_move_gencpp)
add_dependencies(ur3_move_gencpp ur3_move_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur3_move_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur3_move
)
_generate_srv_eus(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/mulObjectsPosition.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur3_move
)
_generate_srv_eus(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/varObjectsPosition.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur3_move
)
_generate_srv_eus(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/getObjectPosition.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur3_move
)
_generate_srv_eus(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/Stop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur3_move
)
_generate_srv_eus(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/movingObjectPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur3_move
)

### Generating Module File
_generate_module_eus(ur3_move
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur3_move
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ur3_move_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ur3_move_generate_messages ur3_move_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_eus _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/mulObjectsPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_eus _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/varObjectsPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_eus _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/getObjectPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_eus _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/Stop.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_eus _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/movingObjectPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_eus _ur3_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur3_move_geneus)
add_dependencies(ur3_move_geneus ur3_move_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur3_move_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur3_move
)
_generate_srv_lisp(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/mulObjectsPosition.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur3_move
)
_generate_srv_lisp(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/varObjectsPosition.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur3_move
)
_generate_srv_lisp(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/getObjectPosition.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur3_move
)
_generate_srv_lisp(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/Stop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur3_move
)
_generate_srv_lisp(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/movingObjectPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur3_move
)

### Generating Module File
_generate_module_lisp(ur3_move
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur3_move
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ur3_move_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ur3_move_generate_messages ur3_move_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_lisp _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/mulObjectsPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_lisp _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/varObjectsPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_lisp _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/getObjectPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_lisp _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/Stop.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_lisp _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/movingObjectPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_lisp _ur3_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur3_move_genlisp)
add_dependencies(ur3_move_genlisp ur3_move_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur3_move_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur3_move
)
_generate_srv_nodejs(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/mulObjectsPosition.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur3_move
)
_generate_srv_nodejs(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/varObjectsPosition.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur3_move
)
_generate_srv_nodejs(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/getObjectPosition.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur3_move
)
_generate_srv_nodejs(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/Stop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur3_move
)
_generate_srv_nodejs(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/movingObjectPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur3_move
)

### Generating Module File
_generate_module_nodejs(ur3_move
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur3_move
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ur3_move_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ur3_move_generate_messages ur3_move_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_nodejs _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/mulObjectsPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_nodejs _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/varObjectsPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_nodejs _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/getObjectPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_nodejs _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/Stop.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_nodejs _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/movingObjectPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_nodejs _ur3_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur3_move_gennodejs)
add_dependencies(ur3_move_gennodejs ur3_move_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur3_move_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_move
)
_generate_srv_py(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/mulObjectsPosition.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_move
)
_generate_srv_py(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/varObjectsPosition.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_move
)
_generate_srv_py(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/getObjectPosition.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_move
)
_generate_srv_py(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/Stop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_move
)
_generate_srv_py(ur3_move
  "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/movingObjectPosition.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_move
)

### Generating Module File
_generate_module_py(ur3_move
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_move
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ur3_move_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ur3_move_generate_messages ur3_move_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_py _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/mulObjectsPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_py _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/varObjectsPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_py _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/getObjectPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_py _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/Stop.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_py _ur3_move_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/arm/ur_control/ur3_move/srv/movingObjectPosition.srv" NAME_WE)
add_dependencies(ur3_move_generate_messages_py _ur3_move_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ur3_move_genpy)
add_dependencies(ur3_move_genpy ur3_move_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ur3_move_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur3_move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ur3_move
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET ur3_move_generate_messages_cpp)
  add_dependencies(ur3_move_generate_messages_cpp ur3_move_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(ur3_move_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur3_move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ur3_move
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET ur3_move_generate_messages_eus)
  add_dependencies(ur3_move_generate_messages_eus ur3_move_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(ur3_move_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur3_move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ur3_move
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET ur3_move_generate_messages_lisp)
  add_dependencies(ur3_move_generate_messages_lisp ur3_move_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(ur3_move_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur3_move)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ur3_move
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET ur3_move_generate_messages_nodejs)
  add_dependencies(ur3_move_generate_messages_nodejs ur3_move_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(ur3_move_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_move)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_move\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ur3_move
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET ur3_move_generate_messages_py)
  add_dependencies(ur3_move_generate_messages_py ur3_move_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(ur3_move_generate_messages_py geometry_msgs_generate_messages_py)
endif()
