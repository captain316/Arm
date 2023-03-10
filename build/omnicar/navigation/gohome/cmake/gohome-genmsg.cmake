# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "gohome: 0 messages, 1 services")

set(MSG_I_FLAGS "")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(gohome_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/omnicar/navigation/gohome/srv/carArm.srv" NAME_WE)
add_custom_target(_gohome_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gohome" "/home/huo/Downloads/ur3_ws/src/omnicar/navigation/gohome/srv/carArm.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(gohome
  "/home/huo/Downloads/ur3_ws/src/omnicar/navigation/gohome/srv/carArm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gohome
)

### Generating Module File
_generate_module_cpp(gohome
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gohome
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(gohome_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(gohome_generate_messages gohome_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/omnicar/navigation/gohome/srv/carArm.srv" NAME_WE)
add_dependencies(gohome_generate_messages_cpp _gohome_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gohome_gencpp)
add_dependencies(gohome_gencpp gohome_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gohome_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(gohome
  "/home/huo/Downloads/ur3_ws/src/omnicar/navigation/gohome/srv/carArm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gohome
)

### Generating Module File
_generate_module_eus(gohome
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gohome
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(gohome_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(gohome_generate_messages gohome_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/omnicar/navigation/gohome/srv/carArm.srv" NAME_WE)
add_dependencies(gohome_generate_messages_eus _gohome_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gohome_geneus)
add_dependencies(gohome_geneus gohome_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gohome_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(gohome
  "/home/huo/Downloads/ur3_ws/src/omnicar/navigation/gohome/srv/carArm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gohome
)

### Generating Module File
_generate_module_lisp(gohome
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gohome
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(gohome_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(gohome_generate_messages gohome_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/omnicar/navigation/gohome/srv/carArm.srv" NAME_WE)
add_dependencies(gohome_generate_messages_lisp _gohome_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gohome_genlisp)
add_dependencies(gohome_genlisp gohome_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gohome_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(gohome
  "/home/huo/Downloads/ur3_ws/src/omnicar/navigation/gohome/srv/carArm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gohome
)

### Generating Module File
_generate_module_nodejs(gohome
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gohome
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(gohome_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(gohome_generate_messages gohome_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/omnicar/navigation/gohome/srv/carArm.srv" NAME_WE)
add_dependencies(gohome_generate_messages_nodejs _gohome_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gohome_gennodejs)
add_dependencies(gohome_gennodejs gohome_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gohome_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(gohome
  "/home/huo/Downloads/ur3_ws/src/omnicar/navigation/gohome/srv/carArm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gohome
)

### Generating Module File
_generate_module_py(gohome
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gohome
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(gohome_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(gohome_generate_messages gohome_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/huo/Downloads/ur3_ws/src/omnicar/navigation/gohome/srv/carArm.srv" NAME_WE)
add_dependencies(gohome_generate_messages_py _gohome_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gohome_genpy)
add_dependencies(gohome_genpy gohome_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gohome_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gohome)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gohome
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gohome)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gohome
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gohome)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gohome
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gohome)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gohome
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gohome)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gohome\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gohome
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
