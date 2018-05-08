# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "object_recognition: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iobject_recognition:/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/object-recognition/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Ibaxter_core_msgs:/scratch/shared/baxter_ws/src/baxter_common/baxter_core_msgs/msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(object_recognition_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/object-recognition/msg/ObjectInfo.msg" NAME_WE)
add_custom_target(_object_recognition_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "object_recognition" "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/object-recognition/msg/ObjectInfo.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(object_recognition
  "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/object-recognition/msg/ObjectInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_recognition
)

### Generating Services

### Generating Module File
_generate_module_cpp(object_recognition
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_recognition
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(object_recognition_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(object_recognition_generate_messages object_recognition_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/object-recognition/msg/ObjectInfo.msg" NAME_WE)
add_dependencies(object_recognition_generate_messages_cpp _object_recognition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_recognition_gencpp)
add_dependencies(object_recognition_gencpp object_recognition_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_recognition_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(object_recognition
  "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/object-recognition/msg/ObjectInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_recognition
)

### Generating Services

### Generating Module File
_generate_module_lisp(object_recognition
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_recognition
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(object_recognition_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(object_recognition_generate_messages object_recognition_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/object-recognition/msg/ObjectInfo.msg" NAME_WE)
add_dependencies(object_recognition_generate_messages_lisp _object_recognition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_recognition_genlisp)
add_dependencies(object_recognition_genlisp object_recognition_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_recognition_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(object_recognition
  "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/object-recognition/msg/ObjectInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_recognition
)

### Generating Services

### Generating Module File
_generate_module_py(object_recognition
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_recognition
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(object_recognition_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(object_recognition_generate_messages object_recognition_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/object-recognition/msg/ObjectInfo.msg" NAME_WE)
add_dependencies(object_recognition_generate_messages_py _object_recognition_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(object_recognition_genpy)
add_dependencies(object_recognition_genpy object_recognition_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS object_recognition_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_recognition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/object_recognition
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(object_recognition_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET baxter_core_msgs_generate_messages_cpp)
  add_dependencies(object_recognition_generate_messages_cpp baxter_core_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_recognition)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/object_recognition
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(object_recognition_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET baxter_core_msgs_generate_messages_lisp)
  add_dependencies(object_recognition_generate_messages_lisp baxter_core_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_recognition)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_recognition\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/object_recognition
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(object_recognition_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET baxter_core_msgs_generate_messages_py)
  add_dependencies(object_recognition_generate_messages_py baxter_core_msgs_generate_messages_py)
endif()
