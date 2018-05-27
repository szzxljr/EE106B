# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "baxter_image: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ibaxter_image:/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/baxter_image/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Ibaxter_core_msgs:/scratch/shared/baxter_ws/src/baxter_common/baxter_core_msgs/msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(baxter_image_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/baxter_image/msg/ObjectInfo.msg" NAME_WE)
add_custom_target(_baxter_image_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "baxter_image" "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/baxter_image/msg/ObjectInfo.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(baxter_image
  "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/baxter_image/msg/ObjectInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/baxter_image
)

### Generating Services

### Generating Module File
_generate_module_cpp(baxter_image
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/baxter_image
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(baxter_image_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(baxter_image_generate_messages baxter_image_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/baxter_image/msg/ObjectInfo.msg" NAME_WE)
add_dependencies(baxter_image_generate_messages_cpp _baxter_image_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(baxter_image_gencpp)
add_dependencies(baxter_image_gencpp baxter_image_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS baxter_image_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(baxter_image
  "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/baxter_image/msg/ObjectInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/baxter_image
)

### Generating Services

### Generating Module File
_generate_module_lisp(baxter_image
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/baxter_image
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(baxter_image_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(baxter_image_generate_messages baxter_image_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/baxter_image/msg/ObjectInfo.msg" NAME_WE)
add_dependencies(baxter_image_generate_messages_lisp _baxter_image_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(baxter_image_genlisp)
add_dependencies(baxter_image_genlisp baxter_image_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS baxter_image_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(baxter_image
  "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/baxter_image/msg/ObjectInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/baxter_image
)

### Generating Services

### Generating Module File
_generate_module_py(baxter_image
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/baxter_image
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(baxter_image_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(baxter_image_generate_messages baxter_image_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106b/sp18/class/ee106b-aaf/ros_workspaces/lab1_ws/src/baxter_image/msg/ObjectInfo.msg" NAME_WE)
add_dependencies(baxter_image_generate_messages_py _baxter_image_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(baxter_image_genpy)
add_dependencies(baxter_image_genpy baxter_image_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS baxter_image_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/baxter_image)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/baxter_image
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(baxter_image_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET baxter_core_msgs_generate_messages_cpp)
  add_dependencies(baxter_image_generate_messages_cpp baxter_core_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/baxter_image)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/baxter_image
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(baxter_image_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET baxter_core_msgs_generate_messages_lisp)
  add_dependencies(baxter_image_generate_messages_lisp baxter_core_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/baxter_image)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/baxter_image\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/baxter_image
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(baxter_image_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET baxter_core_msgs_generate_messages_py)
  add_dependencies(baxter_image_generate_messages_py baxter_core_msgs_generate_messages_py)
endif()
