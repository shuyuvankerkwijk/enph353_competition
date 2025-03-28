# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "neural_net_driving: 1 messages, 1 services")

set(MSG_I_FLAGS "-Ineural_net_driving:/home/fizzer/ros_ws/src/neural_net_driving/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(neural_net_driving_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/fizzer/ros_ws/src/neural_net_driving/msg/ImageWithID.msg" NAME_WE)
add_custom_target(_neural_net_driving_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "neural_net_driving" "/home/fizzer/ros_ws/src/neural_net_driving/msg/ImageWithID.msg" "sensor_msgs/Image:std_msgs/Header"
)

get_filename_component(_filename "/home/fizzer/ros_ws/src/neural_net_driving/srv/ImageProcessor.srv" NAME_WE)
add_custom_target(_neural_net_driving_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "neural_net_driving" "/home/fizzer/ros_ws/src/neural_net_driving/srv/ImageProcessor.srv" "sensor_msgs/Image:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(neural_net_driving
  "/home/fizzer/ros_ws/src/neural_net_driving/msg/ImageWithID.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/neural_net_driving
)

### Generating Services
_generate_srv_cpp(neural_net_driving
  "/home/fizzer/ros_ws/src/neural_net_driving/srv/ImageProcessor.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/neural_net_driving
)

### Generating Module File
_generate_module_cpp(neural_net_driving
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/neural_net_driving
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(neural_net_driving_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(neural_net_driving_generate_messages neural_net_driving_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fizzer/ros_ws/src/neural_net_driving/msg/ImageWithID.msg" NAME_WE)
add_dependencies(neural_net_driving_generate_messages_cpp _neural_net_driving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fizzer/ros_ws/src/neural_net_driving/srv/ImageProcessor.srv" NAME_WE)
add_dependencies(neural_net_driving_generate_messages_cpp _neural_net_driving_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(neural_net_driving_gencpp)
add_dependencies(neural_net_driving_gencpp neural_net_driving_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS neural_net_driving_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(neural_net_driving
  "/home/fizzer/ros_ws/src/neural_net_driving/msg/ImageWithID.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/neural_net_driving
)

### Generating Services
_generate_srv_eus(neural_net_driving
  "/home/fizzer/ros_ws/src/neural_net_driving/srv/ImageProcessor.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/neural_net_driving
)

### Generating Module File
_generate_module_eus(neural_net_driving
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/neural_net_driving
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(neural_net_driving_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(neural_net_driving_generate_messages neural_net_driving_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fizzer/ros_ws/src/neural_net_driving/msg/ImageWithID.msg" NAME_WE)
add_dependencies(neural_net_driving_generate_messages_eus _neural_net_driving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fizzer/ros_ws/src/neural_net_driving/srv/ImageProcessor.srv" NAME_WE)
add_dependencies(neural_net_driving_generate_messages_eus _neural_net_driving_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(neural_net_driving_geneus)
add_dependencies(neural_net_driving_geneus neural_net_driving_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS neural_net_driving_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(neural_net_driving
  "/home/fizzer/ros_ws/src/neural_net_driving/msg/ImageWithID.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/neural_net_driving
)

### Generating Services
_generate_srv_lisp(neural_net_driving
  "/home/fizzer/ros_ws/src/neural_net_driving/srv/ImageProcessor.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/neural_net_driving
)

### Generating Module File
_generate_module_lisp(neural_net_driving
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/neural_net_driving
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(neural_net_driving_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(neural_net_driving_generate_messages neural_net_driving_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fizzer/ros_ws/src/neural_net_driving/msg/ImageWithID.msg" NAME_WE)
add_dependencies(neural_net_driving_generate_messages_lisp _neural_net_driving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fizzer/ros_ws/src/neural_net_driving/srv/ImageProcessor.srv" NAME_WE)
add_dependencies(neural_net_driving_generate_messages_lisp _neural_net_driving_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(neural_net_driving_genlisp)
add_dependencies(neural_net_driving_genlisp neural_net_driving_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS neural_net_driving_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(neural_net_driving
  "/home/fizzer/ros_ws/src/neural_net_driving/msg/ImageWithID.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/neural_net_driving
)

### Generating Services
_generate_srv_nodejs(neural_net_driving
  "/home/fizzer/ros_ws/src/neural_net_driving/srv/ImageProcessor.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/neural_net_driving
)

### Generating Module File
_generate_module_nodejs(neural_net_driving
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/neural_net_driving
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(neural_net_driving_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(neural_net_driving_generate_messages neural_net_driving_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fizzer/ros_ws/src/neural_net_driving/msg/ImageWithID.msg" NAME_WE)
add_dependencies(neural_net_driving_generate_messages_nodejs _neural_net_driving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fizzer/ros_ws/src/neural_net_driving/srv/ImageProcessor.srv" NAME_WE)
add_dependencies(neural_net_driving_generate_messages_nodejs _neural_net_driving_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(neural_net_driving_gennodejs)
add_dependencies(neural_net_driving_gennodejs neural_net_driving_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS neural_net_driving_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(neural_net_driving
  "/home/fizzer/ros_ws/src/neural_net_driving/msg/ImageWithID.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/neural_net_driving
)

### Generating Services
_generate_srv_py(neural_net_driving
  "/home/fizzer/ros_ws/src/neural_net_driving/srv/ImageProcessor.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/neural_net_driving
)

### Generating Module File
_generate_module_py(neural_net_driving
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/neural_net_driving
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(neural_net_driving_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(neural_net_driving_generate_messages neural_net_driving_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/fizzer/ros_ws/src/neural_net_driving/msg/ImageWithID.msg" NAME_WE)
add_dependencies(neural_net_driving_generate_messages_py _neural_net_driving_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/fizzer/ros_ws/src/neural_net_driving/srv/ImageProcessor.srv" NAME_WE)
add_dependencies(neural_net_driving_generate_messages_py _neural_net_driving_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(neural_net_driving_genpy)
add_dependencies(neural_net_driving_genpy neural_net_driving_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS neural_net_driving_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/neural_net_driving)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/neural_net_driving
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(neural_net_driving_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(neural_net_driving_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/neural_net_driving)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/neural_net_driving
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(neural_net_driving_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(neural_net_driving_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/neural_net_driving)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/neural_net_driving
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(neural_net_driving_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(neural_net_driving_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/neural_net_driving)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/neural_net_driving
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(neural_net_driving_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(neural_net_driving_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/neural_net_driving)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/neural_net_driving\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/neural_net_driving
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(neural_net_driving_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(neural_net_driving_generate_messages_py sensor_msgs_generate_messages_py)
endif()
