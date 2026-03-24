# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "my_class_pkg: 8 messages, 1 services")

set(MSG_I_FLAGS "-Imy_class_pkg:/home/bcsh/sdp_class_ws/src/my_class_pkg/msg;-Imy_class_pkg:/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(my_class_pkg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/bcsh/sdp_class_ws/src/my_class_pkg/msg/MyMessage.msg" NAME_WE)
add_custom_target(_my_class_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_class_pkg" "/home/bcsh/sdp_class_ws/src/my_class_pkg/msg/MyMessage.msg" ""
)

get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionAction.msg" NAME_WE)
add_custom_target(_my_class_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_class_pkg" "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionAction.msg" "actionlib_msgs/GoalID:my_class_pkg/MyActionResult:std_msgs/Header:my_class_pkg/MyActionActionGoal:my_class_pkg/MyActionFeedback:my_class_pkg/MyActionGoal:my_class_pkg/MyActionActionFeedback:actionlib_msgs/GoalStatus:my_class_pkg/MyActionActionResult"
)

get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg" NAME_WE)
add_custom_target(_my_class_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_class_pkg" "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg" "actionlib_msgs/GoalID:std_msgs/Header:my_class_pkg/MyActionGoal"
)

get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg" NAME_WE)
add_custom_target(_my_class_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_class_pkg" "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg" "my_class_pkg/MyActionResult:actionlib_msgs/GoalID:std_msgs/Header:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg" NAME_WE)
add_custom_target(_my_class_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_class_pkg" "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg" "actionlib_msgs/GoalID:std_msgs/Header:my_class_pkg/MyActionFeedback:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg" NAME_WE)
add_custom_target(_my_class_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_class_pkg" "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg" ""
)

get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg" NAME_WE)
add_custom_target(_my_class_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_class_pkg" "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg" ""
)

get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg" NAME_WE)
add_custom_target(_my_class_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_class_pkg" "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg" ""
)

get_filename_component(_filename "/home/bcsh/sdp_class_ws/src/my_class_pkg/srv/MyServiceMsg.srv" NAME_WE)
add_custom_target(_my_class_pkg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "my_class_pkg" "/home/bcsh/sdp_class_ws/src/my_class_pkg/srv/MyServiceMsg.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(my_class_pkg
  "/home/bcsh/sdp_class_ws/src/my_class_pkg/msg/MyMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_class_pkg
)
_generate_msg_cpp(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_class_pkg
)
_generate_msg_cpp(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_class_pkg
)
_generate_msg_cpp(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_class_pkg
)
_generate_msg_cpp(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_class_pkg
)
_generate_msg_cpp(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_class_pkg
)
_generate_msg_cpp(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_class_pkg
)
_generate_msg_cpp(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_class_pkg
)

### Generating Services
_generate_srv_cpp(my_class_pkg
  "/home/bcsh/sdp_class_ws/src/my_class_pkg/srv/MyServiceMsg.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_class_pkg
)

### Generating Module File
_generate_module_cpp(my_class_pkg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_class_pkg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(my_class_pkg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(my_class_pkg_generate_messages my_class_pkg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bcsh/sdp_class_ws/src/my_class_pkg/msg/MyMessage.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_cpp _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionAction.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_cpp _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_cpp _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_cpp _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_cpp _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_cpp _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_cpp _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_cpp _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/src/my_class_pkg/srv/MyServiceMsg.srv" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_cpp _my_class_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_class_pkg_gencpp)
add_dependencies(my_class_pkg_gencpp my_class_pkg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_class_pkg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(my_class_pkg
  "/home/bcsh/sdp_class_ws/src/my_class_pkg/msg/MyMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_class_pkg
)
_generate_msg_eus(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_class_pkg
)
_generate_msg_eus(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_class_pkg
)
_generate_msg_eus(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_class_pkg
)
_generate_msg_eus(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_class_pkg
)
_generate_msg_eus(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_class_pkg
)
_generate_msg_eus(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_class_pkg
)
_generate_msg_eus(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_class_pkg
)

### Generating Services
_generate_srv_eus(my_class_pkg
  "/home/bcsh/sdp_class_ws/src/my_class_pkg/srv/MyServiceMsg.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_class_pkg
)

### Generating Module File
_generate_module_eus(my_class_pkg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_class_pkg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(my_class_pkg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(my_class_pkg_generate_messages my_class_pkg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bcsh/sdp_class_ws/src/my_class_pkg/msg/MyMessage.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_eus _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionAction.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_eus _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_eus _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_eus _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_eus _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_eus _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_eus _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_eus _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/src/my_class_pkg/srv/MyServiceMsg.srv" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_eus _my_class_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_class_pkg_geneus)
add_dependencies(my_class_pkg_geneus my_class_pkg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_class_pkg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(my_class_pkg
  "/home/bcsh/sdp_class_ws/src/my_class_pkg/msg/MyMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_class_pkg
)
_generate_msg_lisp(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_class_pkg
)
_generate_msg_lisp(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_class_pkg
)
_generate_msg_lisp(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_class_pkg
)
_generate_msg_lisp(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_class_pkg
)
_generate_msg_lisp(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_class_pkg
)
_generate_msg_lisp(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_class_pkg
)
_generate_msg_lisp(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_class_pkg
)

### Generating Services
_generate_srv_lisp(my_class_pkg
  "/home/bcsh/sdp_class_ws/src/my_class_pkg/srv/MyServiceMsg.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_class_pkg
)

### Generating Module File
_generate_module_lisp(my_class_pkg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_class_pkg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(my_class_pkg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(my_class_pkg_generate_messages my_class_pkg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bcsh/sdp_class_ws/src/my_class_pkg/msg/MyMessage.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_lisp _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionAction.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_lisp _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_lisp _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_lisp _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_lisp _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_lisp _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_lisp _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_lisp _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/src/my_class_pkg/srv/MyServiceMsg.srv" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_lisp _my_class_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_class_pkg_genlisp)
add_dependencies(my_class_pkg_genlisp my_class_pkg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_class_pkg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(my_class_pkg
  "/home/bcsh/sdp_class_ws/src/my_class_pkg/msg/MyMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_class_pkg
)
_generate_msg_nodejs(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_class_pkg
)
_generate_msg_nodejs(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_class_pkg
)
_generate_msg_nodejs(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_class_pkg
)
_generate_msg_nodejs(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_class_pkg
)
_generate_msg_nodejs(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_class_pkg
)
_generate_msg_nodejs(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_class_pkg
)
_generate_msg_nodejs(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_class_pkg
)

### Generating Services
_generate_srv_nodejs(my_class_pkg
  "/home/bcsh/sdp_class_ws/src/my_class_pkg/srv/MyServiceMsg.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_class_pkg
)

### Generating Module File
_generate_module_nodejs(my_class_pkg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_class_pkg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(my_class_pkg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(my_class_pkg_generate_messages my_class_pkg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bcsh/sdp_class_ws/src/my_class_pkg/msg/MyMessage.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_nodejs _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionAction.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_nodejs _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_nodejs _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_nodejs _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_nodejs _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_nodejs _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_nodejs _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_nodejs _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/src/my_class_pkg/srv/MyServiceMsg.srv" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_nodejs _my_class_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_class_pkg_gennodejs)
add_dependencies(my_class_pkg_gennodejs my_class_pkg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_class_pkg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(my_class_pkg
  "/home/bcsh/sdp_class_ws/src/my_class_pkg/msg/MyMessage.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_class_pkg
)
_generate_msg_py(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_class_pkg
)
_generate_msg_py(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_class_pkg
)
_generate_msg_py(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_class_pkg
)
_generate_msg_py(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_class_pkg
)
_generate_msg_py(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_class_pkg
)
_generate_msg_py(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_class_pkg
)
_generate_msg_py(my_class_pkg
  "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_class_pkg
)

### Generating Services
_generate_srv_py(my_class_pkg
  "/home/bcsh/sdp_class_ws/src/my_class_pkg/srv/MyServiceMsg.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_class_pkg
)

### Generating Module File
_generate_module_py(my_class_pkg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_class_pkg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(my_class_pkg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(my_class_pkg_generate_messages my_class_pkg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bcsh/sdp_class_ws/src/my_class_pkg/msg/MyMessage.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_py _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionAction.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_py _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionGoal.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_py _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionResult.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_py _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionActionFeedback.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_py _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionGoal.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_py _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionResult.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_py _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/devel/share/my_class_pkg/msg/MyActionFeedback.msg" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_py _my_class_pkg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bcsh/sdp_class_ws/src/my_class_pkg/srv/MyServiceMsg.srv" NAME_WE)
add_dependencies(my_class_pkg_generate_messages_py _my_class_pkg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(my_class_pkg_genpy)
add_dependencies(my_class_pkg_genpy my_class_pkg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS my_class_pkg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_class_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/my_class_pkg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(my_class_pkg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(my_class_pkg_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_class_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/my_class_pkg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(my_class_pkg_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(my_class_pkg_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_class_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/my_class_pkg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(my_class_pkg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(my_class_pkg_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_class_pkg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/my_class_pkg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(my_class_pkg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(my_class_pkg_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_class_pkg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_class_pkg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/my_class_pkg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(my_class_pkg_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(my_class_pkg_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
