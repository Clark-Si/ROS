cmake_minimum_required(VERSION 3.0.2)
project(my_class_pkg)

# 编译为C++11标准（ROS Noetic推荐）
add_compile_options(-std=c++11)

# 查找catkin依赖（合并重复的find_package，补充完整依赖）
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
  dynamic_reconfigure # 动态配置依赖
  tf2_ros
  upros_message
)

# 生成动态配置文件（必须放在catkin_package之前）
generate_dynamic_reconfigure_options(
  cfg/Tutorials.cfg
)

# 生成自定义消息（msg文件夹）
add_message_files(
  DIRECTORY msg
  FILES MyMessage.msg
)

# 生成自定义服务（srv文件夹）
add_service_files(
  DIRECTORY srv
  FILES MyServiceMsg.srv
)

# 生成自定义动作（action文件夹）
add_action_files(
  DIRECTORY action
  FILES MyAction.action
)

# 生成消息/服务/动作（依赖std_msgs和actionlib_msgs）
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

# catkin包配置（补充INCLUDE_DIRS/LIBRARIES，完善依赖）
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_class_pkg
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    message_runtime 
    actionlib 
    actionlib_msgs
    dynamic_reconfigure # 补充动态配置依赖
    tf2_ros
    upros_message
)

# 包含头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ========== 编译所有节点（核心修复：补充pthread+add_dependencies） ==========
# 1. 发布者节点
add_executable(ros_publisher_node src/ros_publisher.cpp)
target_link_libraries(ros_publisher_node ${catkin_LIBRARIES})
add_dependencies(ros_publisher_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg # 动态配置依赖
)

# 2. 订阅者节点
add_executable(ros_subscriber_node src/ros_subscriber.cpp)
target_link_libraries(ros_subscriber_node ${catkin_LIBRARIES})
add_dependencies(ros_subscriber_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

# 3. 服务端节点
add_executable(ros_server_node src/ros_server.cpp)
target_link_libraries(ros_server_node ${catkin_LIBRARIES})
add_dependencies(ros_server_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

# 4. 客户端节点
add_executable(ros_client_node src/ros_client.cpp)
target_link_libraries(ros_client_node ${catkin_LIBRARIES})
add_dependencies(ros_client_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

# 5. 动作服务器节点（核心修复：添加pthread+add_dependencies）
add_executable(ros_action_server src/ros_action_server.cpp)
target_link_libraries(ros_action_server 
  ${catkin_LIBRARIES}
  pthread # 解决pthread_condattr_setclock未定义错误
)
add_dependencies(ros_action_server 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

# 6. 动作客户端节点（核心修复：添加pthread+add_dependencies）
add_executable(ros_action_client src/ros_action_client.cpp)
target_link_libraries(ros_action_client 
  ${catkin_LIBRARIES}
  pthread # 解决pthread_condattr_setclock未定义错误
)
add_dependencies(ros_action_client 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

# 7. 参数节点
add_executable(ros_param src/ros_param.cpp)
target_link_libraries(ros_param ${catkin_LIBRARIES})
add_dependencies(ros_param 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
add_executable(dynamic_reconfigure_node src/dynamic_reconfigure.cpp)
add_dependencies(dynamic_reconfigure_node ${PROJECT_NAME}_gencfg)
target_link_libraries(dynamic_reconfigure_node ${catkin_LIBRARIES})
add_executable(ros_dynamic_speed_node src/ros_dynamic_speed.cpp)
add_dependencies(ros_dynamic_speed_node my_class_pkg_gencfg)
target_link_libraries(ros_dynamic_speed_node ${catkin_LIBRARIES} ${my_class_pkg_LIBRARIES})
add_executable(ros_log src/ros_log.cpp)
target_link_libraries(ros_log ${catkin_LIBRARIES})
add_executable(tag_grab_node src/tag_grab.cpp)
target_link_libraries(tag_grab_node ${catkin_LIBRARIES})
# （可选）如果有msg_subscriber_node，补充编译规则
# add_executable(msg_subscriber_node src/msg_subscriber.cpp)
# target_link_libraries(msg_subscriber_node ${catkin_LIBRARIES})
# add_dependencies(msg_subscriber_node 
#   ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#   ${catkin_EXPORTED_TARGETS}
#   ${PROJECT_NAME}_gencfg
# )
