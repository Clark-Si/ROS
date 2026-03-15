cmake_minimum_required(VERSION 3.0.2)
project(my_class_pkg)

# 编译为C++11标准（ROS Noetic推荐）
add_compile_options(-std=c++11)

# 查找catkin依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
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
add_action_files(
	DIRECTORY action
	FILES MyAction.action
)
# 生成消息/服务（依赖std_msgs）
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

# catkin包配置（必须包含message_runtime）
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime actionlib actionlib_msgs
)

# 包含头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ========== 编译所有节点（核心修复：取消注释+统一命名） ==========
# 1. 发布者节点（编译后可执行文件名：ros_publisher_node）
add_executable(ros_publisher_node src/ros_publisher.cpp)
target_link_libraries(ros_publisher_node ${catkin_LIBRARIES})
# 依赖自定义消息生成（避免编译顺序问题）
add_dependencies(ros_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 2. 订阅者节点（编译后可执行文件名：ros_subscriber_node）
add_executable(ros_subscriber_node src/ros_subscriber.cpp)
target_link_libraries(ros_subscriber_node ${catkin_LIBRARIES})
add_dependencies(ros_subscriber_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 3. 服务端节点（编译后可执行文件名：ros_server_node）
add_executable(ros_server_node src/ros_server.cpp)
target_link_libraries(ros_server_node ${catkin_LIBRARIES})
add_dependencies(ros_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 4. 客户端节点（编译后可执行文件名：ros_client_node）
add_executable(ros_client_node src/ros_client.cpp)
target_link_libraries(ros_client_node ${catkin_LIBRARIES})
add_dependencies(ros_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(ros_action_server src/ros_action_server.cpp)
target_link_libraries(ros_action_server ${catkin_LIBRARIES})

add_executable(ros_action_client src/ros_action_client.cpp)
target_link_libraries(ros_action_client ${catkin_LIBRARIES})

# （可选）如果有msg_subscriber_node，补充编译规则（根据你的编译日志）
# add_executable(msg_subscriber_node src/msg_subscriber.cpp)
# target_link_libraries(msg_subscriber_node ${catkin_LIBRARIES})
# add_dependencies(msg_subscriber_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
