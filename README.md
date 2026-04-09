更新至第五周作业


针对第五周作业，提供启动命令：

实验二：1.chmod指令，赋予其可执行权限：
roscd my_class_pkg/scripts
chmod a+x follow_line.py
2.运行驱动节点：roslaunch upros_bringup bringup_w2a.launch  
3. 然后运行巡线节点：rosrun my_class_pkg follow_line.py
4. 打开一个新的终端输入以下命令：rostopic pub -1 /enable_move std_msgs/Int16 "data: 1"


实验三：
1.并为上述代码赋予可执行权限。
roscd my_class_pkg/scripts
chmod a+x gesture_movement.py
chmod a+x upros_gesture.py
2.使用以下命令启动机器人运动控制：
roslaunch upros_bringup bringup_w2a.launch
3.然后运行手势控制节点：
rosrun my_class_pkg gesture_movement.py


实验四：
1.终端中赋予代码脚本可执行权限：
roscd my_class_pkg/scripts
chmod a+x apriltag_follow.py
2.打开新的终端，启动机器人的通讯：
roslaunch upros_bringup bringup_w2a.launch
3.打开新的终端，运行上述脚本文件：
rosrun my_class_pkg apriltag_follow.py


实验五：
1.启动智行-W2A的硬件通信：
roslaunch upros_bringup bringup_w2a.launch
2.启动apriltag_ros识别：
roslaunch upros_arm recognize_apriltag.launch
3.并运行上述节点：
rosrun my_class_pkg tag_grab_node
