#include <ros/ros.h>
#include <my_class_pkg/MyMessage.h> // 根据你的包名和消息名生成的头文件
int main(int argc, char** argv)
{
// 初始化节点，命名为 "my_message_publisher"
ros::init(argc, argv, "my_message_publisher");
ros::NodeHandle nh;
// 创建发布者，话题名为 "/my_topic"，队列大小 10
ros::Publisher pub = nh.advertise<my_class_pkg::MyMessage>("/my_msg_topic", 10);
// 设置发布频率（1 Hz）
ros::Rate rate(1);
int key = 0;
while (ros::ok())
{
// 创建消息对象并赋值
my_class_pkg::MyMessage msg;
msg.key = key;
msg.value = "Hello from C++ publisher, key = " + std::to_string(key);
ROS_INFO("Publishing: key=%d, value='%s'", msg.key, msg.value.c_str());
pub.publish(msg);
++key;
rate.sleep();
}
return 0;
}
