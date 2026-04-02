#include<ros/ros.h>
#include<std_msgs/String.h>
//接收消息的回调函数
void callback(const std_msgs::String::ConstPtr& msg){
ROS_INFO("I heard: [%s]",msg->data.c_str());
}
int main(int argc,char** argv){
//初始化ROS节点
ros::init(argc,argv,"my_subscriber");
//创建节点句柄
ros::NodeHandle nh;
//定义一个订阅者对象
ros::Subscriber sub=nh.subscribe("my_topic",10,callback);
//运行节点
ros::spin();
return 0;
}
