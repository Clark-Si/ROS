#include<ros/ros.h>
#include<std_msgs/String.h>
int main(int argc,char **argv)
{
//初始化ROS节点
	ros::init(argc,argv,"my_publisher");
//创建节点句柄
	ros::NodeHandle nh;
//定义一个发布者对象
	ros::Publisher pub=nh.advertise<std_msgs::String>("my_topic",10);
//定义一个ros频率，间歇1.0秒
	ros::Rate rate(1.0);
	while(ros::ok())
	{
//创建一个消息对象并填充数据
	std_msgs::String msg;
	msg.data="Hello,world!";
//发布消息
	pub.publish(msg);
//间歇休息1秒
	rate.sleep();
	}
	return 0;
}
