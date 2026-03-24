#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
rospy.init_node('my_publisher')
# 定义一个发布者对象
pub = rospy.Publisher('my_topic', String, queue_size=10)
# 定义一个发布频率，下面的 1 代表间隔 1 秒
rate = rospy.Rate(1)
while not rospy.is_shutdown():
# 创建一个消息对象并填充数据
	msg = String()
	msg.data = 'Hello, world!' # 发布消息
	pub.publish(msg)
#沉睡 1 秒
	rate.sleep()
