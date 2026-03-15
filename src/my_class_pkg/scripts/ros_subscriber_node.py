#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
#定义一个接收函数，打印接收的消息
def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
def my_subscriber():
	rospy.init_node('my_subscriber', anonymous=True)
#声明一个接收者
	rospy.Subscriber("my_topic", String, callback)
#不断回调
	rospy.spin()
if __name__ == '__main__':
	try:
		my_subscriber()
	except rospy.ROSInterruptException:
		pass
