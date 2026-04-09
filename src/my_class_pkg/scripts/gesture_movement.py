#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image  
from cv_bridge import CvBridge, CvBridgeError  
from upros_gesture import UP_Gesture
from std_srvs.srv import Empty, EmptyRequest

# 识别手势 0 闭合，5 张开
class ImageSubscriberNode:  
    def __init__(self):  
        rospy.init_node('gesture_control_node', anonymous=True)  
        self.bridge = CvBridge()  
        self.up_gesture = UP_Gesture()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)  
        self.image_pub = rospy.Publisher('/image_result', Image, queue_size=10)
        self.last_result_number = -1 # 记录上一帧识别的手势
        try:
            # 等待服务可用（带超时机制）
            rospy.wait_for_service('/upros_arm_control/grab_service', timeout=5.0)
            rospy.wait_for_service('/upros_arm_control/release_service', timeout=5.0)
            # 创建服务代理
            self.grab_proxy = rospy.ServiceProxy('/upros_arm_control/grab_service', Empty)
            self.release_proxy = rospy.ServiceProxy('/upros_arm_control/release_service', Empty)
            # 创建空请求对象
            self.empty_req = EmptyRequest()
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {str(e)}")
        except rospy.ROSException as e:
            rospy.logerr(f"ROS操作错误: {str(e)}")

    def image_callback(self, msg):   
        try:  
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            result = cv_image.copy()
            hands_landmarks = self.up_gesture.findHind(result) # 寻找手掌
            if hands_landmarks:
                resultNumber = self.up_gesture.detectNumber(hands_landmarks, result) # 识别手势代表的数字，返回0到6，没识别返回 -1 
                if (resultNumber >= 0):
                    cv2.putText(result, str(resultNumber), (150, 150), 19, 5, (255, 0, 255), 5, cv2.LINE_AA) # 绘制出识别结果
                    if (resultNumber == 0):
                        print("Claw Close")
                        if(self.last_result_number != resultNumber):
                            self.grab_proxy.call(self.empty_req) # 闭合夹爪
                    elif(resultNumber == 5):
                        print("Claw Open")
                        if(self.last_result_number != resultNumber):
                            self.release_proxy.call(self.empty_req) # 打开夹爪
                    self.last_result_number = resultNumber
                else:
                    cv2.putText(result, "NO NUMBER", (150, 150), 20, 1, (0, 0, 255))
            ros_image = self.bridge.cv2_to_imgmsg(result, "bgr8") 
            self.image_pub.publish(ros_image) # 识别结果发布，可通过rqt_image_view查看        
        except CvBridgeError as e:  
            rospy.logerr(e)  
            return             

if __name__ == '__main__':  
    try:  
        node = ImageSubscriberNode()  
        rospy.spin()  
    except rospy.ROSInterruptException:  
        pass
