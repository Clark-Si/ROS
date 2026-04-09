#!/usr/bin/env python3  
  
import rospy  
import cv2  
import numpy as np
from sensor_msgs.msg import Image  
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError  

class ImageSubscriberNode:  
    def __init__(self):  
  
        rospy.init_node('image_subscriber_node', anonymous=True)  # 初始化节点
        
        self.hmin = 17  # 线条颜色的HSV:101 168 237(blue)
        self.smin = 70
        self.vmin = 130
        self.hmax = 25
        self.smax = 140
        self.vmax = 210
        
        self.enable_move = False # 移动标志位

        self.bridge = CvBridge()  
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)  
        self.enable_sub = rospy.Subscriber('/enable_move', Int16, self.enable_callback)
        self.image_mask_pub = rospy.Publisher('/image_mask', Image, queue_size=10)
        self.image_result_pub = rospy.Publisher('/image_result', Image, queue_size=10)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # 创建一个发布者对象，发布移动速度

    
    def enable_callback(self, msg):
        if(msg.data == 1):
            self.enable_move = True
               
    def image_callback(self, msg):  
        try:  
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  
            src = cv_image.copy()
            self.update_frame(src, self.hmin, self.hmax, self.smin, self.smax, self.vmin, self.vmax)  # 根据线条HSV移动     
        except CvBridgeError as e:  
            rospy.logerr(e)  
            return    
    
    def update_frame(self, img, h_min, h_max, s_min, s_max, v_min, v_max):
        
        result = img

        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) # bgr 转为 hsv 颜色空间

        low_color = np.array([h_min, s_min, v_min])
        high_color = np.array([h_max, s_max, v_max])
        mask_color = cv2.inRange(hsv_frame, low_color, high_color) # 颜色二值化
        mask_color = cv2.medianBlur(mask_color, 7)  # 中值滤波
        h, w, _ = img.shape

        search_top = 5*h//6 
        mask_color[0:search_top, 0:w] = 0 # 排除干扰，只跟踪眼前的线条 

        ros_mask_image = self.bridge.cv2_to_imgmsg(mask_color, "8UC1")
        self.image_mask_pub.publish(ros_mask_image) # 发布掩膜图像
        
        M = cv2.moments(mask_color) # 求截取区域的代数中心，并在此中心画一个红色实心小圆来代表它
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            err = cx - w/2 # 计算线条中心与图像中心的偏移量
            linear_x = 0.5 # 机器人前进速度
            angular_z = -float(err) / 500 # 机器人左右自旋速度
            self.move_up(linear_x, 0.0, angular_z) # 机器人执行速度控制
            cv2.circle(result, (cx, cy), 20, (0,0,255), -1)
        else: 
            self.move_up(0.0, 0.0, 0.0)    
        ros_result_image = self.bridge.cv2_to_imgmsg(result, "bgr8")
        self.image_result_pub.publish(ros_result_image) # 发布结果图像

    # 生成移动命令
    def move_up(self, x, y, th):
        t = Twist()
        t.linear.x = x
        t.linear.y = y
        t.angular.z = th
        if (self.enable_move):
            self.cmd_pub.publish(t)

    def spin(self):  
        # 让ROS节点保持运行，直到被关闭  
        rospy.spin()  
  
if __name__ == '__main__':  
    try:  
        node = ImageSubscriberNode()  
        node.spin()  
    except rospy.ROSInterruptException:  
        pass
