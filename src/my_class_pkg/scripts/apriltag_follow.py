#!/usr/bin/env python3  
  
import rospy  
import cv2  
from sensor_msgs.msg import Image  
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError 
import apriltag 
  
class ImageSubscriberNode:  
    
    def __init__(self):    
        rospy.init_node('image_subscriber_node', anonymous=True)  
        self.tag_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))  # 初始化AprilTag识别器
        self.follow_tag_id = 1  # 待跟踪的AprilTag的ID
        self.bridge = CvBridge()  # 初始化opencv和ros转接的桥梁 
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)  # 订阅RGB图像
        self.image_pub = rospy.Publisher('/image_result', Image, queue_size=10)  # 发布处理后的图像
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # 发布速度控制

    def image_callback(self, msg):  
        try: 
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")  # 将ROS的图像消息转化为opencv可处理的numpy矩阵
            frame = cv_image.copy()  # 复制出来一张原始图像，用于绘制识别结果
            gray_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY) # 将 rgb 图像转化为灰度图像
            height, width = gray_frame.shape  # 获取图像的宽
            tags = self.tag_detector.detect(gray_frame)  # 基于 apriltag检测器开始检验
            # 遍历识别的结果
            for tag in tags:
                # 如果识别到的tag的id与要跟踪的目标id一致
                if(tag.tag_id == self.follow_tag_id):
                    top_left, _, bottom_right, _ = tag.corners # 提取四个角点
                    center_x = int((top_left[0] + bottom_right[0]) / 2)  # 根据左上角和右下角计算出坐标中心
                    center_y = int((top_left[1] + bottom_right[1]) / 2)  # 根据左上角和右下角计算出坐标中心
                    area = int((bottom_right[0] - top_left[0])*(bottom_right[1] - top_left[1])) # 计算面积
                    cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (0, 0, 255), 2)  # 在识别到的AprilTag重心画十字 
                    cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (0, 0, 255), 2)  # 在识别到的AprilTag重心画十字
                    # 声明一个速度控制消息
                    twist = Twist()
                    # 如果识别到的Tag在右侧，则右转
                    if(center_x > width/2 + 20):
                        twist.angular.z = -0.5
                    # 如果识别到的Tag在左侧，则左转
                    elif(center_x < width/2 - 20):
                        twist.angular.z = 0.5
                    # 在中央则停止
                    else:
                        twist.angular.z = 0
                    # 识别到的apriltag面积小于 300 则前进
                    if(area < 300):
                        twist.linear.x = 0.3;
                    self.vel_pub.publish(twist)

            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.image_pub.publish(ros_image)
        except CvBridgeError as e:  
            rospy.logerr(e)  
            return   
        
    def spin(self):  
        rospy.spin()  
  
if __name__ == '__main__':  
    try:  
        node = ImageSubscriberNode()  
        node.spin()  
    except rospy.ROSInterruptException:  
        pass
