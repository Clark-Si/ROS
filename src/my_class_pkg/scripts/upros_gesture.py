#!/usr/bin/env python3

import cv2
import mediapipe as mp
import math

class UP_Gesture:
    def __init__(self):  
        self.draw = mp.solutions.drawing_utils
        self.hands = mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.75, min_tracking_confidence=0.75)

    # 检测人手函数，反馈21个点
    def findHind(self, img):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # 转换为RGB
        handlmsstyle = self.draw.DrawingSpec(color=(0, 0, 255), thickness=5)
        handconstyle = self.draw.DrawingSpec(color=(0, 255, 0), thickness=5)
        results = self.hands.process(imgRGB)
        if results.multi_hand_landmarks:
            for handLms in results.multi_hand_landmarks:
                self.draw.draw_landmarks(img, handLms, mp.solutions.hands.HAND_CONNECTIONS, handlmsstyle, handconstyle)
        return results.multi_hand_landmarks

    def detectNumber(self, hand_landmarks, img):
        h, w, _ = img.shape
        hand_landmark = hand_landmarks[0].landmark # 视野中只可以出现一只手

        thumb_tip_id = 4  # 大拇指指尖
        index_finger_tip_id = 8  # 食指指尖
        middle_finger_tip_id = 12  # 中指指尖
        ring_finger_tip_id = 16  # 无名指指尖
        pinky_finger_tip_id = 20  # 小指指尖
        pinky_finger_mcp_id = 17  # 小指指根（用于判断4和5）
        wrist_id = 0  # 手腕（用于识别数字6）

        # 提取y坐标
        thumb_tip_y = hand_landmark[thumb_tip_id].y * h  # ​拇指指尖的Y坐标
        index_tip_y = hand_landmark[index_finger_tip_id].y * h  # 食指指尖的Y坐标
        middle_tip_y = hand_landmark[middle_finger_tip_id].y * h  # 中指指尖的Y坐标
        ring_tip_y = hand_landmark[ring_finger_tip_id].y * h  # 无名指指尖的Y坐标
        pinky_tip_y = hand_landmark[pinky_finger_tip_id].y * h  # 小指指尖的Y坐标
        pinky_mcp_y = hand_landmark[pinky_finger_mcp_id].y * h  # 小指根部的Y坐标
        wrist_y = hand_landmark[wrist_id].y * h  # 手腕的Y坐标

        # 提取x坐标
        thumb_tip_x = hand_landmark[thumb_tip_id].x * w  # ​拇指指尖的X坐标
        index_tip_x = hand_landmark[index_finger_tip_id].x * w # 食指指尖的X坐标
        middle_tip_x = hand_landmark[middle_finger_tip_id].x * w # 中指指尖的X坐标
        ring_tip_x = hand_landmark[ring_finger_tip_id].x * w  # 无名指指尖的X坐标
        pinky_tip_x = hand_landmark[pinky_finger_tip_id].x * w  # 小指指尖的X坐标
        pinky_mcp_x = hand_landmark[pinky_finger_mcp_id].x * w  # 小指指尖的X坐标
        wrist_x = hand_landmark[wrist_id].x * w  # 手腕的X坐标

        dist_thumb2wrist = math.sqrt((thumb_tip_x - wrist_x)**2 + (thumb_tip_y - wrist_y)**2) # 拇指尖到手腕
        dist_index2wrist = math.sqrt((index_tip_x - wrist_x) ** 2 + (index_tip_y - wrist_y) ** 2) # 食指尖到手腕
        dist_middle2wrist = math.sqrt((middle_tip_x - wrist_x) ** 2 + (middle_tip_y - wrist_y) ** 2) # 中指尖到手腕
        dist_ring2wrist = math.sqrt((ring_tip_x - wrist_x) ** 2 + (ring_tip_y - wrist_y) ** 2)  # 无名指尖到手腕
        dist_pinky2wrist = math.sqrt((pinky_tip_x - wrist_x) ** 2 + (pinky_tip_y - wrist_y) ** 2)  # 小指尖到手腕
        dist_pinky_mcp2wrist = math.sqrt((thumb_tip_x - pinky_mcp_x)**2 + (thumb_tip_y - pinky_mcp_y)**2) # 拇指尖到小指掌指关节（MCP）的距离

        dist_index2wrist_ratio = dist_index2wrist / dist_thumb2wrist # 食指指尖到手腕的距离 / 拇指指尖到手腕的距离 → 消除手部尺寸影响，反映食指相对伸展程度
        dist_middle2wrist_ratio = dist_middle2wrist / dist_thumb2wrist  # 中指指尖到手腕的距离 / 拇指指尖到手腕的距离 → 量化中指相对于拇指的伸展比例
        dist_ring2wrist_ratio = dist_ring2wrist / dist_thumb2wrist  # 无名指指尖到手腕的距离 / 拇指指尖到手腕的距离 → 衡量无名指与拇指的长度关系
        dist_pinky2wrist_ratio = dist_pinky2wrist / dist_thumb2wrist # 小指指尖到手腕的距离 / 拇指指尖到手腕的距离 → 判断小指是否充分伸展（如"张开手掌"手势）
        dist_pinky_mcp2wrist_ratio = dist_pinky_mcp2wrist / dist_thumb2wrist # 小指掌指关节（MCP）到拇指指尖的距离 / 拇指指尖到手腕的距离 → 检测手掌闭合状态

        # 所有指尖距离手腕较近（比值均小于阈值），小指掌关节距离手腕极近，握拳动作，返回 0 
        if dist_index2wrist_ratio < 1.9 and dist_middle2wrist_ratio < 1.8 and dist_ring2wrist_ratio < 1.6 and dist_pinky2wrist_ratio < 1.4 and dist_pinky_mcp2wrist_ratio < 0.8:
            return 0
        # 食指伸直，距离手腕较远（>2.0），其余手指弯曲，小指掌关节距离手腕近，食指伸直，返回 1
        elif 2.0 < dist_index2wrist_ratio and dist_middle2wrist_ratio < 1.8 and dist_ring2wrist_ratio < 1.6 and dist_pinky2wrist_ratio < 1.4 and dist_pinky_mcp2wrist_ratio < 0.8:
            return 1
        # 食指、中指伸直，距离手腕远（>2.0和>1.9），无名指和小指弯曲，返回 2
        elif 2.0 < dist_index2wrist_ratio and 1.9 < dist_middle2wrist_ratio and dist_ring2wrist_ratio < 1.6 and dist_pinky2wrist_ratio < 1.4 and dist_pinky_mcp2wrist_ratio < 0.8:
            return 2
        # 食指、中指、无名指伸直，距离手腕远（>2.0/1.9/1.75），小指弯曲， 返回 3
        elif 2.0 < dist_index2wrist_ratio and 1.9 < dist_middle2wrist_ratio and 1.75 < dist_ring2wrist_ratio and dist_pinky2wrist_ratio < 1.4 and dist_pinky_mcp2wrist_ratio < 0.8:
            return 3
        # 食指至小指伸直，距离手腕均较远（>2.0/1.9/1.75/1.5），小指掌关节距离近， 返回 4
        elif 2.0 < dist_index2wrist_ratio and 1.9 < dist_middle2wrist_ratio and 1.75 < dist_ring2wrist_ratio and 1.5 < dist_pinky2wrist_ratio and dist_pinky_mcp2wrist_ratio < 0.8:
            return 4
        # 所有指尖距离手腕较远（>0.5），小指掌关节与手腕距离适中（0.9~1.2）， 返回 5
        elif dist_index2wrist_ratio > 0.5 and dist_middle2wrist_ratio > 0.5 and dist_ring2wrist_ratio > 0.5 and 0.9 < dist_pinky_mcp2wrist_ratio < 1.2:
            return 5
        # 食指至无名指距离手腕极近（<0.5），小指和拇指距离远， 返回 6
        elif dist_index2wrist_ratio < 0.5 and dist_middle2wrist_ratio < 0.5 and dist_ring2wrist_ratio < 0.5:
            return 6
        # 其他手势， 返回 -1
        else:
            return -1
