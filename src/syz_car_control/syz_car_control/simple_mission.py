#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import math
import threading
import mediapipe as mp
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# ================= 目标点配置 (请按需修改) =================
TARGET_X = -2.7
TARGET_Y = 0.0
TARGET_RZ = 1.0  # 朝向 (Z分量)
TARGET_RW = 0.0  # 朝向 (W分量)
# ========================================================

class GlobalImageStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.robot_image = None
        self.webcam_image = None
        self.status = "Waiting..."

IMG_STORE = GlobalImageStore()

# === 1. 手势识别模块 ===
class GestureRecognizer:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.5)
        self.mp_draw = mp.solutions.drawing_utils
        self.cap = cv2.VideoCapture(0)

    def update(self):
        ret, frame = self.cap.read()
        if not ret: return None
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)
        
        detected = None
        text = "No Hand"

        if results.multi_hand_landmarks:
            for lm in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, lm, self.mp_hands.HAND_CONNECTIONS)
                l = lm.landmark
                # 简单距离法判断手指伸直
                def is_straight(tip, pip):
                    return math.hypot(l[tip].x-l[0].x, l[tip].y-l[0].y) > math.hypot(l[pip].x-l[0].x, l[pip].y-l[0].y) + 0.02
                
                idx = is_straight(8, 6)
                mid = is_straight(12, 10)
                ring = is_straight(16, 14)
                pinky = is_straight(20, 18)

                # 手势 1: 食指直，其他弯
                if idx and not mid and not ring and not pinky:
                    detected = 1
                    text = "Gesture: 1 (GO)"
                # 手势 2: 食指中指直
                elif idx and mid and not ring:
                    detected = 2
                    text = "Gesture: 2 (GRASP)"

        cv2.putText(frame, text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        with IMG_STORE.lock:
            IMG_STORE.webcam_image = frame
        return detected

# === 2. 机器人控制节点 ===
class RobotController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.bridge = CvBridge()
        self.cb_group = ReentrantCallbackGroup()
        
        # 视觉 PID 参数
        self.kp_ang = 0.004
        self.kp_lin = 0.01

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub_img = self.create_subscription(Image, '/front_camera_sensor/image_raw', self.img_cb, qos, callback_group=self.cb_group)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10, callback_group=self.cb_group)
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # 机械臂控制
        self.cli_arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory', callback_group=self.cb_group)
        self.cli_grip = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory', callback_group=self.cb_group)
        self.cli_vac = self.create_client(SetBool, '/gripper_switch', callback_group=self.cb_group)

    def img_cb(self, msg):
        try:
            with IMG_STORE.lock:
                IMG_STORE.robot_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: pass

    def reset_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.orientation.w = 1.0 # 默认朝向0度
        for i in range(36): msg.pose.covariance[i] = 0.0
        msg.pose.covariance[0] = 0.25; msg.pose.covariance[7] = 0.25; msg.pose.covariance[35] = 0.06
        self.pub_pose.publish(msg)

    def visual_servo(self):
        """视觉对准逻辑"""
        with IMG_STORE.lock:
            if IMG_STORE.robot_image is None: return False
            img = IMG_STORE.robot_image.copy()

        h, w, _ = img.shape
        # 红色物体识别
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255)) + cv2.inRange(hsv, (160, 100, 100), (180, 255, 255))
        cnts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        target = max(cnts, key=cv2.contourArea) if cnts else None
        if target is None or cv2.contourArea(target) < 100:
            twist = Twist()
            twist.angular.z = 0.3 # 没找到就转圈找
            self.pub_vel.publish(twist)
            return False

        x, y, wb, hb = cv2.boundingRect(target)
        err_x = (w/2) - (x + wb/2)
        err_size = 5000 - cv2.contourArea(target) # 目标大小 5000

        twist = Twist()
        twist.angular.z = np.clip(self.kp_ang * err_x, -0.5, 0.5)
        twist.linear.x = np.clip(self.kp_lin * math.sqrt(abs(err_size)) * (1 if err_size>0 else -1), -0.15, 0.15)
        
        # 对准判断
        if abs(err_x) < 20: # 左右误差小
            twist.linear.x = 0.05 # 慢速靠近
            
        if abs(err_x) < 10 and abs(err_size) < 800:
            self.pub_vel.publish(Twist()) # 停车
            return True # 完成

        self.pub_vel.publish(twist)
        return False

    def move_arm(self, joints):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        pt = JointTrajectoryPoint()
        pt.positions = joints
        pt.time_from_start.sec = 2
        goal.trajectory.points = [pt]
        self.cli_arm.send_goal_async(goal)
        time.sleep(2.5)

    def move_gripper(self, pos, vacuum):
        # 1. 夹爪开合
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['paw_joint_1', 'paw_joint_2']
        pt = JointTrajectoryPoint()
        pt.positions = [pos, pos]
        pt.time_from_start.sec = 1
        goal.trajectory.points = [pt]
        self.cli_grip.send_goal_async(goal)
        time.sleep(1.2)
        # 2. 吸盘开关
        self.cli_vac.call_async(SetBool.Request(data=vacuum))

# === 主逻辑 ===
def main():
    rclpy.init()
    node = RobotController()
    nav = BasicNavigator()
    hand = GestureRecognizer()
    
    # 多线程运行ROS回调
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    # 初始化位置 (假定Gazebo在原点)
    node.reset_pose()
    
    # 界面显示循环
    def show_ui():
        with IMG_STORE.lock:
            if IMG_STORE.webcam_image is not None:
                img = IMG_STORE.webcam_image.copy()
                cv2.putText(img, IMG_STORE.status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,255), 2)
                cv2.imshow("Control", img)
                cv2.waitKey(1)

    try:
        # --- 阶段 1: 等待手势 '1' ---
        print("等待手势 1...")
        IMG_STORE.status = "Wait Gesture 1 -> GO"
        while True:
            if hand.update() == 1: break
            show_ui()
            time.sleep(0.05)

        # --- 阶段 2: 导航 ---
        print("开始导航...")
        IMG_STORE.status = "Navigating..."
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = nav.get_clock().now().to_msg()
        goal.pose.position.x = TARGET_X
        goal.pose.position.y = TARGET_Y
        goal.pose.orientation.z = TARGET_RZ
        goal.pose.orientation.w = TARGET_RW
        
        nav.goToPose(goal)
        while not nav.isTaskComplete():
            show_ui()
            hand.update() # 保持摄像头画面刷新
            time.sleep(0.1)
        
        # --- 阶段 3: 等待手势 '2' ---
        print("到达! 等待手势 2...")
        IMG_STORE.status = "Arrived! Wait 2 -> GRASP"
        while True:
            if hand.update() == 2: break
            show_ui()
            time.sleep(0.05)

        # --- 阶段 4: 视觉抓取 ---
        print("开始视觉对准...")
        IMG_STORE.status = "Visual Servoing..."
        while not node.visual_servo():
            show_ui()
            hand.update()
            time.sleep(0.05)

        print("抓取动作...")
        IMG_STORE.status = "Grasping..."
        show_ui()
        
        # 抓取序列 (张开 -> 伸出 -> 吸住 -> 收回)
        node.move_gripper(0.02, False) # 张开
        node.move_arm([0.0, -0.7, 0.0, 0.8]) # 伸出 (请根据实际调整)
        node.move_gripper(-0.005, True) # 闭合+吸气
        node.move_arm([0.0, 0.0, 0.0, 0.0]) # 收回

        IMG_STORE.status = "DONE!"
        print("任务完成!")
        while True:
            show_ui()
            hand.update()
            time.sleep(0.1)

    except KeyboardInterrupt: pass
    finally:
        node.pub_vel.publish(Twist()) # 停车
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
