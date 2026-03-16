#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import time
import math
import threading
import numpy as np
import cv2
import pyaudio

# ROS2 (移除了 Nav2)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from cv_bridge import CvBridge

# AI / Vision
from vosk import Model, KaldiRecognizer
import mediapipe as mp

# ================= 用户配置区 =================

# 1. 语音模型路径 (请确保路径正确)
VOSK_MODEL_PATH = "/home/syz/amy_ws/src/syz_voice_control/syz_voice_control/model"

# 2. 颜色阈值 (红色) - 用于自动抓取时的对准
LOWER_RED1 = np.array([0, 100, 100])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([160, 100, 100])
UPPER_RED2 = np.array([180, 255, 255])

# ============================================

class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.camera_image = None    # 摄像头画面
        self.voice_cmd = ""         # 语音指令
        self.gesture_vel = (0.0, 0.0) # 手势速度 (linear, angular)
        self.status_text = "Ready"  # 状态文本
        self.is_auto_acting = False # 是否正在执行自动抓取/放下动作

STATE = SharedState()

class RobotController(Node):
    def __init__(self):
        super().__init__('manual_robot_node')
        self.cb_group = ReentrantCallbackGroup()
        self.bridge = CvBridge()

        # 发布速度
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10, callback_group=self.cb_group)
        
        # 订阅摄像头
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.img_sub = self.create_subscription(Image, '/front_camera_sensor/image_raw', self.img_callback, qos, callback_group=self.cb_group)

        # 机械臂客户端
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory', callback_group=self.cb_group)
        self.grip_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory', callback_group=self.cb_group)
        self.vacuum_client = self.create_client(SetBool, '/gripper_switch', callback_group=self.cb_group)

        self.get_logger().info(">>> 机器人启动：纯手势/语音控制模式")

    def img_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with STATE.lock:
                STATE.camera_image = cv_img
        except Exception:
            pass

    def pub_vel(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.vel_pub.publish(msg)

    def stop(self):
        self.pub_vel(0.0, 0.0)

    # --- 机械臂动作封装 ---
    def arm_move(self, joints, time_sec):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        pt = JointTrajectoryPoint()
        pt.positions = joints
        pt.time_from_start.sec = int(time_sec)
        pt.time_from_start.nanosec = int((time_sec - int(time_sec))*1e9)
        goal.trajectory.points = [pt]
        self.arm_client.send_goal_async(goal)

    def gripper_move(self, pos):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['paw_joint_1', 'paw_joint_2']
        pt = JointTrajectoryPoint()
        pt.positions = [pos, pos]
        pt.time_from_start.sec = 1
        goal.trajectory.points = [pt]
        self.grip_client.send_goal_async(goal)

    def set_pump(self, enable):
        req = SetBool.Request()
        req.data = enable
        self.vacuum_client.call_async(req)

# ================= 语音线程 =================

def run_voice_recognition():
    if not os.path.exists(VOSK_MODEL_PATH):
        print(f"ERROR: 模型路径不存在 {VOSK_MODEL_PATH}")
        return

    model = Model(VOSK_MODEL_PATH)
    rec = KaldiRecognizer(model, 16000)
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=4000)
    stream.start_stream()

    print(">>> 语音监听中... (命令: 前/后/左/右/停/抓取/放下)")
    
    while True:
        try:
            data = stream.read(4000, exception_on_overflow=False)
            text = ""
            if rec.AcceptWaveform(data):
                res = json.loads(rec.Result())
                text = res.get('text', '')
            else:
                res = json.loads(rec.PartialResult())
                text = res.get('partial', '')
            
            if text:
                cmd = text.replace(" ", "")
                final_cmd = ""
                # 关键词匹配
                if "前" in cmd: final_cmd = "FORWARD"
                elif "后" in cmd or "退" in cmd: final_cmd = "BACK"
                elif "左" in cmd: final_cmd = "LEFT"
                elif "右" in cmd: final_cmd = "RIGHT"
                elif "停" in cmd: final_cmd = "STOP"
                elif "抓" in cmd: final_cmd = "GRASP"
                elif "放" in cmd or "丢" in cmd: final_cmd = "DROP"
                
                if final_cmd:
                    with STATE.lock:
                        STATE.voice_cmd = final_cmd
                        STATE.status_text = f"Voice: {final_cmd}"
                    rec.Reset()
        except Exception:
            pass

# ================= 摄像头与手势UI线程 =================

def process_gestures_and_display(mp_hands, mp_draw, hands_detector):
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if not ret: 
            time.sleep(0.1)
            continue

        frame = cv2.flip(frame, 1)
        h, w, c = frame.shape
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = hands_detector.process(rgb)
        
        g_v, g_w = 0.0, 0.0
        
        if res.multi_hand_landmarks:
            for hand_lms in res.multi_hand_landmarks:
                mp_draw.draw_landmarks(frame, hand_lms, mp_hands.HAND_CONNECTIONS)
                
                cx = int(hand_lms.landmark[9].x * w)
                cy = int(hand_lms.landmark[9].y * h)
                cv2.circle(frame, (cx, cy), 10, (255, 0, 255), -1)
                
                # 虚拟摇杆
                dx = cx - w // 2
                dy = cy - h // 2
                
                # 阈值
                if abs(dy) > 50: g_v = -0.2 if dy > 0 else 0.2 # 前后
                if abs(dx) > 50: g_w = -0.5 if dx > 0 else 0.5 # 左右
                
                cv2.line(frame, (w//2, h//2), (cx, cy), (0, 255, 0), 2)

        # 更新状态
        with STATE.lock:
            STATE.gesture_vel = (g_v, g_w)
            status = STATE.status_text
            robot_img = STATE.camera_image
            auto_active = STATE.is_auto_acting

        # 绘制UI
        color = (0, 0, 255) if auto_active else (0, 255, 0)
        cv2.putText(frame, f"STATUS: {status}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
        
        if robot_img is not None:
            # 画中画显示机器人视野
            try:
                small = cv2.resize(robot_img, (240, 180))
                frame[0:180, w-240:w] = small
            except: pass

        cv2.imshow("Hand Control", frame)
        if cv2.waitKey(1) & 0xFF == 27: break

    cap.release()
    cv2.destroyAllWindows()

# ================= 核心控制逻辑 =================

def sequence_auto_grasp(node):
    """ 自动：视觉对准 -> 抓取 """
    print(">>> 启动自动抓取序列...")
    with STATE.lock: STATE.status_text = "Auto Aligning..."
    
    # 1. 视觉 PID 寻找红色物体
    kp_rot = 0.005
    kp_dist = 0.01
    target_area = 5000  # 目标大小
    
    lost_count = 0
    
    while True:
        img = None
        with STATE.lock:
            if STATE.camera_image is not None:
                img = STATE.camera_image.copy()
        
        if img is None:
            time.sleep(0.1)
            continue
            
        h, w, _ = img.shape
        center_x = w / 2
        
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1) + cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        cmd_v, cmd_w = 0.0, 0.0
        locked = False
        
        if cnts:
            lost_count = 0
            c = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(c)
            
            if area > 100:
                x, y, wb, hb = cv2.boundingRect(c)
                cx_obj = x + wb/2
                
                err_x = center_x - cx_obj
                err_area = target_area - area
                
                # PID 计算
                cmd_w = np.clip(err_x * kp_rot, -0.5, 0.5)
                cmd_v = np.clip(err_area * kp_dist * 0.001, -0.1, 0.15)
                
                # 死区与锁定判定
                if abs(err_x) < 10 and abs(err_area) < 800:
                    node.stop()
                    locked = True
            else:
                lost_count += 1
        else:
            lost_count += 1
            
        if lost_count > 50: # 连续丢失画面，放弃
            print(">>> 丢失目标，停止抓取")
            node.stop()
            return
            
        if locked:
            print(">>> 目标锁定，执行机械臂动作")
            break
            
        node.pub_vel(cmd_v, cmd_w)
        time.sleep(0.05)
        
    # 2. 机械臂抓取动作
    with STATE.lock: STATE.status_text = "Arm: Grasping..."
    node.set_pump(False)
    node.gripper_move(0.02) # 张开
    time.sleep(1.0)
    node.arm_move([0.0, -0.777, 0.0, 0.828], 3.0) # 下探
    time.sleep(3.2)
    node.set_pump(True) # 吸气
    time.sleep(0.5)
    node.gripper_move(-0.006) # 闭合
    time.sleep(1.0)
    node.arm_move([0.0, 0.0, 0.0, 0.0], 2.5) # 复位
    time.sleep(2.7)
    print(">>> 抓取完成")

def sequence_drop(node):
    """ 自动：放下 """
    print(">>> 执行放下动作...")
    with STATE.lock: STATE.status_text = "Arm: Dropping..."
    node.arm_move([0.0, -0.777, 0.0, 0.828], 3.0) # 下探
    time.sleep(3.2)
    node.set_pump(False) # 关泵
    node.gripper_move(0.02) # 张开
    time.sleep(1.0)
    node.arm_move([0.0, 0.0, 0.0, 0.0], 2.0) # 复位
    time.sleep(2.2)
    print(">>> 放下完成")

def main_control_loop(node):
    """ 主控制循环：处理手势与语音的混合控制 """
    
    # 初始状态
    with STATE.lock: STATE.status_text = "Manual Mode"
    
    # 语音状态保持 (例如说“前进”后一直前进，直到说“停”)
    voice_running_cmd = "" 
    
    while True:
        # 1. 检查是否触发了特殊任务 (抓取/放下)
        trigger_task = ""
        with STATE.lock:
            if STATE.voice_cmd in ["GRASP", "DROP"]:
                trigger_task = STATE.voice_cmd
                STATE.voice_cmd = "" # 消费掉
        
        if trigger_task == "GRASP":
            with STATE.lock: STATE.is_auto_acting = True
            sequence_auto_grasp(node)
            with STATE.lock: 
                STATE.is_auto_acting = False
                STATE.status_text = "Manual Mode"
            continue # 任务结束，回到循环头部
            
        elif trigger_task == "DROP":
            with STATE.lock: STATE.is_auto_acting = True
            sequence_drop(node)
            with STATE.lock: 
                STATE.is_auto_acting = False
                STATE.status_text = "Manual Mode"
            continue

        # 2. 处理移动控制 (语音优先级 > 手势)
        v_cmd = ""
        g_v, g_w = 0.0, 0.0
        
        with STATE.lock:
            v_cmd = STATE.voice_cmd
            g_v, g_w = STATE.gesture_vel
            # 消费掉一次性指令，但如果是持续性指令则更新 running_cmd
            if v_cmd in ["FORWARD", "BACK", "LEFT", "RIGHT", "STOP"]:
                STATE.voice_cmd = "" # 清空缓冲
        
        # 更新语音持续状态
        if v_cmd == "STOP":
            voice_running_cmd = ""
        elif v_cmd in ["FORWARD", "BACK", "LEFT", "RIGHT"]:
            voice_running_cmd = v_cmd
            
        # 计算最终速度
        final_v, final_w = 0.0, 0.0
        
        # 语音逻辑
        if voice_running_cmd == "FORWARD": final_v = 0.2
        elif voice_running_cmd == "BACK": final_v = -0.2
        elif voice_running_cmd == "LEFT": final_w = 0.5
        elif voice_running_cmd == "RIGHT": final_w = -0.5
        
        # 手势逻辑 (仅当语音没有输出速度时生效，也就是语音说"停"之后，手势可以接管)
        if final_v == 0 and final_w == 0:
            final_v, final_w = g_v, g_w
            
        node.pub_vel(final_v, final_w)
        time.sleep(0.1)

def main():
    rclpy.init()
    node = RobotController()

    # 1. ROS 线程
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # 2. 语音识别线程
    voice_thread = threading.Thread(target=run_voice_recognition, daemon=True)
    voice_thread.start()

    # 3. 逻辑控制线程
    ctrl_thread = threading.Thread(target=main_control_loop, args=(node,), daemon=True)
    ctrl_thread.start()

    # 4. 主线程：运行手势检测与UI
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
    mp_draw = mp.solutions.drawing_utils
    
    try:
        process_gestures_and_display(mp_hands, mp_draw, hands)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
