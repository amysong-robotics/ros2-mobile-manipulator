#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import threading
import time
import math
import json
import numpy as np
import cv2
import pyaudio

# 防止 Protobuf 冲突
os.environ['PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION'] = 'python'

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import SetBool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# AI 库
from vosk import Model, KaldiRecognizer
import mediapipe as mp
try:
    import mediapipe.python.solutions
except ImportError:
    pass

# ================= 配置区 (请修改) =================

# 1. 语音模型路径
VOSK_MODEL_PATH = "/home/syz/amy_ws/src/syz_voice_control/syz_voice_control/model"

# 2. 坐标配置
POINT_1_X, POINT_1_Y = -2.7, 0.0  # 抓取点
POINT_2_X, POINT_2_Y = -2.7, -2.03   # 放置点

# =================================================

class PIDController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.prev_error = 0.0
        self.last_time = time.time()
    def update(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0: dt = 0.001
        p_term = self.kp * error
        d_term = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        self.last_time = current_time
        return p_term + d_term

class GlobalData:
    def __init__(self):
        self.lock = threading.Lock()
        self.robot_img = None 
        self.debug_eye_img = None 
        self.status = "READY"
        self.direction_hint = ""
        self.mode = "MANUAL"     # MANUAL, NAV_P1, NAV_P2, AUTO_GRASP, AUTO_DROP, STOP
        
        # 语音相关
        self.voice_cmd_queue = ""      # 存储任务指令 (如: CMD_GRASP)
        self.voice_vel = (0.0, 0.0)    # 存储移动指令 (linear, angular)

DATA = GlobalData()

class SmartMissionNode(Node):
    def __init__(self):
        super().__init__('smart_mission_voice')
        self.bridge = CvBridge()
        self.cb_group = ReentrantCallbackGroup()
        
        self.pid_linear = PIDController(kp=0.01, kd=0.005)
        self.pid_angular = PIDController(kp=0.004, kd=0.002)

        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10, callback_group=self.cb_group)
        self.pub_init_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub_img = self.create_subscription(Image, '/front_camera_sensor/image_raw', self.img_cb, qos, callback_group=self.cb_group)
        
        self.nav = BasicNavigator()
        self.cli_arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory', callback_group=self.cb_group)
        self.cli_grip = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory', callback_group=self.cb_group)
        self.cli_vac = self.create_client(SetBool, '/gripper_switch', callback_group=self.cb_group)

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_draw = mp.solutions.drawing_utils

        self.cap = cv2.VideoCapture(0)
        self.gesture_buffer_time = 0.0
        self.last_gesture = -1
        
        self.get_logger().info(">>> 系统就绪 <<<")
        self.get_logger().info("支持语音指令: '抓取', '放下', '去一号点', '去二号点', '前进/后退/左转/右转/停'")

    def img_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with DATA.lock:
                DATA.robot_img = cv_image
        except: pass

    # ================= 语音逻辑处理 =================
    def process_voice_logic(self):
        """ 检查是否有语音触发的任务 """
        trigger_task = ""
        
        with DATA.lock:
            if DATA.voice_cmd_queue:
                trigger_task = DATA.voice_cmd_queue
                DATA.voice_cmd_queue = "" # 清除指令
        
        # 只有在手动模式下才响应任务指令，防止任务冲突
        if DATA.mode != "MANUAL" and trigger_task and trigger_task != "CMD_STOP":
            return

        if trigger_task == "CMD_NAV_P1":
            DATA.mode = "NAV_P1"
            threading.Thread(target=self.task_navigation, args=(POINT_1_X, POINT_1_Y)).start()
        
        elif trigger_task == "CMD_GRASP":
            DATA.mode = "AUTO_GRASP"
            threading.Thread(target=self.task_grasping_pid).start()
        
        elif trigger_task == "CMD_NAV_P2":
            DATA.mode = "NAV_P2"
            threading.Thread(target=self.task_navigation, args=(POINT_2_X, POINT_2_Y)).start()
        
        elif trigger_task == "CMD_DROP":
            DATA.mode = "AUTO_DROP"
            threading.Thread(target=self.task_drop_sequence).start()
        
        elif trigger_task == "CMD_STOP":
            DATA.mode = "MANUAL"
            self.nav.cancelTask()
            self.force_stop()
            with DATA.lock: DATA.voice_vel = (0.0, 0.0)

    # ================= 主循环 (手势+UI) =================
    def process_gesture_loop(self):
        if not self.cap.isOpened(): return None
        ret, frame = self.cap.read()
        if not ret: return None

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = self.hands.process(rgb)
        h, w, _ = frame.shape
        
        # 优先检查语音任务
        self.process_voice_logic()

        # 绘制中心十字
        cv2.line(frame, (w//2, h//2-20), (w//2, h//2+20), (100,100,100), 1)
        cv2.line(frame, (w//2-20, h//2), (w//2+20, h//2), (100,100,100), 1)
        
        twist = Twist()
        DATA.direction_hint = ""
        current_fingers = -1
        is_hand_controlling = False 
        
        if res.multi_hand_landmarks:
            for lm in res.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, lm, self.mp_hands.HAND_CONNECTIONS)
                current_fingers = self.count_fingers(lm)
                cx, cy = int(lm.landmark[9].x * w), int(lm.landmark[9].y * h)
                cv2.circle(frame, (cx, cy), 15, (255, 0, 0), 2)

                # 0. 手势急停
                if current_fingers == 0:
                    self.reset_gesture_buffer()
                    if DATA.mode != "STOP":
                        DATA.mode = "STOP"
                        DATA.status = "!!! EMERGENCY STOP !!!"
                        self.force_stop()
                        self.nav.cancelTask()
                        DATA.debug_eye_img = None
                        with DATA.lock: DATA.voice_vel = (0.0, 0.0)

                # 5. 手势手动控制
                elif current_fingers == 5:
                    self.reset_gesture_buffer()
                    if DATA.mode != "MANUAL":
                        DATA.mode = "MANUAL"
                        self.nav.cancelTask()
                        DATA.debug_eye_img = None
                    
                    DATA.status = "MANUAL (HAND)"
                    is_hand_controlling = True
                    deadzone = 60
                    dx, dy = cx - w//2, cy - h//2
                    
                    cv2.line(frame, (w//2, h//2), (cx, cy), (0, 255, 0), 2)

                    if abs(dy) > deadzone:
                        twist.linear.x = -0.5 if dy > 0 else 0.5
                    if abs(dx) > deadzone:
                        twist.angular.z = -1.0 if dx > 0 else 1.0

                # 手势触发任务 (1-4)
                elif DATA.mode == "MANUAL":
                    if current_fingers in [1, 2, 3, 4]:
                        if current_fingers == self.last_gesture:
                            self.gesture_buffer_time += 0.05
                        else:
                            self.gesture_buffer_time = 0.0
                            self.last_gesture = current_fingers
                        
                        progress = min(self.gesture_buffer_time / 1.5, 1.0)
                        bar_w = int(progress * 200)
                        cv2.rectangle(frame, (w//2 - 100, h - 100), (w//2 - 100 + bar_w, h - 80), (0, 255, 0), -1)
                        cv2.rectangle(frame, (w//2 - 100, h - 100), (w//2 + 100, h - 80), (255, 255, 255), 2)
                        
                        task_map = {1: "NAV P1", 2: "GRASP", 3: "NAV P2", 4: "DROP"}
                        cv2.putText(frame, f"HOLD FOR {task_map.get(current_fingers)}...", (w//2 - 60, h - 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                        if self.gesture_buffer_time > 1.5:
                            if current_fingers == 1:
                                DATA.mode = "NAV_P1"
                                threading.Thread(target=self.task_navigation, args=(POINT_1_X, POINT_1_Y)).start()
                            elif current_fingers == 2:
                                DATA.mode = "AUTO_GRASP"
                                threading.Thread(target=self.task_grasping_pid).start()
                            elif current_fingers == 3:
                                DATA.mode = "NAV_P2"
                                threading.Thread(target=self.task_navigation, args=(POINT_2_X, POINT_2_Y)).start()
                            elif current_fingers == 4:
                                DATA.mode = "AUTO_DROP"
                                threading.Thread(target=self.task_drop_sequence).start()
                            self.reset_gesture_buffer()
                    else:
                        self.reset_gesture_buffer()
        else:
            self.reset_gesture_buffer()

        # === 速度控制 (手势 > 语音) ===
        if DATA.mode == "MANUAL":
            if is_hand_controlling:
                self.pub_vel.publish(twist)
                with DATA.lock: DATA.voice_vel = (0.0, 0.0) # 手势接管时清空语音速度
            else:
                # 使用语音设定的速度
                v_x, v_z = 0.0, 0.0
                with DATA.lock: v_x, v_z = DATA.voice_vel
                
                if v_x != 0 or v_z != 0:
                    voice_twist = Twist()
                    voice_twist.linear.x = float(v_x)
                    voice_twist.angular.z = float(v_z)
                    self.pub_vel.publish(voice_twist)
                    DATA.status = "MANUAL (VOICE)"
                else:
                    self.stop_robot()
        
        cv2.putText(frame, f"MODE: {DATA.mode}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(frame, DATA.status, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

        return frame

    def reset_gesture_buffer(self):
        self.gesture_buffer_time = 0.0
        self.last_gesture = -1

    def count_fingers(self, lm):
        cnt = 0
        l = lm.landmark
        if abs(l[4].x - l[17].x) > 0.15: cnt += 1
        wrist = l[0]
        def is_open(tip, pip): return math.hypot(l[tip].x-wrist.x, l[tip].y-wrist.y) > math.hypot(l[pip].x-wrist.x, l[pip].y-wrist.y) * 1.1
        if is_open(8, 6): cnt += 1
        if is_open(12, 10): cnt += 1
        if is_open(16, 14): cnt += 1
        if is_open(20, 18): cnt += 1
        return cnt

    def stop_robot(self):
        self.pub_vel.publish(Twist())

    def force_stop(self):
        for _ in range(5):
            self.pub_vel.publish(Twist())
            time.sleep(0.02)

    # === 任务函数 ===
    def task_navigation(self, x, y):
        DATA.status = f"NAVIGATING TO ({x},{y})..."
        self.nav.clearAllCostmaps()
        time.sleep(0.5)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.nav.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = 1.0
        self.nav.goToPose(goal)
        while not self.nav.isTaskComplete():
            if DATA.mode not in ["NAV_P1", "NAV_P2"]: 
                self.nav.cancelTask()
                return
            time.sleep(0.5)
        self.force_stop()
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED: DATA.status = "ARRIVED!"
        else: DATA.status = "NAV FAILED"
        DATA.mode = "MANUAL"

    def task_grasping_pid(self):
        DATA.status = "PID SEARCHING..."
        target_area = 5000.0
        center_tolerance = 5.0
        area_tolerance = 800.0
        while DATA.mode == "AUTO_GRASP":
            img = None
            with DATA.lock:
                if DATA.robot_img is not None: img = DATA.robot_img.copy()
            if img is None:
                time.sleep(0.1); continue
            h, w, _ = img.shape
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255])) + \
                   cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
            cnts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            twist = Twist()
            largest_cnt = max(cnts, key=cv2.contourArea) if cnts else None
            max_area = cv2.contourArea(largest_cnt) if largest_cnt is not None else 0.0
            display_img = img.copy()
            if max_area > 100:
                x, y, wb, hb = cv2.boundingRect(largest_cnt)
                cv2.rectangle(display_img, (x, y), (x+wb, y+hb), (0, 255, 0), 2)
                err_x = (w / 2.0) - (x + wb/2.0)
                err_dim = math.sqrt(target_area) - math.sqrt(max_area)
                ang_vel = np.clip(self.pid_angular.update(err_x), -0.6, 0.6)
                lin_vel = np.clip(self.pid_linear.update(err_dim), -0.15, 0.2)
                if abs(err_x) < 20.0: twist.linear.x = lin_vel
                twist.angular.z = ang_vel
                self.pub_vel.publish(twist)
                if abs(err_x) < center_tolerance and abs(target_area - max_area) < area_tolerance:
                    self.force_stop()
                    DATA.debug_eye_img = display_img
                    break
            else:
                twist.angular.z = 0.3
                self.pub_vel.publish(twist)
            DATA.debug_eye_img = display_img
            time.sleep(0.05)
        
        if DATA.mode != "AUTO_GRASP": 
            DATA.debug_eye_img = None; return
        
        DATA.status = "ARM: GRASPING..."
        self.set_vacuum(False); self.move_gripper(0.02); time.sleep(1.0)
        self.move_arm([0.0, -0.777, 0.0, 0.828], 3.0); time.sleep(3.2)
        self.set_vacuum(True); time.sleep(0.5)
        self.move_gripper(-0.006); time.sleep(1.0)
        self.move_arm([0.0, 0.0, 0.0, 0.0], 2.5); time.sleep(2.7)
        DATA.status = "GRASP DONE!"; DATA.mode = "MANUAL"; DATA.debug_eye_img = None

    def task_drop_sequence(self):
        DATA.status = "ARM: DROPPING..."
        self.move_arm([0.0, -0.777, 0.0, 0.828], 3.0); time.sleep(3.2)
        self.set_vacuum(False); self.move_gripper(0.02); time.sleep(1.0)
        self.move_arm([0.0, 0.0, 0.0, 0.0], 2.5); time.sleep(2.7)
        DATA.status = "DROP DONE!"; DATA.mode = "MANUAL"

    def move_arm(self, joints, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        pt = JointTrajectoryPoint()
        pt.positions = joints
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration))*1e9)
        goal.trajectory.points = [pt]
        self.cli_arm.send_goal_async(goal)

    def move_gripper(self, pos):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['paw_joint_1', 'paw_joint_2']
        pt = JointTrajectoryPoint()
        pt.positions = [pos, pos]
        pt.time_from_start.sec = 1
        goal.trajectory.points = [pt]
        self.cli_grip.send_goal_async(goal)

    def set_vacuum(self, enable):
        req = SetBool.Request()
        req.data = enable
        self.cli_vac.call_async(req)

# ================= 语音识别线程 =================
def run_voice_system():
    if not os.path.exists(VOSK_MODEL_PATH):
        print(f"ERROR: 找不到语音模型: {VOSK_MODEL_PATH}")
        return
    model = Model(VOSK_MODEL_PATH)
    rec = KaldiRecognizer(model, 16000)
    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=4000)
    stream.start_stream()
    print(">>> 语音监听中...")
    
    while True:
        try:
            data = stream.read(4000, exception_on_overflow=False)
            text = ""
            if rec.AcceptWaveform(data): text = json.loads(rec.Result()).get('text', '')
            else: text = json.loads(rec.PartialResult()).get('partial', '')
            
            if text:
                cmd = text.replace(" ", "")
                with DATA.lock:
                    # 动作指令
                    if "抓" in cmd or "拿" in cmd: DATA.voice_cmd_queue = "CMD_GRASP"
                    elif "放" in cmd or "松" in cmd: DATA.voice_cmd_queue = "CMD_DROP"
                    elif "一号" in cmd or "出发" in cmd: DATA.voice_cmd_queue = "CMD_NAV_P1"
                    elif "二号" in cmd or "回" in cmd: DATA.voice_cmd_queue = "CMD_NAV_P2"
                    elif "停" in cmd: DATA.voice_cmd_queue = "CMD_STOP"
                    
                    # 移动指令
                    if "前" in cmd: DATA.voice_vel = (0.2, 0.0)
                    elif "后" in cmd or "退" in cmd: DATA.voice_vel = (-0.2, 0.0)
                    elif "左" in cmd: DATA.voice_vel = (0.0, 0.5)
                    elif "右" in cmd: DATA.voice_vel = (0.0, -0.5)
                    
                    if DATA.voice_cmd_queue or DATA.voice_vel != (0.0, 0.0):
                        rec.Reset()
        except: pass

def main():
    rclpy.init()
    node = SmartMissionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    voice_thread = threading.Thread(target=run_voice_system, daemon=True)
    voice_thread.start()

    try:
        while rclpy.ok():
            frame = node.process_gesture_loop()
            if DATA.debug_eye_img is not None: cv2.imshow("Robot Eye", DATA.debug_eye_img)
            else: 
                try: cv2.destroyWindow("Robot Eye")
                except: pass
            if frame is not None: cv2.imshow("Controller", frame)
            if cv2.waitKey(10) == 27: break
    except KeyboardInterrupt: pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
