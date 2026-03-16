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

# ROS 相关库
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient

# 消息类型
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# 导航库
from nav2_simple_commander.robot_navigator import BasicNavigator

# 图像转换
from cv_bridge import CvBridge

# 语音库
import pyaudio
from vosk import Model, KaldiRecognizer

# ================= 坐标配置 =================
PICKUP_1_X, PICKUP_1_Y = -2.7, 0.0
PICKUP_1_RZ, PICKUP_1_RW = 1.0, 0.0
# ============================================

class GlobalSharedData:
    def __init__(self):
        self.lock = threading.Lock()
        self.image = None              
        self.status_text = "Init..."   
        self.detection_info = ""       
        self.last_voice_cmd = ""       
        self.manual_active = False # 是否处于手动控制模式

SHARED_DATA = GlobalSharedData()

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

class UnifiedRobotNode(Node):
    def __init__(self):
        super().__init__('unified_voice_robot')
        self.bridge = CvBridge()
        self.cb_group = ReentrantCallbackGroup()
        
        # --- 手动控制参数 ---
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # --- 参数与发布订阅 ---
        self.declare_parameter('target_area', 5000.0)    
        self.declare_parameter('area_tolerance', 800.0) 
        self.declare_parameter('center_tolerance', 5.0)

        self.pid_linear = PIDController(kp=0.01, kd=0.005)
        self.pid_angular = PIDController(kp=0.004, kd=0.002)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.image_sub = self.create_subscription(Image, '/front_camera_sensor/image_raw', self.img_callback, qos, callback_group=self.cb_group)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10, callback_group=self.cb_group)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # 机械臂控制
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory', callback_group=self.cb_group)
        self.grip_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory', callback_group=self.cb_group)
        self.vacuum_client = self.create_client(SetBool, '/gripper_switch', callback_group=self.cb_group)

        # 定时器：用于手动控制时的持续发包 (10Hz)
        self.manual_timer = self.create_timer(0.1, self.manual_control_loop, callback_group=self.cb_group)

        # --- 语音初始化 ---
        self.init_voice_system()
        self.get_logger().info(">>> 系统就绪：等待 [导航] 指令，或直接使用 [前进/后退]")

    def init_voice_system(self):
        try:
            # 绝对路径
            model_path = "/home/syz/amy_ws/src/syz_voice_control/syz_voice_control/model"

            if not os.path.exists(model_path):
                self.get_logger().error(f"找不到模型文件夹 {model_path}")
                return 

            self.model = Model(model_path)
            self.p = pyaudio.PyAudio()
            self.stream = self.p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=4000)
            self.stream.start_stream()
            self.rec = KaldiRecognizer(self.model, 16000)
            
            self.voice_thread = threading.Thread(target=self.run_voice_loop)
            self.voice_thread.daemon = True
            self.voice_thread.start()
            
        except Exception as e:
            self.get_logger().error(f"语音初始化失败: {e}")

    def run_voice_loop(self):
        while rclpy.ok():
            try:
                data = self.stream.read(4000, exception_on_overflow=False)
                if self.rec.AcceptWaveform(data):
                    self.parse_voice_command(json.loads(self.rec.Result()).get('text', ''))
                else:
                    self.parse_voice_command(json.loads(self.rec.PartialResult()).get('partial', ''))
            except Exception:
                pass

    def parse_voice_command(self, text):
        cmd = text.replace(" ", "")
        if not cmd: return

        is_manual_cmd = False
        task_cmd = ""
        
        # === 1. 手动控制指令 ===
        if "前进" in cmd or "向前" in cmd:
            self.target_linear = 0.2
            self.target_angular = 0.0
            is_manual_cmd = True
        elif "后退" in cmd or "向后" in cmd:
            self.target_linear = -0.2
            self.target_angular = 0.0
            is_manual_cmd = True
        elif "左转" in cmd:
            self.target_linear = 0.0
            self.target_angular = 0.5
            is_manual_cmd = True
        elif "右转" in cmd:
            self.target_linear = 0.0
            self.target_angular = -0.5
            is_manual_cmd = True
        elif "停" in cmd:
            self.target_linear = 0.0
            self.target_angular = 0.0
            is_manual_cmd = True
            task_cmd = "STOP" 

        # === 2. 任务触发指令 ===
        if "导航" in cmd or "出发" in cmd:
            task_cmd = "NAV_START"
        elif "抓取" in cmd or "抓" in cmd:
            task_cmd = "GRASP_START"
        elif "放下" in cmd or "松开" in cmd or "丢" in cmd:  # 新增
            task_cmd = "DROP_START"

        # === 状态更新 ===
        with SHARED_DATA.lock:
            if is_manual_cmd:
                SHARED_DATA.manual_active = True
                self.get_logger().info(f"手动指令: {cmd}")
            
            if task_cmd:
                SHARED_DATA.last_voice_cmd = task_cmd
                # 触发自动任务时，暂时关闭手动模式
                if task_cmd in ["NAV_START", "GRASP_START", "DROP_START"]:
                    SHARED_DATA.manual_active = False
                    self.get_logger().info(f"任务指令触发: {task_cmd}")
        
        if is_manual_cmd or task_cmd:
            self.rec.Reset()

    def manual_control_loop(self):
        with SHARED_DATA.lock:
            active = SHARED_DATA.manual_active
        
        if active:
            msg = Twist()
            msg.linear.x = float(self.target_linear)
            msg.angular.z = float(self.target_angular)
            self.vel_pub.publish(msg)

    def stop_robot(self):
        self.vel_pub.publish(Twist())
        self.target_linear = 0.0
        self.target_angular = 0.0

    def reset_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = [0.0]*36
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.05
        self.pose_pub.publish(msg)

    def img_callback(self, msg):
        try:
            with SHARED_DATA.lock:
                SHARED_DATA.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: pass

    # --- 视觉对准逻辑 ---
    def process_vision(self):
        with SHARED_DATA.lock:
            if SHARED_DATA.image is None: return False
            img = SHARED_DATA.image.copy()

        h, w, _ = img.shape
        center_x = w / 2.0
        
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255])) + \
               cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        largest_cnt = max(contours, key=cv2.contourArea) if contours else None
        max_area = cv2.contourArea(largest_cnt) if largest_cnt is not None else 0.0

        if max_area > 100:
            x, y, wb, hb = cv2.boundingRect(largest_cnt)
            obj_cx = x + wb/2.0
            
            target_area = self.get_parameter('target_area').value
            err_x = center_x - obj_cx
            err_dim = math.sqrt(target_area) - math.sqrt(max_area)
            
            SHARED_DATA.detection_info = f"ErrX:{err_x:.1f}"

            ang_vel = np.clip(self.pid_angular.update(err_x), -0.6, 0.6)
            if abs(err_x) > 3.0 and abs(ang_vel) < 0.1:
                ang_vel = 0.1 if ang_vel > 0 else -0.1
            twist = Twist()
            twist.angular.z = ang_vel

            lin_vel = self.pid_linear.update(err_dim)
            if abs(err_x) < 20.0:
                twist.linear.x = np.clip(lin_vel, -0.15, 0.2)
            else:
                twist.linear.x = 0.0
            self.vel_pub.publish(twist)

            if abs(err_x) < self.get_parameter('center_tolerance').value and \
               abs(target_area - max_area) < self.get_parameter('area_tolerance').value:
                self.stop_robot()
                SHARED_DATA.detection_info += " [LOCKED]"
                return True
        else:
            SHARED_DATA.detection_info = "Searching..."
            twist = Twist()
            twist.angular.z = 0.3
            self.vel_pub.publish(twist)
            
        return False

    def send_arm(self, joints, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        pt = JointTrajectoryPoint()
        pt.positions = joints
        pt.time_from_start.sec = int(duration)
        pt.time_from_start.nanosec = int((duration - int(duration))*1e9)
        goal.trajectory.points = [pt]
        self.arm_client.send_goal_async(goal)

    def send_gripper(self, pos):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['paw_joint_1', 'paw_joint_2']
        pt = JointTrajectoryPoint()
        pt.positions = [pos, pos]
        pt.time_from_start.sec = 1
        goal.trajectory.points = [pt]
        self.grip_client.send_goal_async(goal)

    def set_vacuum(self, enable):
        req = SetBool.Request()
        req.data = enable
        self.vacuum_client.call_async(req)

# ================= 辅助函数 =================
def create_pose(navigator, x, y, rz, rw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = navigator.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = float(rz)
    p.pose.orientation.w = float(rw)
    return p

def spin_and_refresh(duration=0.01):
    end_time = time.time() + duration
    while time.time() < end_time:
        with SHARED_DATA.lock:
            if SHARED_DATA.image is not None:
                display_img = SHARED_DATA.image.copy()
                h, w, _ = display_img.shape
                cv2.rectangle(display_img, (0, 0), (w, 100), (0, 0, 0), -1)
                
                status_color = (0, 255, 0)
                if SHARED_DATA.manual_active:
                    status_str = "MANUAL MODE"
                    status_color = (0, 0, 255)
                else:
                    status_str = f"AUTO: {SHARED_DATA.status_text}"
                
                cv2.putText(display_img, status_str, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                cv2.putText(display_img, f"CMD: {SHARED_DATA.last_voice_cmd}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(display_img, SHARED_DATA.detection_info, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                cv2.imshow("Robot Monitor", display_img)
                cv2.waitKey(1)
        time.sleep(0.02)

def wait_for_voice_command(target_cmd):
    print(f">>> 等待语音: '{target_cmd}' (此时你也可以说'前进'来手动玩)...")
    SHARED_DATA.status_text = f"Wait: {target_cmd}"
    with SHARED_DATA.lock: SHARED_DATA.last_voice_cmd = ""
    while True:
        spin_and_refresh(0.1)
        with SHARED_DATA.lock: current_cmd = SHARED_DATA.last_voice_cmd
        if current_cmd == target_cmd:
            print(f">>> 确认指令: {target_cmd}")
            return
        elif current_cmd == "STOP":
            with SHARED_DATA.lock: SHARED_DATA.last_voice_cmd = ""

# ================= 动作序列 =================
def do_grasp_sequence(node):
    SHARED_DATA.status_text = "Action: Grasping"
    node.set_vacuum(False)
    node.send_gripper(0.02)
    spin_and_refresh(1.0)
    node.send_arm([0.0, -0.777, 0.0, 0.828], 3.0)
    spin_and_refresh(3.2)
    node.set_vacuum(True)
    spin_and_refresh(0.5)
    node.send_gripper(-0.006)
    spin_and_refresh(1.0)
    node.send_arm([0.0, 0.0, 0.0, 0.0], 2.5)
    spin_and_refresh(2.7)

def do_drop_sequence(node):
    SHARED_DATA.status_text = "Action: Dropping"
    # 伸出
    node.send_arm([0.0, -0.777, 0.0, 0.828], 3.0)
    spin_and_refresh(3.2)
    # 松开
    node.set_vacuum(False)
    node.send_gripper(0.02)
    spin_and_refresh(1.0)
    # 收回
    node.send_arm([0.0, 0.0, 0.0, 0.0], 2.5)
    spin_and_refresh(2.7)

# ================= 主程序 =================
def main():
    rclpy.init()
    node = UnifiedRobotNode()
    navigator = BasicNavigator()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.reset_initial_pose()
    spin_and_refresh(1.0)

    print(">>> 正在等待 Nav2 导航栈启动...")
    navigator.waitUntilNav2Active() 
    print(">>> Nav2 已就绪！")

    try:
        # 1. 导航
        wait_for_voice_command("NAV_START")
        print(">>> [Task] 导航...")
        SHARED_DATA.status_text = "Navigating..."
        navigator.clearAllCostmaps()
        spin_and_refresh(0.5)
        navigator.goToPose(create_pose(navigator, PICKUP_1_X, PICKUP_1_Y, PICKUP_1_RZ, PICKUP_1_RW))
        while not navigator.isTaskComplete():
            spin_and_refresh(0.1)
        node.stop_robot()
        
        # 2. 抓取
        wait_for_voice_command("GRASP_START")
        print(">>> [Task] 视觉搜索...")
        SHARED_DATA.status_text = "Visual Align..."
        while True:
            if node.process_vision(): break
            spin_and_refresh(0.05)
        print(">>> [Task] 抓取中...")
        do_grasp_sequence(node)

        # 3. 放下 (新增)
        wait_for_voice_command("DROP_START")
        print(">>> [Task] 正在放下物体...")
        do_drop_sequence(node)

        # 4. 结束
        SHARED_DATA.status_text = "DONE. Manual Mode"
        print(">>> 任务完成，进入手动模式")
        SHARED_DATA.manual_active = True 
        while True:
            spin_and_refresh(1.0)

    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
