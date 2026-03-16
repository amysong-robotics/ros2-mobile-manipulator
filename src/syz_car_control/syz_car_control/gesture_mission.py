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

# ==================================================================================
#                                   1. 坐标配置
# ==================================================================================

# --- 任务 1: 去第一个可乐 ---
PICKUP_1_X, PICKUP_1_Y = -2.7, 0.0
PICKUP_1_RZ, PICKUP_1_RW = 1.0, 0.0

# --- 任务 1: 去卸货点 (途径点) ---
WAYPOINTS_LIST = [
    (-2.2968, -3.31792, -0.707, 0.707), 
    (-4.25566, -4.982, 1.0, 0.0),   
    (-5.9662, -6.2957, 1.0, 0.0) 
]

# --- 卸货点 (通用) ---
DROP_X, DROP_Y = -8.1566, -6.406
DROP_RZ, DROP_RW = 1.0, 0.0

# --- 任务 2: 过渡路径 (卸货点 -> 抓取点2) ---
WAYPOINTS_TRANSIT_LIST = [
    (-8.1566, -6.406, 0.0, 1.0),
    (-5.065, -5.7347, 0.0, 1.0),  
    (-4.94097, -3.9266, 0.707, 0.707),  
    (-9.044, -0.7727, 0.707, 0.707)   
]

# --- 任务 2: 第二个可乐 ---
PICKUP_2_X, PICKUP_2_Y = -9.044, -0.772
PICKUP_2_RZ, PICKUP_2_RW = 0.707, 0.707

# --- 任务 2: 返回卸货点 (途径点) ---
WAYPOINTS_2_LIST = [
    (-8.044, -0.5727, -0.707, 0.707) ,
    (-4.94097, -3.9266, -0.707, 0.707),
    (-5.065, -5.7347, 1.0, 0.0), 
]

# ==================================================================================
#                                2. 辅助类定义
# ==================================================================================

class GlobalImageStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.robot_image = None     # 机器人摄像头画面
        self.webcam_image = None    # 电脑摄像头画面
        self.status_text = "Initializing..."
        self.detection_info = ""
        self.current_gesture = "NONE"

IMG_STORE = GlobalImageStore()

class GestureRecognizer:
    def __init__(self):
        self.mp_hands = mp.solutions.hands
        # 降低置信度，提高识别灵敏度
        self.hands = self.mp_hands.Hands(
            max_num_hands=1, 
            min_detection_confidence=0.5, 
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.cap = cv2.VideoCapture(0) # 电脑摄像头

    def update(self):
        ret, frame = self.cap.read()
        if not ret: return None
        
        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        
        detected_num = None
        gesture_text = "No Hand"
        debug_info = ""

        if results.multi_hand_landmarks:
            for hand_lms in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_lms, self.mp_hands.HAND_CONNECTIONS)
                lm = hand_lms.landmark
                
                # === 距离法判断手指伸直 ===
                # 逻辑：指尖到手腕的距离 > 关节到手腕的距离 + 缓冲值
                def is_open(tip_id, pip_id):
                    dist_tip = math.hypot(lm[tip_id].x - lm[0].x, lm[tip_id].y - lm[0].y)
                    dist_pip = math.hypot(lm[pip_id].x - lm[0].x, lm[pip_id].y - lm[0].y)
                    return dist_tip > (dist_pip + 0.02)

                # 拇指判断
                thumb_open = math.hypot(lm[4].x - lm[17].x, lm[4].y - lm[17].y) > 0.15
                
                index_open = is_open(8, 6)
                middle_open = is_open(12, 10)
                ring_open = is_open(16, 14)
                pinky_open = is_open(20, 18)
                
                # 调试信息 (显示在屏幕上)
                debug_info = f"Index:{int(index_open)} Mid:{int(middle_open)} Ring:{int(ring_open)} Pinky:{int(pinky_open)}"
                
                # === 判定逻辑 ===
                # 手势 1: 食指直，中指必须弯 (忽略其他)
                if index_open and not middle_open:
                    if not ring_open and not pinky_open: # 严格模式：其他指头也弯曲
                        detected_num = 1
                        gesture_text = "Gesture: 1 (NAV)"
                
                # 手势 2: 食指直，中指直，无名指弯
                elif index_open and middle_open and not ring_open:
                    detected_num = 2
                    gesture_text = "Gesture: 2 (GRASP)"
                    
                # 手势 5: 全张开
                elif index_open and middle_open and ring_open and pinky_open:
                    gesture_text = "Gesture: 5 (Paper)"
                else:
                    gesture_text = "Unknown"

        # 绘制 UI
        cv2.putText(frame, gesture_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, debug_info, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)

        with IMG_STORE.lock:
            IMG_STORE.webcam_image = frame
            IMG_STORE.current_gesture = str(detected_num) if detected_num else "NONE"
            
        return detected_num

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

# ==================================================================================
#                                3. 机器人控制节点
# ==================================================================================

class CokeFetcher(Node):
    def __init__(self):
        super().__init__('coke_fetcher')
        self.bridge = CvBridge()
        self.cb_group = ReentrantCallbackGroup()
        
        self.declare_parameter('target_area', 5000.0)    
        self.declare_parameter('area_tolerance', 800.0) 
        self.declare_parameter('center_tolerance', 5.0)

        # 视觉伺服 PID
        self.pid_linear = PIDController(kp=0.01, kd=0.005)
        self.pid_angular = PIDController(kp=0.004, kd=0.002)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        
        # 订阅机器人摄像头
        self.image_sub = self.create_subscription(Image, '/front_camera_sensor/image_raw', self.img_callback, qos, callback_group=self.cb_group)
        # 发布速度
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10, callback_group=self.cb_group)
        # 发布初始定位 (虽然nav.launch里有自动发布，这里留作备用)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # 机械臂 Action Clients
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory', callback_group=self.cb_group)
        self.grip_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory', callback_group=self.cb_group)
        self.vacuum_client = self.create_client(SetBool, '/gripper_switch', callback_group=self.cb_group)
        
        self.get_logger().info(">>> 机器人节点启动就绪")

    def stop_robot(self):
        self.vel_pub.publish(Twist())

    def reset_initial_pose(self):
        # 这里的坐标要和 Gazebo 里的出生点一致
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.z = -0.707
        msg.pose.pose.orientation.w = 0.707
        # 协方差矩阵 (简化)
        for i in range(36): msg.pose.covariance[i] = 0.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06
        self.pose_pub.publish(msg)

    def img_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with IMG_STORE.lock:
                IMG_STORE.robot_image = cv_image
        except Exception:
            pass

    def process_vision(self):
        """视觉伺服逻辑：返回 True 表示到达目标"""
        with IMG_STORE.lock:
            if IMG_STORE.robot_image is None: return False
            img = IMG_STORE.robot_image.copy()

        h, w, _ = img.shape
        center_x = w / 2.0
        
        # 红色 HSV 范围 (根据可乐罐调整)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
        mask = mask1 + mask2
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))
        
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        largest_cnt = max(contours, key=cv2.contourArea) if contours else None
        max_area = cv2.contourArea(largest_cnt) if largest_cnt is not None else 0.0

        if max_area > 100:
            x, y, wb, hb = cv2.boundingRect(largest_cnt)
            obj_cx = x + wb/2.0
            
            target_area = self.get_parameter('target_area').value
            err_x = center_x - obj_cx
            
            target_dim = math.sqrt(target_area)
            current_dim = math.sqrt(max_area)
            err_dim = target_dim - current_dim 
            err_area = target_area - max_area 
            
            IMG_STORE.detection_info = f"ErrX:{err_x:.1f} DimErr:{err_dim:.1f}"

            # 角度控制
            ang_vel = self.pid_angular.update(err_x)
            if abs(err_x) > 3.0 and abs(ang_vel) < 0.1:
                ang_vel = 0.1 if ang_vel > 0 else -0.1
            
            twist = Twist()
            twist.angular.z = np.clip(ang_vel, -0.6, 0.6)

            # 距离控制
            lin_vel = self.pid_linear.update(err_dim)
            ALIGNMENT_THRESHOLD = 20.0
            
            if abs(err_x) < ALIGNMENT_THRESHOLD:
                speed_factor = 1.0 - (abs(err_x) / ALIGNMENT_THRESHOLD)
                # 接近目标时减速
                if abs(err_dim) < 5.0:
                    lin_vel = np.clip(lin_vel, -0.03, 0.03)
                else:
                    lin_vel = np.clip(lin_vel, -0.15, 0.2)
                twist.linear.x = lin_vel * speed_factor
            else:
                twist.linear.x = 0.0

            self.vel_pub.publish(twist)

            # 判断是否对准且距离合适
            if abs(err_x) < self.get_parameter('center_tolerance').value and \
               abs(err_area) < self.get_parameter('area_tolerance').value:
                self.stop_robot()
                IMG_STORE.detection_info += " [LOCKED]"
                return True
        else:
            IMG_STORE.detection_info = "Searching..."
            # 原地旋转寻找
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

# ==================================================================================
#                                4. 流程控制与 UI
# ==================================================================================

def wait_for_gesture(gesture_rec, target_num, prompt_text):
    """阻塞循环：等待特定手势"""
    print(f"\n✋ {prompt_text} (等待手势 {target_num})...")
    IMG_STORE.status_text = f"WAITING FOR GESTURE {target_num}"
    
    while True:
        detected = gesture_rec.update()
        
        # 刷新 UI
        with IMG_STORE.lock:
            # 拼接画面：左（机器人） 右（手势）
            if IMG_STORE.robot_image is not None and IMG_STORE.webcam_image is not None:
                r_img = cv2.resize(IMG_STORE.robot_image, (320, 240))
                w_img = cv2.resize(IMG_STORE.webcam_image, (320, 240))
                
                # 提示文字
                color = (0, 255, 0) if detected == target_num else (0, 0, 255)
                cv2.rectangle(w_img, (0,0), (320, 40), (0,0,0), -1)
                cv2.putText(w_img, f"Need: {target_num} | Got: {IMG_STORE.current_gesture}", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                
                combined = np.hstack((r_img, w_img))
                cv2.imshow("Mission Control Center", combined)
                if cv2.waitKey(1) == 27: # ESC 退出
                    exit(0)
        
        if detected == target_num:
            print(f"✅ 检测到手势 {target_num}! 3秒后执行...")
            # 倒计时效果
            for i in range(3, 0, -1):
                print(f"Go in {i}...")
                gesture_rec.update()
                cv2.waitKey(1000)
            break
        
        time.sleep(0.02)

def spin_and_refresh(gesture_rec, duration=0.01):
    """非阻塞刷新：在任务执行期间保持窗口更新"""
    end_time = time.time() + duration
    while time.time() < end_time:
        gesture_rec.update()
        with IMG_STORE.lock:
            if IMG_STORE.robot_image is not None:
                r_img = cv2.resize(IMG_STORE.robot_image, (320, 240))
                if IMG_STORE.webcam_image is not None:
                    w_img = cv2.resize(IMG_STORE.webcam_image, (320, 240))
                    combined = np.hstack((r_img, w_img))
                else:
                    combined = r_img
                
                # 状态栏
                cv2.rectangle(combined, (0, 0), (640, 30), (0, 0, 0), -1)
                cv2.putText(combined, IMG_STORE.status_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cv2.imshow("Mission Control Center", combined)
                cv2.waitKey(1)
        time.sleep(0.02)

def create_pose(navigator, x, y, rz, rw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = navigator.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = float(rz)
    p.pose.orientation.w = float(rw)
    return p

def do_grasp_action(node, gesture_rec):
    IMG_STORE.status_text = "Action: Grasping..."
    # 1. 张开夹爪
    node.set_vacuum(False)
    node.send_gripper(0.02)
    spin_and_refresh(gesture_rec, 1.0)
    
    # 2. 伸出机械臂
    # 请根据你的机械臂关节限制调整这些角度
    node.send_arm([0.0, -0.777, 0.0, 0.828], 3.0)
    spin_and_refresh(gesture_rec, 3.2)
    
    # 3. 吸附/闭合
    node.set_vacuum(True)
    spin_and_refresh(gesture_rec, 0.5)
    node.send_gripper(-0.006)
    spin_and_refresh(gesture_rec, 1.0)
    
    # 4. 收回机械臂
    node.send_arm([0.0, 0.0, 0.0, 0.0], 2.5)
    spin_and_refresh(gesture_rec, 2.7)

def do_drop_action(node, gesture_rec):
    IMG_STORE.status_text = "Action: Dropping..."
    # 1. 伸出
    node.send_arm([0.0, -0.777, 0.0, 0.828], 2.5)
    spin_and_refresh(gesture_rec, 2.7)
    
    # 2. 松开
    node.set_vacuum(False)
    node.send_gripper(0.02)
    spin_and_refresh(gesture_rec, 1.0)
    
    # 3. 收回
    node.send_arm([0.0, 0.0, 0.0, 0.0], 2.0)
    spin_and_refresh(gesture_rec, 2.2)

def main():
    rclpy.init()
    
    node = CokeFetcher()
    navigator = BasicNavigator()
    gesture_rec = GestureRecognizer()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # 初始化等待
    node.reset_initial_pose()
    spin_and_refresh(gesture_rec, 2.0)

    try:
        # ==================== 任务循环 ====================
        
        # --- 任务 1 ---
        wait_for_gesture(gesture_rec, 1, "请比 '1' 开始去第一个点")
        
        print(">>> [Task 1] 导航去抓取点...")
        IMG_STORE.status_text = "Nav: To Pickup 1"
        navigator.clearAllCostmaps()
        goal_pose = create_pose(navigator, PICKUP_1_X, PICKUP_1_Y, PICKUP_1_RZ, PICKUP_1_RW)
        navigator.goToPose(goal_pose)
        
        while not navigator.isTaskComplete():
            spin_and_refresh(gesture_rec, 0.1)
        node.stop_robot()

        wait_for_gesture(gesture_rec, 2, "到达! 请比 '2' 开始视觉抓取")
        
        print(">>> [Task 1] 视觉抓取...")
        IMG_STORE.status_text = "Visual: Searching..."
        while True:
            is_locked = node.process_vision()
            spin_and_refresh(gesture_rec, 0.05)
            if is_locked: break
        
        do_grasp_action(node, gesture_rec)

        print(">>> [Task 1] 去卸货点...")
        IMG_STORE.status_text = "Nav: To Drop"
        navigator.clearAllCostmaps() 
        wps = [create_pose(navigator, *wp) for wp in WAYPOINTS_LIST]
        wps.append(create_pose(navigator, DROP_X, DROP_Y, DROP_RZ, DROP_RW))
        navigator.followWaypoints(wps)
        while not navigator.isTaskComplete():
            spin_and_refresh(gesture_rec, 0.1)
        node.stop_robot()
        
        do_drop_action(node, gesture_rec)

        # --- 任务 2 ---
        wait_for_gesture(gesture_rec, 1, "卸货完毕。请比 '1' 去第二个点")

        print(">>> [Task 2] 导航去抓取点...")
        IMG_STORE.status_text = "Nav: To Pickup 2"
        navigator.clearAllCostmaps() 
        transit_wps = [create_pose(navigator, *wp) for wp in WAYPOINTS_TRANSIT_LIST]
        transit_wps.append(create_pose(navigator, PICKUP_2_X, PICKUP_2_Y, PICKUP_2_RZ, PICKUP_2_RW))
        navigator.followWaypoints(transit_wps)
        while not navigator.isTaskComplete():
            spin_and_refresh(gesture_rec, 0.1)
        node.stop_robot()

        wait_for_gesture(gesture_rec, 2, "到达! 请比 '2' 开始视觉抓取")

        print(">>> [Task 2] 视觉抓取...")
        IMG_STORE.status_text = "Visual: Searching..."
        while True:
            is_locked = node.process_vision()
            spin_and_refresh(gesture_rec, 0.05)
            if is_locked: break
        
        do_grasp_action(node, gesture_rec)

        print(">>> [Task 2] 返回卸货点...")
        IMG_STORE.status_text = "Nav: Return"
        navigator.clearAllCostmaps() 
        wps2 = [create_pose(navigator, *wp) for wp in WAYPOINTS_2_LIST]
        wps2.append(create_pose(navigator, DROP_X, DROP_Y, DROP_RZ, DROP_RW))
        navigator.followWaypoints(wps2)
        while not navigator.isTaskComplete():
            spin_and_refresh(gesture_rec, 0.1)
        node.stop_robot()

        do_drop_action(node, gesture_rec)

        IMG_STORE.status_text = "ALL DONE!"
        print(">>> 全部任务完成！")
        spin_and_refresh(gesture_rec, 5.0)

    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        gesture_rec.cap.release()

if __name__ == '__main__':
    main()
