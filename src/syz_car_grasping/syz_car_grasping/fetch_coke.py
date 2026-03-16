#!/usr/bin/env python3
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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# ================= 坐标配置 =================

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

# --- [新增] 过渡路径: 从 卸货点1 -> 抓取点2 的途径点 ---
# 请根据你的地图实际情况修改这里的坐标 (x, y, z, w)
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
# ============================================

class GlobalImageStore:
    def __init__(self):
        self.lock = threading.Lock()
        self.image = None
        self.status_text = "Initializing..."
        self.detection_info = ""

IMG_STORE = GlobalImageStore()

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

class CokeFetcher(Node):
    def __init__(self):
        super().__init__('coke_fetcher')
        self.bridge = CvBridge()
        self.cb_group = ReentrantCallbackGroup()
        
        self.declare_parameter('target_area', 5000.0)    
        self.declare_parameter('area_tolerance', 800.0) 
        self.declare_parameter('center_tolerance', 5.0)

        # PID 参数
        self.pid_linear = PIDController(kp=0.01, kd=0.005)
        self.pid_angular = PIDController(kp=0.004, kd=0.002)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.image_sub = self.create_subscription(Image, '/front_camera_sensor/image_raw', self.img_callback, qos, callback_group=self.cb_group)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10, callback_group=self.cb_group)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory', callback_group=self.cb_group)
        self.grip_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory', callback_group=self.cb_group)
        self.vacuum_client = self.create_client(SetBool, '/gripper_switch', callback_group=self.cb_group)
        
        self.get_logger().info(">>> 节点启动")

    def stop_robot(self):
        self.vel_pub.publish(Twist())

    def reset_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[35] = 0.05
        self.pose_pub.publish(msg)
        self.get_logger().info(">>> 重置定位")

    def img_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with IMG_STORE.lock:
                IMG_STORE.image = cv_image
        except Exception:
            pass

    def process_vision(self):
        with IMG_STORE.lock:
            if IMG_STORE.image is None: return False
            img = IMG_STORE.image.copy()

        h, w, _ = img.shape
        total_pixels = w * h
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
            
            target_dim = math.sqrt(target_area)
            current_dim = math.sqrt(max_area)
            err_dim = target_dim - current_dim 
            err_area = target_area - max_area 
            
            IMG_STORE.detection_info = f"ErrX:{err_x:.1f} DimErr:{err_dim:.1f}"

            # PID 控制
            ang_vel = self.pid_angular.update(err_x)
            
            if abs(err_x) > 3.0 and abs(ang_vel) < 0.1:
                ang_vel = 0.1 if ang_vel > 0 else -0.1
            
            twist = Twist()
            twist.angular.z = np.clip(ang_vel, -0.6, 0.6)

            lin_vel = self.pid_linear.update(err_dim)
            ALIGNMENT_THRESHOLD = 20.0
            
            if abs(err_x) < ALIGNMENT_THRESHOLD:
                speed_factor = 1.0 - (abs(err_x) / ALIGNMENT_THRESHOLD)
                if abs(err_dim) < 5.0:
                    lin_vel = np.clip(lin_vel, -0.03, 0.03)
                else:
                    lin_vel = np.clip(lin_vel, -0.15, 0.2)
                twist.linear.x = lin_vel * speed_factor
            else:
                twist.linear.x = 0.0

            self.vel_pub.publish(twist)

            if abs(err_x) < self.get_parameter('center_tolerance').value and \
               abs(err_area) < self.get_parameter('area_tolerance').value:
                self.stop_robot()
                IMG_STORE.detection_info += " [LOCKED]"
                return True
        else:
            IMG_STORE.detection_info = "Searching..."
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

def spin_and_refresh(duration=0.01):
    end_time = time.time() + duration
    while time.time() < end_time:
        with IMG_STORE.lock:
            if IMG_STORE.image is not None:
                display_img = IMG_STORE.image.copy()
                h, w, _ = display_img.shape
                cv2.rectangle(display_img, (0, 0), (w, 80), (0, 0, 0), -1)
                cv2.putText(display_img, IMG_STORE.status_text, (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.putText(display_img, IMG_STORE.detection_info, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
                cx, cy = w // 2, h // 2
                cv2.line(display_img, (cx-20, cy), (cx+20, cy), (0,0,255), 2)
                cv2.line(display_img, (cx, cy-20), (cx, cy+20), (0,0,255), 2)
                cv2.imshow("Robot Monitor (Safe)", display_img)
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

def do_grasp_action(node):
    IMG_STORE.status_text = "Action: Grasping..."
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

def do_drop_action(node):
    IMG_STORE.status_text = "Action: Dropping..."
    node.send_arm([0.0, -0.777, 0.0, 0.828], 2.5)
    spin_and_refresh(2.7)
    
    node.set_vacuum(False)
    node.send_gripper(0.02)
    spin_and_refresh(1.0)
    
    node.send_arm([0.0, 0.0, 0.0, 0.0], 2.0)
    spin_and_refresh(2.2)

def main():
    rclpy.init()
    node = CokeFetcher()
    navigator = BasicNavigator()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.reset_initial_pose()
    spin_and_refresh(2.0)

    try:
        # Task 1
        print(">>> [Task 1] 导航去抓取点...")
        IMG_STORE.status_text = "Nav: To Pickup 1"
        navigator.clearAllCostmaps() 
        spin_and_refresh(0.5)

        navigator.goToPose(create_pose(navigator, PICKUP_1_X, PICKUP_1_Y, PICKUP_1_RZ, PICKUP_1_RW))
        while not navigator.isTaskComplete():
            spin_and_refresh(0.1)
        node.stop_robot()

        print(">>> [Task 1] 视觉抓取...")
        IMG_STORE.status_text = "Visual: Searching..."
        while True:
            is_locked = node.process_vision()
            spin_and_refresh(0.05)
            if is_locked: break
        
        do_grasp_action(node)

        print(">>> [Task 1] 去卸货点...")
        IMG_STORE.status_text = "Nav: To Drop (Waypoints)"
        navigator.clearAllCostmaps() 
        wps = [create_pose(navigator, *wp) for wp in WAYPOINTS_LIST]
        wps.append(create_pose(navigator, DROP_X, DROP_Y, DROP_RZ, DROP_RW))
        navigator.followWaypoints(wps)
        while not navigator.isTaskComplete():
            spin_and_refresh(0.1)
        node.stop_robot()
        
        do_drop_action(node)

        # ========================================================
        # Task 2 (此处已修改为使用途径点)
        # ========================================================
        print(">>> [Task 2] 导航去抓取点 (途径点模式)...")
        IMG_STORE.status_text = "Nav: To Pickup 2 (Transiting)"
        navigator.clearAllCostmaps() 
        
        # 1. 创建过渡点列表
        transit_wps = [create_pose(navigator, *wp) for wp in WAYPOINTS_TRANSIT_LIST]
        # 2. 将最终抓取点作为列表的最后一站
        transit_wps.append(create_pose(navigator, PICKUP_2_X, PICKUP_2_Y, PICKUP_2_RZ, PICKUP_2_RW))
        
        # 3. 执行跟随路径点
        navigator.followWaypoints(transit_wps)
        while not navigator.isTaskComplete():
            spin_and_refresh(0.1)
        node.stop_robot()

        print(">>> [Task 2] 视觉抓取...")
        IMG_STORE.status_text = "Visual: Searching..."
        while True:
            is_locked = node.process_vision()
            spin_and_refresh(0.05)
            if is_locked: break
        
        do_grasp_action(node)

        print(">>> [Task 2] 返回卸货点...")
        IMG_STORE.status_text = "Nav: Return (New WPs)"
        navigator.clearAllCostmaps() 
        
        wps2 = [create_pose(navigator, *wp) for wp in WAYPOINTS_2_LIST]
        wps2.append(create_pose(navigator, DROP_X, DROP_Y, DROP_RZ, DROP_RW))
        
        navigator.followWaypoints(wps2)
        while not navigator.isTaskComplete():
            spin_and_refresh(0.1)
        node.stop_robot()

        do_drop_action(node)

        IMG_STORE.status_text = "ALL DONE!"
        spin_and_refresh(5.0)

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
