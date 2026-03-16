#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import cv2
import mediapipe as mp
import math

class GestureControlNode(Node):
    def __init__(self):
        super().__init__('gesture_control')
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 初始化手势库
        self.mp_hands = mp.solutions.hands
        # min_detection_confidence=0.5: 调低一点，让它更容易发现手
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_draw = mp.solutions.drawing_utils
        
        self.cap = cv2.VideoCapture(0)
        self.timer = self.create_timer(0.05, self.process_frame) # 20FPS
        self.get_logger().info(">>> 手势控制启动：请比划数字 1, 2, 3, 4, 5 <<<")

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret: return

        # 1. 镜像并转色
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)
        
        finger_count = 0
        status_text = "No Hand"

        if results.multi_hand_landmarks:
            for lm in results.multi_hand_landmarks:
                # 画骨架
                self.mp_draw.draw_landmarks(frame, lm, self.mp_hands.HAND_CONNECTIONS)
                
                # === 核心算法：通用数手指 ===
                # 获取关键点坐标
                l = lm.landmark
                # 4=拇指尖, 8=食指尖, 12=中指尖, 16=无名指尖, 20=小指尖
                # 关节: 2, 6, 10, 14, 18
                # 掌根: 0
                
                # 计算两点距离函数
                def get_dist(idx1, idx2):
                    return math.hypot(l[idx1].x - l[idx2].x, l[idx1].y - l[idx2].y)

                # 判断手指伸直：指尖到掌根的距离 > 关节到掌根的距离 * 1.1
                # 这种方法不管手怎么旋转都有效
                cnt = 0
                # 拇指 (4 vs 2)
                if get_dist(4, 0) > get_dist(2, 0) * 1.1: cnt += 1
                # 食指 (8 vs 6)
                if get_dist(8, 0) > get_dist(6, 0) * 1.1: cnt += 1
                # 中指 (12 vs 10)
                if get_dist(12, 0) > get_dist(10, 0) * 1.1: cnt += 1
                # 无名指 (16 vs 14)
                if get_dist(16, 0) > get_dist(14, 0) * 1.1: cnt += 1
                # 小指 (20 vs 18)
                if get_dist(20, 0) > get_dist(18, 0) * 1.1: cnt += 1
                
                finger_count = cnt
                status_text = f"Count: {cnt}"

                # === 简单的控制逻辑示例 ===
                twist = Twist()
                if cnt == 1: # 食指 -> 前进
                    status_text += " (GO)"
                    twist.linear.x = 0.2
                elif cnt == 2: # 剪刀手 -> 后退
                    status_text += " (BACK)"
                    twist.linear.x = -0.2
                elif cnt == 3: # OK手势/三指 -> 左转
                    status_text += " (LEFT)"
                    twist.angular.z = 0.5
                elif cnt == 4: # 四指 -> 右转
                    status_text += " (RIGHT)"
                    twist.angular.z = -0.5
                elif cnt == 5: # 五指张开 -> 停止
                    status_text += " (STOP)"
                    # 发送0速
                elif cnt == 0: # 握拳 -> 停止
                    status_text += " (STOP)"
                
                # 发布速度
                self.pub_vel.publish(twist)

        # UI显示
        color = (0, 255, 0) if finger_count > 0 else (0, 0, 255)
        cv2.putText(frame, status_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 3)
        
        cv2.imshow("Hand Control", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = GestureControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # === 修复报错的关键：在这里处理退出 ===
        print("\n检测到退出信号，正在刹车...")
        # 此时 ROS 还是活着的，可以发布消息
        stop_msg = Twist()
        node.pub_vel.publish(stop_msg)
    finally:
        # 销毁节点
        node.destroy_node()
        # 关闭 ROS
        if rclpy.ok():
            rclpy.shutdown()
        # 关闭窗口
        cv2.destroyAllWindows()
        print("程序已退出。")

if __name__ == '__main__':
    main()
