#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class ArmDebugger(Node):
    def __init__(self):
        super().__init__('arm_debugger')
        self.client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.grip_client = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

    def send(self, j1, j2, j3, j4):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        pt = JointTrajectoryPoint()
        pt.positions = [float(j1), float(j2), float(j3), float(j4)]
        pt.time_from_start.sec = 1
        goal.trajectory.points = [pt]
        self.client.wait_for_server()
        self.client.send_goal_async(goal)

    def gripper(self, val):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['paw_joint_1']
        pt = JointTrajectoryPoint()
        pt.positions = [float(val)] 
        pt.time_from_start.sec = 1
        goal.trajectory.points = [pt]
        self.grip_client.wait_for_server()
        self.grip_client.send_goal_async(goal)

def main():
    rclpy.init()
    node = ArmDebugger()
    print("请输入4个角度 (例如: 0 0.5 0.5 -0.5)")
    print("输入 g -0.05 张开爪子")
    while True:
        s = input("指令 > ")
        try:
            if s.startswith('g'): node.gripper(s.split()[1])
            else: 
                v = s.split()
                node.send(v[0], v[1], v[2], v[3])
        except: pass

if __name__ == '__main__':
    main()
