#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
import time

class PickPlaceController(Node):
    def __init__(self):
        super().__init__('pick_place_controller')

        # Action client to your arm controller
        self._arm_client = ActionClient(self, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')
        # If you have a gripper controller:
        self._gripper_client = ActionClient(self, FollowJointTrajectory, 'gripper_controller/follow_joint_trajectory')

        # Joint names
        self.arm_joints = ['swivel_link_joint','link1_link_joint','link2_link_joint','end_link_joint']
        self.gripper_joints = ['gripper_finger_joint']  # adjust to your robot

    # -------------------- Helper to send trajectory --------------------
    def send_joint_goal(self, client: ActionClient, joint_names, positions, duration_sec=3.0):
        client.wait_for_server()
        traj = JointTrajectory()
        traj.joint_names = joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration_sec)
        traj.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj
        future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

    # -------------------- Arm functions --------------------
    def move_arm(self, positions, duration=3.0):
        self.send_joint_goal(self._arm_client, self.arm_joints, positions, duration)

    # -------------------- Gripper functions --------------------
    def open_gripper(self):
        self.send_joint_goal(self._gripper_client, self.gripper_joints, [0.04], duration_sec=1.0)  # adjust open width

    def close_gripper(self):
        self.send_joint_goal(self._gripper_client, self.gripper_joints, [0.0], duration_sec=1.0)  # closed

    # -------------------- Pick and Place --------------------
    def pick(self, approach_pos, grasp_pos):
        # Approach above object
        self.move_arm(approach_pos, duration=2)
        time.sleep(0.5)

        # Move down to grasp
        self.move_arm(grasp_pos, duration=2)
        time.sleep(0.5)

        # Close gripper
        self.close_gripper()
        time.sleep(0.5)

        # Lift object back to approach
        self.move_arm(approach_pos, duration=2)
        time.sleep(0.5)

    def place(self, approach_pos, place_pos):
        # Approach above place location
        self.move_arm(approach_pos, duration=2)
        time.sleep(0.5)

        # Move down to place
        self.move_arm(place_pos, duration=2)
        time.sleep(0.5)

        # Open gripper
        self.open_gripper()
        time.sleep(0.5)

        # Retreat back to approach
        self.move_arm(approach_pos, duration=2)
        time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceController()

    # -------------------- Define positions --------------------
    # Joint angles for pick: [swivel, link1, link2, end_link]
    pick_approach = [0.5, 0.0, 0.0, 0.0]  # above object
    pick_grasp = [0.5, -0.2, 0.0, 0.0]    # at object

    place_approach = [0.0, 0.3, 0.0, 0.0]  # above place
    place_joint = [0.0, 0.0, -0.2, 0.0]    # at place

    # -------------------- Execute pick and place --------------------
    node.pick(pick_approach, pick_grasp)
    node.place(place_approach, place_joint)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
