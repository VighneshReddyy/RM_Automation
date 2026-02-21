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

        self._arm_client = ActionClient(
            self, FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory'
        )

        self._gripper_client = ActionClient(
            self, FollowJointTrajectory,
            'gripper_controller/follow_joint_trajectory'
        )

        self.arm_joints = [
            'swivel_link_joint',
            'link1_link_joint',
            'link2_link_joint',
            'end_link_joint'
        ]
        self.gripper_joints = ['gripper_finger_joint']

        l1=float(input("enter link1 :"))
        l2=float(input("enter link2 :"))
        swivel=float(input("enter swivel :"))
        self.pick_approach = [swivel, l1, l2, 0.0]
#        self.pick_grasp    = [0.0, -0.2, 0.5, 0.0]
        self.place_approach = [0.0, 0.0, 0.0, 0.0]
#        self.place_joint    = [0.0, 0.0, -0.2, 0.0]

        # Run once after startup
        self.timer = self.create_timer(1.0, self.run_sequence)
        self.ran = False

    def send_joint_goal(self, client, joint_names, positions, duration_sec=2.0):
        client.wait_for_server()

        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration_sec)
        traj.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        # goal it
        send_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("Reached target")

        # this to move the rm
    def move_arm(self, positions, duration=5.0):
        self.send_joint_goal(self._arm_client, self.arm_joints, positions, duration)




    
    def pick(self):
        self.move_arm(self.pick_approach)
        time.sleep(0.5)
        

    def place(self):
        self.move_arm(self.place_approach)
        time.sleep(0.5)


    def run_sequence(self):
        if self.ran:
            return

        self.get_logger().info("Starting pick and place")

        self.pick()
        self.place()

        self.get_logger().info("Pick and place done")
        self.ran = True


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
