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

        self.arm_joints = [
            'swivel_link_joint',
            'link1_link_joint',
            'link2_link_joint',
            'end_link_joint'
        ]

        l1 = float(input("enter link1 :"))
        l2 = float(input("enter link2 :"))
        swivel = float(input("enter swivel :"))

        self.pick_approach = [swivel, l1, l2, 0.0]
        self.place_approach = [0.0, 0.0, 0.0, 0.0]

        # run once after startup
        self.timer = self.create_timer(0.5, self.start_once)
        self.started = False


    def start_once(self):
        if self.started:
            return
        self.started = True
        self.get_logger().info("Starting pick motion")
        self.send_joint_goal(self.pick_approach, self.pick_done_cb)


    def send_joint_goal(self, positions, done_cb, duration_sec=3):
        self._arm_client.wait_for_server()

        traj = JointTrajectory()
        traj.joint_names = self.arm_joints

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = duration_sec
        traj.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        send_future = self._arm_client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda f: self.goal_response_cb(f, done_cb)
        )


    def goal_response_cb(self, future, done_cb):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted, executing...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(done_cb)


    def pick_done_cb(self, future):
        result = future.result().result

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Pick  finished")
            self.get_logger().info("Starting place motion")
            self.send_joint_goal(self.place_approach, self.place_done_cb)
        else:
            self.get_logger().error(f"pick failed: {result.error_code}")


    def place_done_cb(self, future):
        result = future.result().result

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Place motion finished")
        else:
            self.get_logger().error(f"Place failed: {result.error_code}")

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
