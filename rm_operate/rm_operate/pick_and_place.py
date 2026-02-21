#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PickPlaceController(Node):
    def __init__(self):
        super().__init__('pick_place_controller')

        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory'
        )

        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            'gripper_controller/follow_joint_trajectory'
        )

        # MUST match ros2_control yaml
        self.arm_joints = ['swivel', 'link1', 'link2', 'bevel_roll', 'bevel_pitch']
        self.gripper_joints = ['gripper_l', 'gripper_r']

        self.arm_states = {
            "home":   [0.0, 0.0, 0.0, 0.0, 0.0],
            "slu":    [0.0, -0.6116, 0.3381, 0.0, 0.0],
        }

        self.gripper_states = {
            "open":  [0.0, 0.0],
            "close": [0.0513, 0.0526],
        }

        self.state = "START"
        self.timer = self.create_timer(1.0, self.state_machine)

    # ---------------- ACTION SENDERS ----------------

    def send_arm_goal(self, state_name):
        self.get_logger().info(f"Sending ARM -> {state_name}")
        self.send_goal(self.arm_client, self.arm_joints, self.arm_states[state_name])

    def send_gripper_goal(self, state_name):
        self.get_logger().info(f"Sending GRIPPER -> {state_name}")
        self.send_goal(self.gripper_client, self.gripper_joints, self.gripper_states[state_name])

    def send_goal(self, client, joint_names, positions, duration=2.0):
        client.wait_for_server()

        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        traj.points.append(point)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        send_future = client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)



    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Motion finished")


        self.advance_state()

    # -multiplexer <3

    def state_machine(self):
        if self.state == "START":
            self.send_gripper_goal("open")
            self.state = "WAIT_OPEN"

    def advance_state(self):
        if self.state == "WAIT_OPEN":
            self.send_arm_goal("home")
            self.state = "WAIT_HOME"

        elif self.state == "WAIT_HOME":
            self.send_arm_goal("slu")
            self.state = "WAIT_SLU"

        elif self.state == "WAIT_SLU":
            self.send_gripper_goal("close")
            self.state = "WAIT_CLOSE"

        elif self.state == "WAIT_CLOSE":
            self.send_arm_goal("home")
            self.state = "WAIT_RETURN"

        elif self.state == "WAIT_RETURN":
            self.send_gripper_goal("open")
            self.state = "DONE"

        elif self.state == "DONE":
            self.get_logger().info("Pick and place DONE ✅")
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
