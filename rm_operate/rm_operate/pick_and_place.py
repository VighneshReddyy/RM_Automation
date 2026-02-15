#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_py import MoveItPy
from geometry_msgs.msg import Pose


class PickPlace(Node):
    def __init__(self):
        super().__init__("pick_place_node")

        self.moveit = MoveItPy(node_name="moveit_py_node")
        self.arm = self.moveit.get_planning_component("arm")

        self.pick()

    def pick(self):
        pose = Pose()
        pose.position.x = 0.3
        pose.position.y = 0.0
        pose.position.z = 0.2
        pose.orientation.w = 1.0

        self.arm.set_goal_state(pose_stamped_msg=pose, pose_link="tool0")

        plan = self.arm.plan()
        if plan:
            self.arm.execute()


def main():
    rclpy.init()
    node = PickPlace()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
