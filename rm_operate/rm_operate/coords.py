#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient


class MoveToCoords(Node):
    def __init__(self):
        super().__init__("move_to_coords")

        self.client = ActionClient(self, MoveGroup, "move_action")

        # run once after startup
        self.timer = self.create_timer(1.0, self.send_goal)

    def send_goal(self):
        self.timer.cancel()

        x = float(input("Enter X (meters): "))
        y = float(input("Enter Y (meters): "))
        z = float(input("Enter Z (meters): "))

        # ---- Target pose ----
        pose = PoseStamped()
        pose.header.frame_id = "body_link"   # from your SRDF
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0       # neutral orientation

        # ---- Build goal ----
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2

        constraints = Constraints()

        # ---- Position constraint ----
        pc = PositionConstraint()
        pc.header.frame_id = "body_link"
        pc.link_name = "end_link"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.001, 0.001, 0.001]

        pc.constraint_region.primitives.append(box)
        pc.constraint_region.primitive_poses.append(pose.pose)
        pc.weight = 1.0

        # ---- Orientation constraint (IMPORTANT) ----
        oc = OrientationConstraint()
        oc.header.frame_id = "body_link"
        oc.link_name = "end_link"
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.2
        oc.absolute_y_axis_tolerance = 0.2
        oc.absolute_z_axis_tolerance = 0.2
        oc.weight = 1.0

        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)

        goal.request.goal_constraints.append(constraints)

        # ---- Send to MoveIt ----
        self.client.wait_for_server()
        self.get_logger().info("Sending XYZ goal to MoveIt...")
        self.client.send_goal_async(goal)


def main():
    rclpy.init()
    node = MoveToCoords()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
