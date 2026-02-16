import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive


class MoveToCoords(Node):
    def __init__(self):
        super().__init__('move_to_coords')

        self.client = ActionClient(self, MoveGroup, 'move_action')

        self.get_logger().info("Waiting for MoveGroup action server...")
        self.client.wait_for_server()

        goal_msg = MoveGroup.Goal()

        # ---------------- Planning request ----------------
        goal_msg.request = MotionPlanRequest()
        goal_msg.request.group_name = "arm"
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0

        # ---------------- Target pose ----------------
        pose = PoseStamped()
        pose.header.frame_id = "body_link"
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.3
        pose.pose.orientation.w = 1.0

        # ---------------- Position constraint ----------------
        pc = PositionConstraint()
        pc.header.frame_id = "body_link"
        pc.link_name = "bevel_link.001"

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 1 cm tolerance

        bv = BoundingVolume()
        bv.primitives.append(sphere)
        bv.primitive_poses.append(pose.pose)

        pc.constraint_region = bv
        pc.weight = 1.0

        constraint = Constraints()
        constraint.position_constraints.append(pc)

        goal_msg.request.goal_constraints.append(constraint)

        # 🔥 THIS IS THE MISSING PART 🔥
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = True

        self.send_goal(goal_msg)

    def send_goal(self, goal_msg):
        self.get_logger().info("Sending goal...")
        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info("Motion finished")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = MoveToCoords()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
