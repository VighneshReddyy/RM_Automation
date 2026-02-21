#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, Constraints, JointConstraint

class FinalMoveNode(Node):
    def __init__(self):
        super().__init__('final_move_coords')

        # 1. Action Client for execution
        self.move_client = ActionClient(self, MoveGroup, 'move_action')
        
        # 2. Service Client for IK math
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')

        self.get_logger().info("Connecting to MoveIt...")
        self.ik_client.wait_for_service()
        self.move_client.wait_for_server()

        # TEST COORDINATES (Adjust based on your tf2_echo)
        # We move 7cm forward from your current -0.52 position
        self.target_x = -0.45 
        self.target_y = 0.0
        self.target_z = 0.75

        self.run_sequence()

    def run_sequence(self):
        self.get_logger().info(f"Targeting: x={self.target_x}, y={self.target_y}, z={self.target_z}")
        
        # Build IK Request
        request = GetPositionIK.Request()
        req = PositionIKRequest()
        req.group_name = "arm"
        req.avoid_collisions = True
        req.ik_link_name = "bevel_link_link"
        req.pose_stamped.header.frame_id = "body_link"
        req.pose_stamped.pose.position.x = self.target_x
        req.pose_stamped.pose.position.y = self.target_y
        req.pose_stamped.pose.position.z = self.target_z
        
        # Pointing the gripper "down" (standard for pick and place)
        # Using a neutral orientation first
        req.pose_stamped.pose.orientation.w = 1.0 

        request.ik_request = req
        
        self.get_logger().info("Solving IK...")
        future = self.ik_client.call_async(request)
        future.add_done_callback(self.ik_result_callback)

    def ik_result_callback(self, future):
        res = future.result()
        if res.error_code.val != 1:
            self.get_logger().error(f"IK Failed (Code {res.error_code.val}). The spot is unreachable.")
            return

        self.get_logger().info("IK Success! Moving arm...")
        
        # Convert IK solution to Joint Constraints for MoveGroup
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        constraints = Constraints()

        # Filter and map joints from IK solution
        for i, name in enumerate(res.solution.joint_state.name):
            if name in ['swivel', 'link1', 'link2', 'bevel_roll', 'bevel_pitch']:
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = res.solution.joint_state.position[i]
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(constraints)
        self.move_client.send_goal_async(goal)

def main():
    rclpy.init()
    node = FinalMoveNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()