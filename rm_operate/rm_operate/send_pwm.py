#!/usr/bin/env python3
import rclpy
import math
import socket

from rclpy.node import Node
from sensor_msgs.msg import JointState
from custom_msgs.msg import ArmPosition

class ArmStateSubscriber(Node):
    def __init__(self):
        super().__init__('arm_state_subscriber')
        self.send_string="M0L0R0T0U0G0Z0E"
        self.ref_string=self.send_string
        self.udp_ip = "10.0.0.7"
        self.udp_port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.virtualarm = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        self.realArm = self.create_subscription(
            ArmPosition,
            "/real/arm_position",
            self.arm_callback,
            10
        )
        self.timer = self.create_timer(0.0025, self.send_pwm)
        self.real_l1=0.0
        self.real_l2=0.0
        self.real_swivel=0.0
        self.sim_l1=0.0
        self.sim_l2=0.0
        self.sim_swivel=0.0

        self.pwm_l1=0
        self.pwm_l2=0
        self.pwm_swivel=0

#        self.get_logger().info("Subscribed to /real/arm_position")
    def rad_to_deg(self,rad):
        return rad * 180.0 / math.pi


    def joint_callback(self, msg: JointState):
        # Print joint names and positions
        for name, pos in zip(msg.name, msg.position):
            match name: 
                case "swivel_link_joint":
                    self.sim_swivel=round(self.rad_to_deg(pos),2)
                case "link1_link_joint":
                    self.sim_l1=round(self.rad_to_deg(pos),2)
                case "link2_link_joint":
                    self.sim_l2=round(self.rad_to_deg(pos),2)
                

            #self.get_logger().info(f"{name} : {pos:.3f}")

        # Or print as a list
        # self.get_logger().info(str(msg.position))
    def arm_callback(self, msg:ArmPosition):
        #print(msg.link1,msg.link2,msg.swivel)
        self.real_l1=round(msg.link1,2)
        self.real_l2=round(msg.link2,2)
        self.real_swivel=round(msg.swivel,2)


    def send_pwm(self):
        print("yes")
        print(self.real_l1,self.real_l2,self.real_swivel)
        print(self.sim_l1,self.sim_l2,self.sim_swivel)
        self.pwm_l1=0
        self.pwm_l2=0

        if(abs(self.sim_l1-self.real_l1)>1):
        
            self.pwm_l1=self.sim_l1-self.real_l1W
        
        if(abs(self.sim_l2-self.real_l2)>1):
        
            self.pwm_l2=self.sim_l2-self.real_l2
        
        self.send_string=self.ref_string




        if(self.pwm_l1>0):
            self.send_string=self.send_string.replace("U0","U-1") #down
        elif self.pwm_l1<0:
            self.send_string=self.send_string.replace("U0","U1")#up

        if(self.pwm_l2>0):
            self.send_string=self.send_string.replace("T0","T-1") #down
        elif self.pwm_l2<0:
            self.send_string=self.send_string.replace("T0","T1")#up
        
        self.sock.sendto(self.send_string.encode(), (self.udp_ip, self.udp_port))

        print("pwm ",self.pwm_l1,self.pwm_l2,"\n",self.send_string)

        




def main(args=None):
    rclpy.init(args=args)
    node = ArmStateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
