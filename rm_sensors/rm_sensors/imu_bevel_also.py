import rclpy
from rclpy.node import Node
from custom_msgs.msg import ImuData
from geometry_msgs.msg import Vector3
from custom_msgs.msg import ArmPwm
from custom_msgs.msg import ArmPosition
from std_msgs.msg import Bool

import socket
import time
import pickle
import os
import serial
def safe_float(val, default=None):
    try:
        return float(val)
    except (ValueError, TypeError):
        return default

class BNO055Node(Node):
    def __init__(self):
        super().__init__('rm_auto')

##### filler
        self.ip = "10.0.0.7"
        self.port = 5005
        self.sock = None
        self.server_addr = (self.ip, self.port)
        self.initialize_sock()
        self.msgR="L0R0T0U0E|Z1"
        self.msg="L0R0T0U0E|Z1"
        self.p=0
#####
        self.deliveryNow=False

        self.l1=dict({"r":400.0,"p":400.0,"y":400.0})
        self.l2=dict({"r":400.0,"p":400.0,"y":400.0})
        self.bevel=dict({"r":400.0,"p":400.0,"y":400.0})
        #self.imu_data=dict({"r":400.0,"p":400.0,"y":400.0})
        self.toAnglel1=90
        self.toAnglel2=180
        self.rvYaw=400
        self.rvRoll=400
        self.rvPitch=400


        self.link1Pwm=0
        self.link2Pwm=0
#        self.l2=dict()
        self.line="123"
        
        self.imu_pub_ = self.create_publisher(ImuData, '/external_imu', 10)

        self.delivery_sub_ = self.create_subscription(Bool, '/deliver_now',self.delivery_callback, 10)
        
        self.armPwm_ =self.create_publisher(ArmPwm,'/arm_pwm',10)
        self.armpos=self.create_publisher(ArmPosition,"/real/arm_position",10)
        
        self.delivered_pub_=self.create_publisher(Bool,"/delivered",10)

        self.initialize_imu()
        self.timer_ = self.create_timer(0.0025, self.publish_imu)
#        self.timer_ = self.create_timer(0.01, self.send)


        time.sleep(1)
        if(self.serial_port):
            print("into the serial port1 saf asf")
        else:
            print("serial port issue")
        self.serial_port.dtr = False   # GPIO0 HIGH
        self.serial_port.rts = True    # EN LOW (reset)
        time.sleep(0.1)

        self.serial_port.rts = False   # EN HIGH (run)
        time.sleep(0.1)
        self.serial_port.dtr = True 
        self.serial_port.close()
        self.initialize_imu()
        time.sleep(1)
        if(self.serial_port):
            print("into the serial port reseted")
        else:
            print("serial port issue")

    
    def initialize_imu(self):
        #self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)
        self.serial_port = serial.Serial('/dev/ttyUSB0',1000000,timeout=0.0025 )

    def delivery_callback(self,msg):
        if(msg.data):
            self.deliveryNow=True
            print(msg.data)
        else:
            self.deliveryNow=False
            print(msg.data)
    def initialize_sock(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print("UDP socket initialized")
        except socket.error as e:
            print("Socket error:", e)
            self.sock = None
        
    def send(self):
        print(self.msg)
        if self.sock is None:
            print("Socket not initialized")
            return

        try:
            data = self.msg.encode("utf-8")
            self.sock.sendto(data, self.server_addr)
        except socket.error as e:
            print("Send error:", e)




    def publish_imu(self):
        try:
            
            line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            if not line.startswith("yl1"):
                print("error is error ",line)
                return

            z = line.split('_')
            if len(z) != 11:
                print("trash IMU packet:", line)
                return

            # Link 1
            self.l1["y"] = safe_float(z[1])
            self.l1["r"] = safe_float(z[2])
            self.l1["p"] = (-safe_float(z[3])) % 360

            # Link 2
            self.l2["y"] = safe_float(z[5])
            self.l2["r"] = safe_float(z[6])
            self.l2["p"] = safe_float(z[7]) % 360

            self.bevel["y"] = safe_float(z[8])
            self.bevel["r"] = safe_float(z[9])% 360
            self.bevel["p"] = safe_float(z[10]) % 360

            msging=ArmPosition()

        

            #self.bevel["p"]=self.bevel["p"]-self.l2["p"]
            self.l2["p"]=self.l2["p"]-self.l1["p"]
            msging.link1=self.l1["p"]-90
            msging.link2=self.l2["p"]-90
            
            msging.swivel=self.l1["y"]
            
            msging.bevel_pitch=self.bevel['p']

            msging.bevel_roll=self.bevel['r']

            self.armpos.publish(msging)
            print(
    f"L1 [Y:{self.l1['y']:.2f} R:{self.l1['r']:.2f} P:{self.l1['p']:.2f}] hello | "
    f"L2 [Y:{self.l2['y']:.2f} R:{self.l2['r']:.2f} P:{self.l2['p']:.2f}] | "
    f"be [Y:{self.bevel['y']:.2f} R:{self.bevel['r']:.2f} P:{self.bevel['p']:.2f}] | ")




            


        except Exception as e:
            print("IMU parse error:", e)



def main(args=None):
    rclpy.init(args=args)
    node = BNO055Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_calibration()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()