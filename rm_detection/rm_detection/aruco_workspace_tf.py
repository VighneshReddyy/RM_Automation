import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

marker_size=0.05
ids_needed=[1,2,3]

K=np.array([[600,0,320],[0,600,240],[0,0,1]],dtype=float)
D=np.zeros((5,1))

aruco_dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params=cv2.aruco.DetectorParameters()
detector=cv2.aruco.ArucoDetector(aruco_dict,params)

objp=np.array([
[-marker_size/2, marker_size/2,0],
[ marker_size/2, marker_size/2,0],
[ marker_size/2,-marker_size/2,0],
[-marker_size/2,-marker_size/2,0]],dtype=np.float32)

def rt_to_T(rvec,tvec):
    Rm,_=cv2.Rodrigues(rvec)
    T=np.eye(4)
    T[:3,:3]=Rm
    T[:3,3]=tvec.reshape(3)
    return T

class ExternalCam(Node):
    def __init__(self):
        super().__init__('external_webcam_aruco')
        self.bridge=CvBridge()
        self.pub=self.create_publisher(Image,'/external_webcam',10)
        self.br=TransformBroadcaster(self)
        self.cap=cv2.VideoCapture(4)
        self.timer=self.create_timer(0.03,self.loop)

    def publish_tf(self,T):
        msg=TransformStamped()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id='camera_link'
        msg.child_frame_id='tf_aruco'

        t=T[:3,3]
        r=R.from_matrix(T[:3,:3]).as_quat()

        msg.transform.translation.x=float(t[0])
        msg.transform.translation.y=float(t[1])
        msg.transform.translation.z=float(t[2])
        msg.transform.rotation.x=float(r[0])
        msg.transform.rotation.y=float(r[1])
        msg.transform.rotation.z=float(r[2])
        msg.transform.rotation.w=float(r[3])

        self.br.sendTransform(msg)

    def loop(self):
        ret,frame=self.cap.read()
        if not ret: return

        msg=self.bridge.cv2_to_imgmsg(frame,'bgr8')
        self.pub.publish(msg)

        corners,ids,_=detector.detectMarkers(frame)
        poses={}

        if ids is not None:
            ids=ids.flatten()
            for i,id_ in enumerate(ids):
                imgp=corners[i].reshape(-1,2).astype(np.float32)
                ok,rvec,tvec=cv2.solvePnP(objp,imgp,K,D,flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if ok:
                    poses[id_]=rt_to_T(rvec,tvec)

            if all(i in poses for i in ids_needed):
                Ta=poses[ids_needed[0]]
                Tb=poses[ids_needed[1]]
                Tc=poses[ids_needed[2]]

                pa=Ta[:3,3]
                pb=Tb[:3,3]
                pc=Tc[:3,3]

                x=pb-pa; x/=np.linalg.norm(x)
                y_raw=pc-pa
                y=y_raw-np.dot(y_raw,x)*x; y/=np.linalg.norm(y)
                z=np.cross(x,y)

                T=np.eye(4)
                T[:3,0]=x
                T[:3,1]=y
                T[:3,2]=z
                T[:3,3]=pa

                self.publish_tf(T)

def main():
    rclpy.init()
    rclpy.spin(ExternalCam())
    rclpy.shutdown()

if __name__=='__main__':
    main()
