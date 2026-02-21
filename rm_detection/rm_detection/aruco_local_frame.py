import cv2
import numpy as np

marker_size = 0.05
ids_needed = [1,2,3]

K = np.array([[600,0,320],
              [0,600,240],
              [0,0,1]], dtype=float)
D = np.zeros((5,1))

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

cap = cv2.VideoCapture(4)

objp = np.array([
    [-marker_size/2,  marker_size/2, 0],
    [ marker_size/2,  marker_size/2, 0],
    [ marker_size/2, -marker_size/2, 0],
    [-marker_size/2, -marker_size/2, 0]
], dtype=np.float32)

def rt_to_T(rvec,tvec):
    R,_ = cv2.Rodrigues(rvec)
    T=np.eye(4)
    T[:3,:3]=R
    T[:3,3]=tvec.reshape(3)
    return T

while True:
    ret,frame=cap.read()
    if not ret: break

    corners,ids,_=detector.detectMarkers(frame)

    poses={}

    if ids is not None:
        ids=ids.flatten()

        for i,id_ in enumerate(ids):
            imgp=corners[i].reshape(-1,2).astype(np.float32)
            ok,rvec,tvec=cv2.solvePnP(objp,imgp,K,D,flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok: continue

            T=rt_to_T(rvec,tvec)
            poses[id_]=T
            cv2.drawFrameAxes(frame,K,D,rvec,tvec,0.03)

        if all(i in poses for i in ids_needed):
            Ta=poses[ids_needed[0]]
            Tb=poses[ids_needed[1]]
            Tc=poses[ids_needed[2]]

            pa=Ta[:3,3]
            pb=Tb[:3,3]
            pc=Tc[:3,3]

            x=pb-pa
            x/=np.linalg.norm(x)

            y_raw=pc-pa
            y=y_raw-np.dot(y_raw,x)*x
            y/=np.linalg.norm(y)

            z=np.cross(x,y)

            T_local=np.eye(4)
            T_local[:3,0]=x
            T_local[:3,1]=y
            T_local[:3,2]=z
            T_local[:3,3]=pa

            print(T_local)

    cv2.imshow("frame",frame)
    if cv2.waitKey(1)==27: break
