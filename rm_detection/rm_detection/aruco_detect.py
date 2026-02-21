import cv2
import numpy as np
import os

base_dir=os.path.dirname(os.path.abspath(__file__))
calib_dir=os.path.join(base_dir,"config","calibration")

camera_matrix=np.load(os.path.join(calib_dir,"K.npy"))
dist_coeffs=np.load(os.path.join(calib_dir,"dist.npy"))

aruco_dict=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params=cv2.aruco.DetectorParameters()
detector=cv2.aruco.ArucoDetector(aruco_dict,params)

cap=cv2.VideoCapture(4)
marker_length=0.05

objp=np.array([
[-marker_length/2, marker_length/2,0],
[ marker_length/2, marker_length/2,0],
[ marker_length/2,-marker_length/2,0],
[-marker_length/2,-marker_length/2,0]
],dtype=np.float32)

while True:
    ret,frame=cap.read()
    if not ret: break

    corners,ids,_=detector.detectMarkers(frame)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame,corners,ids)

        for i,mid in enumerate(ids.flatten()):
            imgp=corners[i].reshape(4,2).astype(np.float32)
            ok,rvec,tvec=cv2.solvePnP(objp,imgp,camera_matrix,dist_coeffs,flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                cv2.drawFrameAxes(frame,camera_matrix,dist_coeffs,rvec,tvec,0.03)
                c=corners[i].reshape(4,2).mean(axis=0).astype(int)
                dist=float(np.linalg.norm(tvec))
                cv2.putText(frame,f"ID {mid}",(c[0]-40,c[1]-20),cv2.FONT_HERSHEY_SIMPLEX,1.2,(0,255,0),3,cv2.LINE_AA)
                print(f"ID: {mid}")
                print(f"rvec: {rvec.flatten()}")
                print(f"tvec: {tvec.flatten()}")
                print(f"distance(m): {dist}")
                print("------")

    cv2.imshow("aruco",frame)
    if cv2.waitKey(1)==27: break

cap.release()
cv2.destroyAllWindows()
