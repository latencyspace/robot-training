import numpy as np
import cv2
import time
from picamera2 import Picamera2

def pose_estimation(frame, aruco_dict_type, matrix_coefficients,
distortion_coefficients):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)
    print(f"ids: {ids}")
    
    if ids is not None:
        for i in range(len(ids)):
 # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients, distortion_coefficients)
            print(f"rotation vector: {rvec}")
            print(f"translation vector: {tvec}\n")

            x, y, z = tvec[0][0] * 100 # cm로 변환
            text = f"id: {ids[i][0]} x: {x:.2f}, y: {y:.2f}, z: {z:.2f}"
            cv2.putText(frame, text, (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

 # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners)
    return frame

if __name__ == '__main__':
    aruco_dict_type = cv2.aruco.DICT_5X5_250 # 아루코마커 타입설정

    with np.load('camera_calibration.npz') as data:
        calibration_matrix = data['camera_matrix']
        dist_coeffs = data['distortion_coefficients']
 
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
    picam2.start()
 
    while True:
        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        output = pose_estimation(frame, aruco_dict_type, calibration_matrix, dist_coeffs)

        cv2.imshow('Estimated Pose', output)

        if cv2.waitKey(1) == ord('q'):
            break
    picam2.close()
    cv2.destroyAllWindows()