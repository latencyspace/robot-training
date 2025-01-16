import numpy as np
import cv2
from glob import glob

def calibrate_camera(img_path, chessboard_size=(8, 6)):
    obj_points = [] # 3D 좌표 공간의 점들 (체스보드의 코너들)
    img_points = [] # 이미지 평면의 점들 (체스보드의 코너들)

 # 코너 검출 기준 설정
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

 # 3D 체커보드 좌표 초기화
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1,
2)
 # 이미지 파일 경로들 읽기
    images = glob(img_path + '/*.jpg')
 
    for image in images:
        img = cv2.imread(image)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

 # 체커보드 코너 찾기
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size,
cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK +
cv2.CALIB_CB_NORMALIZE_IMAGE)

        if ret:
            obj_points.append(objp) # 3D 점 추가
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    # 코너 정밀화
            img_points.append(corners2) # 2D 점 추가
    
    # 이미지에 체커보드 코너 그리기
            img = cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
 
        cv2.imshow('img', img) # 이미지 표시
        cv2.waitKey(500) # 0.5초 대기
    cv2.destroyAllWindows()
 # 카메라 캘리브레이션 실행
    ret, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
    return ret, camera_matrix, distortion_coefficients, rvecs, tvecs

if __name__ == "__main__":
 # 체커보드 이미지가 저장된 폴더
    img_path = './calib_img'
    ret, camera_matrix, distortion_coefficients, rvecs, tvecs = calibrate_camera(img_path)

    if ret:
        print("Camera Matrix : \n", camera_matrix)
        print("\nDistortion Coefficients : \n", distortion_coefficients)

 # 카메라 캘리브레이션 결과 저장 경로 설정 및 저장
        np.savez("./camera_calibration.npz", camera_matrix=camera_matrix,
distortion_coefficients=distortion_coefficients, rvecs=rvecs, tvecs=tvecs)