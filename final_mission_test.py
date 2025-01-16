import RPi.GPIO as GPIO
import time
from time import sleep
from ultralytics import YOLO
import cv2
from picamera2 import Picamera2
import numpy as np

TRIG = 23
ECHO = 24
AIN1 = 17
AIN2 = 27
PWMA = 18
BIN1 = 5
BIN2 = 6
PWMB = 13
STBY = 25
BUZZER_PIN = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(AIN1, GPIO.OUT)
GPIO.setup(AIN2, GPIO.OUT)
GPIO.setup(PWMA, GPIO.OUT)
GPIO.setup(BIN1, GPIO.OUT)
GPIO.setup(BIN2, GPIO.OUT)
GPIO.setup(PWMB, GPIO.OUT)
GPIO.setup(STBY, GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

GPIO.output(TRIG, False)
GPIO.output(STBY, GPIO.HIGH)

pwm_a = GPIO.PWM(PWMA, 1000)
pwm_b = GPIO.PWM(PWMB, 1000)
pwm_a.start(0)
pwm_b.start(0)

def buzz():
    pwm = GPIO.PWM(BUZZER_PIN, 1000)
    pwm.start(50)

    pwm.ChangeDutyCycle(50)
    time.sleep(0.1)
      
    pwm.stop()

def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    print(f"ids: {ids}")
    
    if ids is not None:
        for i in range(len(ids)):
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients, distortion_coefficients)
            print(f"rotation vector: {rvec}")
            print(f"translation vector: {tvec}\n")

            x, y, z = tvec[0][0] * 100 # cm로 변환
            text = f"id: {ids[i][0]} x: {x:.2f}, y: {y:.2f}, z: {z:.2f}"
            cv2.putText(frame, text, (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            cv2.aruco.drawDetectedMarkers(frame, corners)
    return frame, corners, ids  # corners 추가 반환



def stop():
    GPIO.output(AIN1, False)
    GPIO.output(AIN2, False)
    GPIO.output(BIN1, False)
    GPIO.output(BIN2, False)
    GPIO.output(PWMA, False)
    GPIO.output(PWMB, False)

def forward():
    GPIO.output(AIN1, GPIO.HIGH)
    GPIO.output(AIN2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(70)
    
    GPIO.output(BIN1, GPIO.HIGH)
    GPIO.output(BIN2, GPIO.LOW)
    pwm_b.ChangeDutyCycle(70)

def right():
    GPIO.output(AIN1, GPIO.HIGH)
    GPIO.output(AIN2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(60)

    GPIO.output(BIN1, GPIO.HIGH)
    GPIO.output(BIN2, GPIO.LOW)
    pwm_b.ChangeDutyCycle(40)

def left():
    GPIO.output(AIN1, GPIO.HIGH)
    GPIO.output(AIN2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(40)

    GPIO.output(BIN1, GPIO.HIGH)
    GPIO.output(BIN2, GPIO.LOW)
    pwm_b.ChangeDutyCycle(60)

def rotate():
    time.sleep(0.1)
    stop()          # 잠시 멈춤
    time.sleep(0.4)
    
    GPIO.output(AIN1, GPIO.HIGH)
    GPIO.output(AIN2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(30)

    GPIO.output(BIN1, GPIO.LOW)
    GPIO.output(BIN2, GPIO.HIGH)
    pwm_b.ChangeDutyCycle(30)


picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)}))
picam2.start()

if __name__ == '__main__':
    buzz()
    aruco_dict_type = cv2.aruco.DICT_5X5_250 # 아루코마커 타입설정

    with np.load('camera_calibration.npz') as data:
        calibration_matrix = data['camera_matrix']
        dist_coeffs = data['distortion_coefficients']

marker_sequence = [2, 1, 0, 3, 4]
current_target_index = 0

while True:
    frame = picam2.capture_array()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    output, corners, ids = pose_estimation(frame, aruco_dict_type, calibration_matrix, dist_coeffs)

    if ids is not None:
        for i in range(len(ids)):
            target_id = marker_sequence[current_target_index]
            if target_id in ids:
                _, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, calibration_matrix, dist_coeffs)
                x, y, z = tvec[0][0] * 100

                if target_id in [1, 4]:  # 1과 4일 때
                    if z > 11:
                        if x < -5:
                            left()
                        elif x > 5:
                            right()
                        else:
                            forward()
                    else:
                        stop()
                        buzz()
                        sleep(1)  # 잠시 멈춤
                        current_target_index = (current_target_index + 1) % len(marker_sequence)

                elif target_id in [2, 0, 3]:  # 0과 3일 때
                    if z > 20:
                        if x < -9:
                            left()
                        elif x > 9:
                            right()
                        else:
                            forward()
                    else:
                        current_target_index = (current_target_index + 1) % len(marker_sequence)
            else:
                rotate()
    else:
        rotate()

    # Check if output is a valid image before displaying
    if isinstance(output, np.ndarray):
        cv2.imshow('Estimated Pose', output)
    else:
        print("Output is not a valid image")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.close()
GPIO.cleanup()
