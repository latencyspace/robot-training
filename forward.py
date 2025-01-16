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

def left():
    GPIO.output(AIN1, GPIO.HIGH)
    GPIO.output(AIN2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(60)

    GPIO.output(BIN1, GPIO.HIGH)
    GPIO.output(BIN2, GPIO.LOW)
    pwm_b.ChangeDutyCycle(40)

def right():
    GPIO.output(AIN1, GPIO.HIGH)
    GPIO.output(AIN2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(40)

    GPIO.output(BIN1, GPIO.HIGH)
    GPIO.output(BIN2, GPIO.LOW)
    pwm_b.ChangeDutyCycle(60)

def rotate():
    time.sleep(0.2)
    stop()          # 잠시 멈춤
    time.sleep(0.4)
    
    GPIO.output(AIN1, GPIO.HIGH)
    GPIO.output(AIN2, GPIO.LOW)
    pwm_a.ChangeDutyCycle(30)

    GPIO.output(BIN1, GPIO.LOW)
    GPIO.output(BIN2, GPIO.HIGH)
    pwm_b.ChangeDutyCycle(30)
    
while True: 
    rotate()