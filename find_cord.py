from ultralytics import YOLO
import cv2
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size":(640, 480)}))
picam2.start()

def detect_yolo(results):
    boxes = results[0].boxes.xyxy.numpy()
    clss = results[0].boxes.cls.numpy()
    for box, cls in zip(boxes, clss):
        if cls == 0:
            [x1, y1, x2, y2] = box
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            return cx, cy
    return None, None

model = YOLO("yolov8n.pt")

while True:
    frame = picam2.capture_array()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    results = model(frame, verbose=True)
    frame = results[0].plot()
    cx, cy = detect_yolo(results)
    print(cx, cy)
    cv2.imshow("camera", frame)
    cv2.waitKey(1)
 
picam2.close()