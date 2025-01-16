from picamera2 import Picamera2
import cv2

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size":
 (640, 480)}))
picam2.start()

file_number = 1

while True:
    frame = picam2.capture_array()
    frame = cv2.rotate(frame, cv2.ROTATE_180)
    cv2.imshow("camera", frame)

    if cv2.waitKey(1) == ord('c'):
        # 이미지 저장
        cv2.imwrite(f"./calib_img/{file_number}.jpg", frame)
        print(f"Saved {file_number}.jpg")
        file_number += 1
        
    if cv2.waitKey(1) == ord('q'): #q 키를 누르면 종료
        break

picam2.close()
cv2.destroyAllWindows()