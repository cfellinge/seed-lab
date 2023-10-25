import numpy as np
import cv2, time

#Setup camera
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
camera.set(cv2.CAP_PROP_FPS, 40)

prev_frame_time = time.time()

cal_image_count = 0
frame_count = 0

while True:
    ret, frame = camera.read()

    frame_count += 1

    if frame_count == 30:
        cv2.imwrite("../Aruco/cal_image_" + str(cal_image_count) + ".jpeg",frame)
        cal_image_count += 1
        frame_count = 0

    new_frame_time = time.time()
    fps=1/(new_frame_time-prev_frame_time)
    prev_frame_time = new_frame_time
    
    cv2.imshow("overlay",frame)

    k = cv2.waitKey(1) & 0xFF
    if k == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
